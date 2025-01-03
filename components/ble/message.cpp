#include "message.h"

#include "app.h"
#include "ble.h"
#include "esp_log.h"

static const char *TAG = "Message";

MessageHandler::MessageHandler(App *app) : app_(app) {
  handlers_ = new MessageFragHandler[MAX_CONCURRENT_CONN];
  for (size_t i = 0; i < MAX_CONCURRENT_CONN; i++) {
    handlers_[i].free = true;
    handlers_[i].data = new uint8_t[MAX_MESSAGE_SIZE];
  }
}

MessageHandler::~MessageHandler() { delete[] handlers_; }

void MessageHandler::setPeerMTU(uint16_t peerMTU) { peerMTU_ = peerMTU; }

void MessageHandler::handle_syn(Message &msg) {
  // Get the size of the payload from the message
  uint32_t payload_size = msg.getSize();

  // Check if the app has the corresponding service in the BLE service scope
  if (app_->HasSrv(msg.getService(), ServiceScope::SERVICE_SCOPE_BLE)) {
    ESP_LOGD(TAG, "Sync App Service: %d", msg.getService());

    // Clear the buffer if it is not empty
    if (!requestBuffer_.empty()) {
      requestBuffer_.clear();
    }

    // Insert the message payload into the buffer
    requestBuffer_.insert(requestBuffer_.end(), msg.getPayload(),
                          msg.getPayload() + payload_size);

    // Resize the buffer if it exceeds the maximum message size
    if (requestBuffer_.size() > MAX_MESSAGE_SIZE) {
      requestBuffer_.resize(MAX_MESSAGE_SIZE);
    }

    // Send a success response before processing all data
    uint8_t data[] = {0};
    sendResponse(msg, data, sizeof(data), BLE_SYN_ACK);
  }
}

void MessageHandler::handle_syn_ack(Message &msg) {
  // Ignore unexpected SYN-ACK messages
  ESP_LOGW(TAG, "Unexpected SYN-ACK message received; ignoring.");
}

void MessageHandler::handle_fin(Message &msg) {
  // Ignore unexpected FIN messages
  ESP_LOGW(TAG, "Unexpected FIN message received; ignoring.");
}

void MessageHandler::handle_ack(Message &msg) {
  ESP_LOGD(TAG, "BLE ACK Message received, service: 0x%02x", msg.getService());
  // Extracting the 4th and 5th bits for FRAG FLAGS
  uint8_t frag_flags = (msg.getFlags() >> 3) & 0x3;
  MessageFragHandler *handler = getHandlerById(msg.getID());
  uint32_t payload_size = msg.getSize();
  size_t length = 0;

  // If there's no existing handler for this ID, create a new one
  if (handler == NULL) {
    handler = getFreeHandler();
    if (handler == NULL) {
      return;
    }
    handler->id = msg.getID();
    handler->expected_sequence = 0;
    handler->size = 0;
  }

  if (IS_FIRST_FRAG(frag_flags)) {
    // This is the first fragment, reset the buffer
    handler->size = 0;
    handler->expected_sequence = 0;
    handler->free = false;
  }

  if (msg.getSequence() != handler->expected_sequence &&
      frag_flags == FRAG_LAST) {
    ESP_LOGE(TAG, "Recieved unexpected sequence number: %d, expected: %d",
             msg.getSequence(), handler->expected_sequence);
    return;
  }

  // If the sequence number matches the expected sequence number, store the
  // fragment
  if (handler->size + payload_size <= MAX_MESSAGE_SIZE) {
    ESP_LOGD(TAG, "Copying fragment %d of size %" PRIu32 " to buffer",
             msg.getSequence(), payload_size);
    memcpy(handler->data + handler->size, msg.getPayload(), payload_size);
    handler->size += payload_size;
    handler->expected_sequence++;
  }

  if (!IS_LAST_FRAG(frag_flags)) {
    // More fragments are expected, do nothing
    return;
  }

  requestBuffer_.insert(requestBuffer_.end(), handler->data,
                        handler->data + handler->size);
  if (requestBuffer_.size() > MAX_MESSAGE_SIZE) {
    requestBuffer_.resize(MAX_MESSAGE_SIZE);
  }
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, requestBuffer_.data(), requestBuffer_.size(),
                           ESP_LOG_DEBUG);

  if (app_->HasSrv(msg.getService(), ServiceScope::SERVICE_SCOPE_BLE)) {
    length = app_->ExecSrvFromBle(msg.getService(), requestBuffer_.data(),
                                  requestBuffer_.size(), handler->data,
                                  MAX_MESSAGE_SIZE);
    sendResponse(msg, handler->data, length, BLE_FIN);
    ESP_LOGD(TAG, "BLE Message sent, service: 0x%02x, status: %d",
             msg.getService(), handler->data[0]);
  }
  requestBuffer_.clear();
  handler->free = true;
}

void MessageHandler::sendResponse(Message &msg, const uint8_t *data,
                                  size_t dataLength, uint8_t tcpFlag) const {
  const size_t chunkSize = MIN(peerMTU_, BUFSIZE) - sizeof(BLEHeader);
  std::vector<Message> messages;

  size_t offset = 0;
  size_t sequence = 0;
  uint8_t fragFlags = dataLength > chunkSize ? FRAG_FIRST : FRAG_NONE;

  ESP_LOGD(TAG, "Sending data: %d bytes", dataLength);
  while (offset < dataLength) {
    size_t currChunkSize = MIN(chunkSize, dataLength - offset);
    uint8_t flags = IS_LAST_FRAG(fragFlags) ? tcpFlag : BLE_ACK;
    flags |= (fragFlags << 3);
    ESP_LOGD(TAG, "Sending chunk: %d bytes, flags: 0x%02x, offset: %d",
             currChunkSize, flags, offset);
    messages.emplace_back(msg.getVersion(), msg.getID(), -msg.getService(),
                          sequence, flags, data + offset, currChunkSize);

    offset += currChunkSize;
    // Update fragFlags for next iteration
    if (offset < dataLength) {
      // More data left to process
      if (offset + chunkSize < dataLength) {
        // The next chunk won't be the last one
        fragFlags = FRAG_MORE;
      } else {
        // The next chunk is the last one
        fragFlags = FRAG_LAST;
      }
    }

    sequence++;
  }

  for (const Message &msg : messages) {
    std::vector<uint8_t> serialized_data = msg.serialize();
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, serialized_data.data(),
                             serialized_data.size(), ESP_LOG_DEBUG);
    if (serialized_data.empty()) {
      ESP_LOGW(TAG, "Serialization failed");
      return;
    }

    ble_notify(serialized_data.data(), serialized_data.size());
  }
}

MessageFragHandler *MessageHandler::getHandlerById(uint8_t id) {
  for (int i = 0; i < MAX_CONCURRENT_CONN; i++) {
    if (handlers_[i].free == false && handlers_[i].id == id) {
      return &handlers_[i];
    }
  }
  return nullptr;
}

MessageFragHandler *MessageHandler::getFreeHandler() {
  for (int i = 0; i < MAX_CONCURRENT_CONN; i++) {
    if (handlers_[i].free == true) {
      handlers_[i].free = false;  // mark the slot as used
      return &handlers_[i];
    }
  }
  return nullptr;
}
