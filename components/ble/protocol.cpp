#include "protocol.h"

#include <numeric>

#include "esp_log.h"
#include "ionbridge.h"
#include "nvs_namespace.h"
#include "service.h"

static const char *TAG = "Protocol";

uint32_t get_ble_size_t(ble_size_t b, uint8_t version) {
  uint32_t result = 0;

  switch (version) {
    case 0: {
      result |= ((uint32_t)b.bytes[0]) << 16;
      result |= ((uint32_t)b.bytes[1]) << 8;
      result |= ((uint32_t)b.bytes[2]);
      break;
    }
    case 1: {
      result |= ((uint32_t)b.bytes[2]) << 16;
      result |= ((uint32_t)b.bytes[1]) << 8;
      result |= ((uint32_t)b.bytes[0]);
      break;
    }
  }

  return result;
}

ble_size_t set_ble_size_t(uint32_t value, uint8_t version) {
  ble_size_t b;

  switch (version) {
    case 0: {
      b.bytes[0] = (value >> 16) & 0xFF;
      b.bytes[1] = (value >> 8) & 0xFF;
      b.bytes[2] = value & 0xFF;
      break;
    }
    case 1: {
      b.bytes[2] = (value >> 16) & 0xFF;
      b.bytes[1] = (value >> 8) & 0xFF;
      b.bytes[0] = value & 0xFF;
      break;
    }
  }

  return b;
}

uint8_t calcChecksum(BLEHeader *header) {
  const uint8_t *begin = reinterpret_cast<const uint8_t *>(header);
  const uint8_t *end =
      begin + sizeof(BLEHeader) - 1;  // excluding checksum byte
  return std::accumulate(begin, end, static_cast<uint8_t>(0));
}

bool validateChecksum(BLEHeader *header) {
  uint8_t expected_checksum = header->checksum;
  // the checksum field itself should not influence the result.
  header->checksum = 0;
  uint8_t calculated_checksum = calcChecksum(header);
  header->checksum = expected_checksum;
  return calculated_checksum == expected_checksum;
}

esp_err_t MessageFactory::createMessage(const uint8_t *data, size_t dataLength,
                                        Message **msg) {
  if (data == nullptr || msg == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  size_t max_header_size = sizeof(BLEHeader);
  // Validate the length of the provided byte array.
  if (dataLength < max_header_size) {
    // The byte array isn't even long enough to contain the header.
    return ESP_ERR_INVALID_SIZE;
  }

  BLEHeader header;
  memcpy(&header, data, max_header_size);
  if (!validateChecksum(&header)) {
    // Invalid checksum.
    return ESP_ERR_INVALID_CRC;
  }

  // Check if the remaining length in bytes is consistent with the size field of
  // the header. Assuming ble_size_t is a typedef for some integer type:
  uint32_t payload_size = get_ble_size_t(header.size, header.version);
  if (dataLength < max_header_size + payload_size) {
    // Not enough data to copy the complete payload.
    return ESP_ERR_INVALID_SIZE;
  }

  *msg = new Message(header, data + max_header_size, payload_size);
  if (msg == nullptr) {
    return ESP_ERR_NO_MEM;
  }
  if (!(*msg)->validate()) {
    ESP_LOGW(TAG, "Received invalid message");
    return ESP_FAIL;
  }
  return ESP_OK;
}

Message::Message(BLEHeader header, const uint8_t *payload, size_t payloadSize) {
  if (payloadSize > BUFSIZE) {
    payloadSize = BUFSIZE;
  }
  header_ = header;
  header_.size = set_ble_size_t(payloadSize, header.version);
  header_.checksum = calcChecksum(&header_);
  payload_ = new uint8_t[payloadSize];
  memcpy(payload_, payload, payloadSize);
}

Message::Message(uint8_t version, uint8_t id, int8_t service, uint8_t sequence,
                 uint8_t flags, const uint8_t *data, size_t dataLength) {
  if (dataLength > BUFSIZE) {
    dataLength = BUFSIZE;
  }
  header_ = {.version = version,
             .id = id,
             .service = service,
             .sequence = sequence,
             .flags = flags,
             .size = set_ble_size_t(dataLength, version),
             .checksum = 0};
  header_.checksum = calcChecksum(&header_);
  payload_ = new uint8_t[dataLength];
  memcpy(payload_, data, dataLength);
}

Message::Message(Message &&other) noexcept
    : payload_(other.payload_), header_(other.header_) {
  // so that other doesn't delete payload_ on destruction
  other.payload_ = nullptr;
}

Message &Message::operator=(Message &&other) noexcept {
  if (this != &other) {  // self-assignment check
    delete[] payload_;   // free existing resource

    // steal resources from other
    header_ = other.header_;
    payload_ = other.payload_;

    other.payload_ = nullptr;  // set to nullptr to avoid double free
  }
  return *this;
}

Message::~Message() { delete[] payload_; }

bool Message::validate() {
  if (!token_required((ServiceCommand)header_.service)) {
    return true;
  }

  uint8_t incoming_token = payload_[0], token;
  ESP_RETURN_FALSE_ON_ERROR(NVSGetAuthToken(&token), "NVSGetAuthToken");
  bool valid = incoming_token == token;
  if (!valid) {
    ESP_LOGW(TAG, "Token mismatch, expected %d, got %d, service: %d", token,
             incoming_token, header_.service);
  }
  size_t payload_size = get_ble_size_t(header_.size, header_.version);
  for (size_t i = 1; i < payload_size; i++) {
    payload_[i - 1] = payload_[i];
  }
  header_.size = set_ble_size_t(payload_size - 1, header_.version);
  return valid;
}

uint8_t Message::getVersion() const { return header_.version; }

uint8_t Message::getID() const { return header_.id; }

int8_t Message::getService() const { return header_.service; }

uint8_t Message::getFlags() const { return header_.flags; }

uint8_t Message::getSequence() const { return header_.sequence; }

uint32_t Message::getSize() const {
  return get_ble_size_t(header_.size, header_.version);
}

uint8_t Message::getChecksum() const { return header_.checksum; }

uint8_t *Message::getPayload() { return payload_; }

std::vector<uint8_t> Message::serialize() const {
  std::vector<uint8_t> buffer;
  size_t headerSize = sizeof(BLEHeader);
  uint32_t payloadSize = get_ble_size_t(header_.size, header_.version);
  buffer.resize(headerSize + payloadSize);

  memcpy(buffer.data(), &header_, headerSize);
  if (payloadSize > 0) {
    memcpy(buffer.data() + headerSize, payload_, payloadSize);
  }
  return buffer;
}

void process_message(Message &msg, IMessageHandler &handler) {
  ESP_LOGD(
      TAG,
      "BLEHeader version: %d, id: %d, service: %d, sequence: %d, flags: %d,"
      " size: %" PRIu32 ", checksum: %d",
      msg.getVersion(), msg.getID(), msg.getService(), msg.getSequence(),
      msg.getFlags(), msg.getSize(), msg.getChecksum());
  // Extracting the last 3 bits for FLAGS
  uint8_t tcp_flags = msg.getFlags() & 0x7;
  // Extracting the 4th and 5th bits for FRAG FLAGS
  uint8_t frag_flags = (msg.getFlags() >> 3) & 0x3;

  switch (tcp_flags) {
    case BLE_SYN:
      if (frag_flags != FRAG_NONE) {
        break;
      }
      handler.handle_syn(msg);
      break;
    case BLE_SYN_ACK:
      handler.handle_syn_ack(msg);
      break;
    case BLE_ACK:
      handler.handle_ack(msg);
      break;
    case BLE_FIN:
      handler.handle_fin(msg);
      break;
  }
}
