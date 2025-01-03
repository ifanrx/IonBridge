#include "logging.h"

#include <arpa/inet.h>  // For inet_addr() and inet_ntoa_r()
#include <netdb.h>
#include <netinet/in.h>  // For sockaddr_in structure
#include <stdio.h>       // Provides vsnprintf function
#include <string.h>      // For strlen function
#include <sys/socket.h>  // Includes socket-related functions like socket(), sendto(), close()

#include <cstdint>
#include <cstring>

#include "esp_log.h"
#include "freertos/ringbuf.h"
#include "sdkconfig.h"

namespace {

static const char* TAG = "Logging";

#ifdef CONFIG_ENABLE_LOG_STDOUT
#define ENABLE_LOG_STDOUT
#endif

}  // namespace

uint8_t LogCollector::ring_buffer_[RING_BUFFER_SIZE];

LogCollector::LogCollector() {
  buf_handle_ = xRingbufferCreateStatic(sizeof(LogCollector::ring_buffer_),
                                        RINGBUF_TYPE_BYTEBUF,
                                        LogCollector::ring_buffer_, &buf_);
  if (buf_handle_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create ring buffer");
  }
}

LogCollector::~LogCollector() {
  if (buf_handle_) {
    vRingbufferDelete(buf_handle_);
  }
}

void LogCollector::Push(const char* data, size_t size) {
  BaseType_t ret =
      xRingbufferSend(buf_handle_, static_cast<const void*>(data), size, 0);
  if (ret != pdTRUE) {
    size_t item_size = 0;
    char* item = static_cast<char*>(
        xRingbufferReceive(buf_handle_, &item_size, pdMS_TO_TICKS(0)));
    if (item != nullptr) {
      vRingbufferReturnItem(buf_handle_, item);
      xRingbufferSend(buf_handle_, static_cast<const void*>(data), size, 0);
    }
  }
}

void LogCollector::Pop(char* data, size_t* size) {
  size_t item_size = 0;
  char* item = static_cast<char*>(
      xRingbufferReceive(buf_handle_, &item_size, pdMS_TO_TICKS(100)));
  if (item != nullptr) {
    size_t data_size = (item_size > *size) ? *size : item_size;
    memcpy(data, item, data_size);
    *size = data_size;
    vRingbufferReturnItem(buf_handle_, item);
  } else {
    *size = 0;
  }
}
