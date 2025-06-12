#include "logging.h"

#include <arpa/inet.h>  // For inet_addr() and inet_ntoa_r()
#include <netdb.h>
#include <netinet/in.h>  // For sockaddr_in structure
#include <stdio.h>       // Provides vsnprintf function
#include <string.h>      // For strlen function
#include <sys/socket.h>  // Includes socket-related functions like socket(), sendto(), close()

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

LogCollector::LogCollector() {
  buf_handle_ = xRingbufferCreateStatic(sizeof(LogCollector::ring_buffer_),
                                        RINGBUF_TYPE_BYTEBUF,
                                        LogCollector::ring_buffer_, &buf_);
  if (buf_handle_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create ring buffer");
  }

  mutex_ = xSemaphoreCreateMutex();
  if (mutex_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create mutex");
  }
}

LogCollector::~LogCollector() {
  if (buf_handle_) {
    vRingbufferDelete(buf_handle_);
  }
  if (mutex_) {
    vSemaphoreDelete(mutex_);
  }
}

bool LogCollector::Push(const char* data, size_t size) {
  if (!buf_handle_ || !data || size == 0) return false;

  // Try to acquire lock with timeout
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(11)) != pdTRUE) {
    return false;  // Failed to acquire lock
  }

  BaseType_t ret = xRingbufferSend(buf_handle_, static_cast<const void*>(data),
                                   size, pdMS_TO_TICKS(PUSH_TIMEOUT_MS));
  if (ret != pdTRUE) {
    // Make space by removing oldest items
    TryFreeSpace(size);
    ret = xRingbufferSend(buf_handle_, static_cast<const void*>(data), size,
                          pdMS_TO_TICKS(PUSH_TIMEOUT_MS));
  }

  xSemaphoreGive(mutex_);  // Release the lock
  return (ret == pdTRUE);
}

bool LogCollector::Pop(char* data, size_t* size) {
  if (!buf_handle_ || !data || !size || *size == 0) return false;

  // Try to acquire lock with timeout
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(23)) != pdTRUE) {
    return false;  // Failed to acquire lock
  }

  bool ret = false;
  size_t item_size = 0;
  char* item = static_cast<char*>(xRingbufferReceive(
      buf_handle_, &item_size, pdMS_TO_TICKS(POP_TIMEOUT_MS)));

  if (item != nullptr) {
    // Prevent buffer overflow
    size_t data_size = (item_size > *size) ? *size : item_size;
    memcpy(data, item, data_size);
    *size = data_size;
    vRingbufferReturnItem(buf_handle_, item);
    ret = true;
  } else {
    *size = 0;
    ret = false;  // No data available
  }

  xSemaphoreGive(mutex_);  // Release the lock
  return ret;
}

void LogCollector::TryFreeSpace(size_t needed_size) {
  if (buf_handle_ == nullptr) return;

  size_t freed = 0;
  while (freed < needed_size) {
    size_t item_size = 0;
    char* item = static_cast<char*>(
        xRingbufferReceive(buf_handle_, &item_size, PUSH_TIMEOUT_MS));

    if (item == nullptr) break;

    freed += item_size;
    vRingbufferReturnItem(buf_handle_, item);
  }
}
