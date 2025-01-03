#ifndef LOGGING_H_
#define LOGGING_H_

#include "freertos/ringbuf.h"

#define RING_BUFFER_SIZE 256

class LogCollector {
 public:
  // Access the singleton instance
  static LogCollector &GetInstance() {
    static LogCollector instance;  // Guaranteed to be destroyed.
                                   // Instantiated on first use.
    return instance;
  }

  void Push(const char *data, size_t size);
  void Pop(char *data, size_t *size);

 private:
  static uint8_t ring_buffer_[RING_BUFFER_SIZE];
  RingbufHandle_t buf_handle_ = nullptr;
  StaticRingbuffer_t buf_;

  LogCollector();
  ~LogCollector();

  // Delete copy constructor and assignment operator
  LogCollector(const LogCollector &) = delete;
  LogCollector &operator=(const LogCollector &) = delete;
};

#endif
