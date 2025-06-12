#ifndef LOGGING_H_
#define LOGGING_H_

#include "freertos/ringbuf.h"

class LogCollector {
 public:
  static constexpr size_t RING_BUFFER_SIZE = 256;
  static constexpr TickType_t POP_TIMEOUT_MS = 100;
  static constexpr TickType_t PUSH_TIMEOUT_MS = 0;

  /**
   * @brief Access the singleton instance
   * @return Reference to the global LogCollector instance
   */
  static LogCollector &GetInstance() {
    static LogCollector instance;  // Guaranteed to be destroyed.
                                   // Instantiated on first use.
    return instance;
  }

  /**
   * @brief Push data into the ring buffer
   * @param data Pointer to data to store
   * @param size Size of the data in bytes
   * @return true if successful, false if failed
   * @thread-safety Thread-safe
   */
  bool Push(const char *data, size_t size);

  /**
   * @brief Pop data from the ring buffer
   * @param data Buffer to store retrieved data
   * @param size In: max buffer size; Out: actual data size
   * @return true if data retrieved, false if buffer empty
   * @thread-safety Thread-safe
   */
  bool Pop(char *data, size_t *size);

 private:
  uint8_t ring_buffer_[RING_BUFFER_SIZE];
  RingbufHandle_t buf_handle_ = nullptr;
  StaticRingbuffer_t buf_;
  SemaphoreHandle_t mutex_ = NULL;

  /**
   * @brief Try to free space in the buffer
   * @param needed_size Size in bytes to try freeing
   */
  void TryFreeSpace(size_t needed_size);

  LogCollector();
  ~LogCollector();

  // Delete copy constructor and assignment operator
  LogCollector(const LogCollector &) = delete;
  LogCollector &operator=(const LogCollector &) = delete;
};

#endif
