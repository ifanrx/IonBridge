#ifndef H_UTILS_
#define H_UTILS_

#include <stddef.h>
#include <stdint.h>

#include <array>
#include <cstdint>
#include <string>

#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/task.h"      // IWYU pragma: keep

#define INCREMENTAL_AVERAGE(old_avg, new_val, count)                           \
  (old_avg = old_avg +                                                         \
             (static_cast<int32_t>(new_val) - static_cast<int32_t>(old_avg)) / \
                 static_cast<float>(count))

#ifdef __cpluscplus
extern "C" {
#endif

// static char global_log_buffer[1024];
// static int global_log_buffer_position = 0;

#define DELAY_MS(ms) vTaskDelay(pdMS_TO_TICKS(ms))
bool validate_partition_hash(const uint8_t *hash, size_t length);
std::string to_hex_string(const uint8_t *data, size_t len);
uint32_t bytes_to_u32(const uint8_t *data);
void int_to_bytes(uint8_t *bytes, int num);
uint16_t crc16(uint8_t *data, uint8_t length);

#define EMPLACE_BACK_INT64(payload, value)                                \
  do {                                                                    \
    (payload).emplace_back(static_cast<uint8_t>((value) & 0xFF));         \
    (payload).emplace_back(static_cast<uint8_t>(((value) >> 8) & 0xFF));  \
    (payload).emplace_back(static_cast<uint8_t>(((value) >> 16) & 0xFF)); \
    (payload).emplace_back(static_cast<uint8_t>(((value) >> 24) & 0xFF)); \
    (payload).emplace_back(static_cast<uint8_t>(((value) >> 32) & 0xFF)); \
    (payload).emplace_back(static_cast<uint8_t>(((value) >> 40) & 0xFF)); \
    (payload).emplace_back(static_cast<uint8_t>(((value) >> 48) & 0xFF)); \
    (payload).emplace_back(static_cast<uint8_t>(((value) >> 56) & 0xFF)); \
  } while (0)

#define EMPLACE_BACK_INT32(payload, value)                                \
  do {                                                                    \
    (payload).emplace_back(static_cast<uint8_t>((value) & 0xFF));         \
    (payload).emplace_back(static_cast<uint8_t>(((value) >> 8) & 0xFF));  \
    (payload).emplace_back(static_cast<uint8_t>(((value) >> 16) & 0xFF)); \
    (payload).emplace_back(static_cast<uint8_t>(((value) >> 24) & 0xFF)); \
  } while (0)

#define EMPLACE_BACK_INT16(payload, value)                               \
  do {                                                                   \
    (payload).emplace_back(static_cast<uint8_t>((value) & 0xFF));        \
    (payload).emplace_back(static_cast<uint8_t>(((value) >> 8) & 0xFF)); \
  } while (0)

void log_memory_info();
void hex_to_uint8_array(const char *hex, uint8_t *array, size_t size);

#ifdef __cpluscplus
}
#endif

class BytesWriter {
 public:
  BytesWriter(uint8_t *data) : data_(data) {};
  // 以大端序写入
  void write_uint16(uint16_t num);
  void write_uint8(uint8_t num);
  size_t get_size();

 private:
  size_t offset_ = 0;
  uint8_t *data_ = nullptr;
};

/**
 * @class MovingAverage
 * @brief A class that calculates the moving average of the last 10 values.
 */
class MovingAverage {
 public:
  /**
   * @brief Constructs a MovingAverage10 object.
   */
  MovingAverage();

  /**
   * @brief Adds a new value to the moving average calculation.
   * @param value The value to be added.
   * @return The current moving average.
   */
  uint8_t AddValue(uint8_t value);

  uint8_t GetAverage() const;

  size_t GetCount() const { return count_; }

  void Reset();

 private:
  static constexpr size_t MAX_SIZE = 256;
  std::array<uint8_t, MAX_SIZE> buffer_;
  size_t index_;
  size_t count_;
  uint16_t sum_;
};

#endif  // !H_UTILS_
