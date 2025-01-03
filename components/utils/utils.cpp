#include "utils.h"

#include <cstring>
#include <iomanip>
#include <sstream>

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "ionbridge.h"

#define HASH_LENGTH 32
// CRC-16-CCITT Polynomial
#define CRC16_POLY 0x1021

static const char *TAG = "Utils";

bool validate_partition_hash(const uint8_t *hash, size_t length) {
  if (length != HASH_LENGTH) {
    ESP_LOGW(TAG, "Incorrect hash length: want %d, got %d", HASH_LENGTH,
             length);
    return false;
  }

  uint8_t runningPartitionHash[HASH_LENGTH];
  const esp_partition_t *partition = esp_ota_get_running_partition();
  ESP_RETURN_FALSE_ON_ERROR(
      esp_partition_get_sha256(partition, runningPartitionHash),
      "esp_partition_get_sha256");

  for (size_t i = 0; i < HASH_LENGTH; i++) {
    if (runningPartitionHash[i] != hash[i]) {
      ESP_LOGW(TAG, "Hash mismatch at index %zu: want %02x, got %02x", i,
               hash[i], runningPartitionHash[i]);
      return false;
    }
  }
  return true;
}

std::string to_hex_string(const uint8_t *data, size_t len) {
  std::stringstream ss;
  for (size_t i = 0; i < len; i++) {
    ss << std::hex << std::setw(2) << std::setfill('0')
       << static_cast<int>(data[i]);
  }
  return ss.str();
}

uint32_t bytes_to_u32(const uint8_t *data) {
  return (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
}

void int_to_bytes(uint8_t *bytes, int num) {
  bytes[0] = (num >> 24) & 0xFF;
  bytes[1] = (num >> 16) & 0xFF;
  bytes[2] = (num >> 8) & 0xFF;
  bytes[3] = num & 0xFF;
}

// Function to calculate CRC16 dynamically
uint16_t crc16(uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;  // Common initial value for CRC-16-CCITT
  for (uint8_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i]
           << 8;  // Data is XORed into the most significant byte
    for (int j = 0; j < 8; j++) {  // Process each bit
      if (crc & 0x8000) {  // If the leftmost (most significant) bit is set
        crc = (crc << 1) ^ CRC16_POLY;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void BytesWriter::write_uint16(uint16_t num) {
  data_[offset_] = (num >> 8) & 0xff;
  data_[offset_ + 1] = num & 0xff;
  offset_ += 2;
}

void BytesWriter::write_uint8(uint8_t num) {
  data_[offset_] = num;
  offset_ += 1;
}

size_t BytesWriter::get_size() { return offset_; }

void log_memory_info() {
  uint32_t free_heap_size = esp_get_free_heap_size();
  uint32_t free_internal_heap_size = esp_get_free_internal_heap_size();
  uint32_t minimum_free_heap_size = esp_get_minimum_free_heap_size();

  ESP_LOGI(TAG,
           "Memory Statistics: Free Heap Size = %" PRIu32
           ", Free Internal Heap Size = %" PRIu32
           ", Minimum Free Heap Size = %" PRIu32,
           free_heap_size, free_internal_heap_size, minimum_free_heap_size);
}

void hex_to_uint8_array(const char *hex, uint8_t *arr, size_t arr_len) {
  for (size_t i = 0; i < arr_len; i++) {
    // Extract 2 characters (1 byte) at a time from the hex string
    char temp[3] = {
        0};  // Temporary buffer for 2 hex characters + null terminator
    strncpy(temp, hex + (i * 2), 2);

    // Convert the hex substring to a uint8_t
    arr[i] = (uint8_t)strtoul(temp, NULL, 16);
  }
}

MovingAverage::MovingAverage() : index_(0), count_(0), sum_(0) {
  buffer_.fill(0);
}

uint8_t MovingAverage::AddValue(uint8_t value) {
  sum_ -= buffer_[index_];
  buffer_[index_] = value;
  sum_ += value;
  index_ = (index_ + 1) % MAX_SIZE;
  if (count_ < MAX_SIZE) {
    count_++;
  }
  return GetAverage();
}

uint8_t MovingAverage::GetAverage() const {
  if (count_ == 0) {
    return 0;
  }
  return static_cast<uint8_t>(sum_ / count_);
}

void MovingAverage::Reset() {
  index_ = 0;
  count_ = 0;
  sum_ = 0;
  buffer_.fill(0);
}
