#include "utils.h"

#include <sys/time.h>

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <ios>
#include <sstream>
#include <string>

#include "esp_log.h"
#include "esp_system.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"  // IWYU pragma: keep

#define HASH_LENGTH 32
// CRC-16-CCITT Polynomial
#define CRC16_POLY 0x1021

static const char *TAG = "Utils";

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

void hexlify(const uint8_t *data, size_t len, char *out, size_t out_len) {
  for (size_t i = 0; i < len; i++) {
    snprintf(out + (i * 2), out_len - (i * 2), "%02x", data[i]);
  }
}

uint32_t get_epoch_timestamp() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return static_cast<uint32_t>(tv.tv_sec);
}

void sync_system_time(uint64_t timestamp) {
  struct timeval tv;
  tv.tv_sec = timestamp;
  tv.tv_usec = 0;

  settimeofday(&tv, nullptr);
}

bool resolve_hostname(const char *hostname, in_addr_t *addr, bool is_udp) {
  struct addrinfo hints, *res, *p;
  memset(&hints, 0, sizeof(hints));

  hints.ai_family = AF_INET;  // Only IPv4
  if (is_udp) {
    hints.ai_socktype = SOCK_DGRAM;   // Only UDP
    hints.ai_protocol = IPPROTO_UDP;  // Only UDP
  } else {
    hints.ai_socktype = SOCK_STREAM;  // Only TCP
    hints.ai_protocol = IPPROTO_TCP;  // Only TCP
  }

  int err = getaddrinfo(hostname, nullptr, &hints, &res);
  if (err != 0) {
    ESP_LOGE(TAG, "getaddrinfo failed for %s: %d", hostname, err);
    if (res) {
      freeaddrinfo(res);
    }
    return false;
  }
  if (res == nullptr) {
    ESP_LOGE(TAG, "getaddrinfo returned no results for: %s", hostname);
    return false;
  }

  if (addr == nullptr) {
    freeaddrinfo(res);
    return true;
  }

  for (p = res; p != nullptr; p = p->ai_next) {
    if (p->ai_family == AF_INET) {
      struct sockaddr_in *ipv4 =
          reinterpret_cast<struct sockaddr_in *>(p->ai_addr);
      memcpy(addr, &ipv4->sin_addr, sizeof(in_addr_t));
      freeaddrinfo(res);
      return true;
    }
  }

  freeaddrinfo(res);
  return false;
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
