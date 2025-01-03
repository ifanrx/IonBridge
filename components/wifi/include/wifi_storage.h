#ifndef H_WIFI_STORAGE_
#define H_WIFI_STORAGE_

#include <stdint.h>

#include <array>
#include <string>

#include "esp_err.h"

#define WIFI_SLOT_COUNT 64

typedef struct {
  char ssid[32];
  char passwd[64];
  uint8_t auth_mode;
} WiFiEntry;

class WiFiStorage {
 public:
  WiFiStorage();
  ~WiFiStorage() = default;
  esp_err_t Create(const WiFiEntry &entry);
  esp_err_t Add(const WiFiEntry &entry);
  esp_err_t Remove(const std::string &ssid);
  esp_err_t RemoveAll();
  esp_err_t Get(uint8_t index, WiFiEntry *entry) const;
  bool HasSSID(const char *ssid) const;
  esp_err_t GetBySSID(const char *ssid, WiFiEntry *entry) const;
  const std::array<uint32_t, WIFI_SLOT_COUNT> &GetHash() const {
    return hash_;
  };
  void Reset() { hash_.fill(0); };

 private:
  std::array<uint32_t, WIFI_SLOT_COUNT> hash_ = {};
  size_t HashBytes() const { return hash_.size() * sizeof(uint32_t); }
  int FindEmptySlot() const;
  int FindSlot(const std::string &ssid) const;
};

typedef struct {
  uint8_t index;
  int8_t rssi;
} WiFiRSSI;

#endif  // H_WIFI_STORAGE_
