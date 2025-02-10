#ifndef H_WIFI_MANAGER_
#define H_WIFI_MANAGER_

#include <stdint.h>

#include <array>
#include <cstdint>
#include <string>

#include "esp_err.h"
#include "wifi.h"
#include "wifi_storage.h"

#define WIFI_SLOT_COUNT 64
#define WIFI_SCAN_LIST_SIZE CONFIG_ESP_WIFI_SCAN_LIST_SIZE

#ifdef CONFIG_WIFI_CONNECT_SET_BSSID
#define WIFI_CONNECT_SET_BSSID
#endif

class WiFiManager {
 public:
  static WiFiManager& GetInstance() {
    static WiFiManager instance;
    return instance;
  }

  WiFiManager(const WiFiManager&) = delete;
  WiFiManager& operator=(const WiFiManager&) = delete;

  esp_err_t Add(const std::string& ssid, const std::string& passwd,
                const uint8_t auth_mode);
  esp_err_t Remove(const std::string& ssid);
  esp_err_t RemoveAll() { return wifi_storage_.RemoveAll(); }
  bool GetWiFi(char* ssid, char* passwd);
  bool AllWiFiTried() const { return use_index_ >= found_count_; }
  esp_err_t GetWiFiByIndex(uint8_t index, std::string* ssid,
                           std::string* passwd, uint8_t* auth_mode) const;
  bool HasSSID(const std::string& ssid);
  esp_err_t GetWiFiBySSID(const std::string& ssid, std::string* passwd,
                          uint8_t* auth_mode) const;
  const std::array<uint32_t, WIFI_SLOT_COUNT>& GetHash() const {
    return wifi_storage_.GetHash();
  }
  bool HasStoredWiFi() const;
  esp_err_t ScanWiFi();
  esp_err_t GetScanResult();
  int GetScanAps(ssid_info_t* aps);
#ifdef WIFI_CONNECT_SET_BSSID
  bool GetBSSIDBySSID(const char* ssid, uint8_t bssid[6]);
#endif

  uint16_t GetAssociationCount() const { return association_count_; }
  uint16_t GetDisassociationCount() const { return disassociation_count_; }
  uint16_t GetWifiConnectionTime() const { return wifi_connection_time_; }
  uint16_t GetConnectivityFailureCount() const {
    return connectivity_failure_count_;
  }
  uint16_t GetDNSResolutionFailureCount() const {
    return dns_resolution_failure_count_;
  }

  void AddConnectivityFailure() { connectivity_failure_count_++; }
  void AddDNSResolutionFailure() { dns_resolution_failure_count_++; }
  void AddWifiConnectionTime(uint16_t time) { wifi_connection_time_ += time; }

  void OnConnected();
  void OnDisconnected();

  void Reset();

 private:
  WiFiManager() {
    use_index_ = 0;
    found_count_ = 0;
    found_default_wifi_ = false;
  };
  ~WiFiManager() = default;

  // Helper functions
  esp_err_t UpdateAvailableWiFi(
      std::array<ssid_info_t, WIFI_SCAN_LIST_SIZE>& aps);
  bool GetDefaultWiFiCredentials(char* ssid, char* passwd);
  bool GetNextWiFiEntry(char* ssid, char* passwd);

  uint8_t use_index_ = 0;
  uint8_t found_count_ = 0;
  bool found_default_wifi_ = false;
  std::array<WiFiRSSI, WIFI_SLOT_COUNT> wifi_rssi_ = {};
  WiFiStorage wifi_storage_;
  std::array<ssid_info_t, WIFI_SCAN_LIST_SIZE> aps_ = {};

  uint16_t association_count_ = 0;
  uint16_t disassociation_count_ = 0;
  uint16_t wifi_connection_time_ = 0;
  uint16_t connectivity_failure_count_ = 0;
  uint16_t dns_resolution_failure_count_ = 0;
};

#endif  // H_WIFI_MANAGER_
