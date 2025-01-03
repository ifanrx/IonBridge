#include "wifi_manager.h"

#include <string.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

#include "esp_check.h"
#include "esp_crc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_wifi_types_generic.h"
#include "ionbridge.h"
#include "nvs_default.h"
#include "nvs_namespace.h"
#include "sdkconfig.h"
#include "wifi.h"

#define WIFI_DEFAULT_SSID CONFIG_IONBRIDGE_WIFI_DEFAULT_SSID
#define WIFI_DEFAULT_PASSWD CONFIG_IONBRIDGE_WIFI_DEFAULT_PASSWORD

static const char* TAG = "WiFiManager";

// Ensure WIFI_RECORD_COUNT matches the number of Wi-Fi credentials stored in
// NVS
static constexpr size_t WIFI_RECORD_COUNT = 2;
static_assert(WIFI_RECORD_COUNT == 2,
              "WIFI_RECORD_COUNT must be 2 to match ssid_keys and passwd_keys "
              "initialization.");

esp_err_t WiFiManager::Add(const std::string& ssid, const std::string& passwd,
                           const uint8_t auth_mode) {
  WiFiEntry entry;
  strlcpy(entry.ssid, ssid.c_str(), sizeof(entry.ssid));
  strlcpy(entry.passwd, passwd.c_str(), sizeof(entry.passwd));
  entry.auth_mode = auth_mode;
  esp_err_t err = wifi_storage_.Add(entry);
  ESP_ERROR_COMPLAIN(err, "Failed to add Wi-Fi entry");
  if (err != ESP_OK) {
    return err;
  }
  use_index_ = 0;
  return ESP_OK;
}

esp_err_t WiFiManager::Remove(const std::string& ssid) {
  esp_err_t err = wifi_storage_.Remove(ssid);
  ESP_ERROR_COMPLAIN(err, "Failed to remove Wi-Fi entry");
  if (err != ESP_OK) {
    return err;
  }
  use_index_ = 0;
  return ESP_OK;
}

bool WiFiManager::GetWiFi(char* ssid, char* passwd) {
  if (use_index_ >= found_count_) {
    use_index_ = 0;
    found_count_ = 0;
  }

  if (use_index_ == 0) {
    aps_.fill(ssid_info_t{});
    ESP_RETURN_FALSE_ON_ERROR(ScanAndUpdateAvailableWiFi(aps_),
                              "Failed to scan and update available Wi-Fi");

    if (found_count_ == 0) {
      return GetDefaultWiFiCredentials(ssid, passwd);
    }

    if (found_count_ > wifi_rssi_.size()) {
      ESP_LOGE(TAG, "found_count_ (%u) exceeds wifi_rssi_ size (%zu)",
               found_count_, wifi_rssi_.size());
      return false;
    }

    std::sort(
        wifi_rssi_.begin(), wifi_rssi_.begin() + found_count_,
        [](const WiFiRSSI& a, const WiFiRSSI& b) { return a.rssi > b.rssi; });
  }

  if (GetNextWiFiEntry(ssid, passwd)) {
    use_index_++;
    return true;
  }
  return GetDefaultWiFiCredentials(ssid, passwd);
}

esp_err_t WiFiManager::ScanAndUpdateAvailableWiFi(
    std::array<ssid_info_t, WIFI_SCAN_LIST_SIZE>& aps) {
  ESP_RETURN_FALSE_ON_ERROR(wifi_scan_ap(aps.data(), false),
                            "Failed to scan for APs");

  std::vector<uint32_t> ap_crcs(aps.size(), 0);
  for (size_t i = 0; i < aps.size(); ++i) {
    if (aps[i].size == 0) {
      continue;
    }
    ap_crcs[i] = esp_crc32_le(
        0, reinterpret_cast<uint8_t*>(aps[i].ssid),
        strnlen(reinterpret_cast<char*>(aps[i].ssid), sizeof(aps[i].ssid)));
  }

  const auto& hash_list = wifi_storage_.GetHash();
  found_count_ = 0;
  found_default_wifi_ = false;

  for (size_t i = 0; i < WIFI_SLOT_COUNT; ++i) {
    uint32_t saved_crc = hash_list[i];
    if (saved_crc == 0) {
      continue;
    }
    for (size_t j = 0; j < aps.size(); ++j) {
      if (aps[j].size == 0 || ap_crcs[j] != saved_crc) {
        continue;
      }
      WiFiEntry entry;
      if (wifi_storage_.Get(i, &entry) != ESP_OK) {
        continue;
      }
      bool same_authmode = (aps[j].auth_mode == WIFI_AUTH_OPEN &&
                            entry.auth_mode == WIFI_AUTH_OPEN) ||
                           (aps[j].auth_mode != WIFI_AUTH_OPEN &&
                            entry.auth_mode != WIFI_AUTH_OPEN);
      if (strcmp(reinterpret_cast<char*>(aps[j].ssid), entry.ssid) == 0 &&
          same_authmode) {
        if (found_count_ < WIFI_SLOT_COUNT) {
          wifi_rssi_[found_count_].index = i;
          wifi_rssi_[found_count_].rssi = aps[j].rssi;
          ++found_count_;
        }
        break;
      }
    }
  }

  for (const auto& ap : aps) {
    if (ap.size != 0 && strcmp(reinterpret_cast<const char*>(ap.ssid),
                               WIFI_DEFAULT_SSID) == 0) {
      found_default_wifi_ = true;
      break;
    }
  }

  return ESP_OK;
}

bool WiFiManager::GetDefaultWiFiCredentials(char* ssid, char* passwd) {
  if (found_default_wifi_) {
    strlcpy(ssid, WIFI_DEFAULT_SSID, WIFI_SSID_MAXIMUM);
    strlcpy(passwd, WIFI_DEFAULT_PASSWD, WIFI_PASSWD_MAXIMUM);
    return true;
  }
  return false;
}

bool WiFiManager::GetNextWiFiEntry(char* ssid, char* passwd) {
  if (use_index_ >= found_count_) {
    return false;
  }
  const WiFiRSSI& wr = wifi_rssi_[use_index_];
  WiFiEntry entry;
  ESP_RETURN_FALSE_ON_ERROR(wifi_storage_.Get(wr.index, &entry),
                            "Failed to get Wi-Fi entry at index %d", wr.index);
  strlcpy(ssid, entry.ssid, WIFI_SSID_MAXIMUM);
  strlcpy(passwd, entry.passwd, WIFI_PASSWD_MAXIMUM);
  return true;
}

esp_err_t WiFiManager::GetWiFiBySSID(const std::string& ssid,
                                     std::string* passwd,
                                     uint8_t* auth_mode) const {
  if (ssid.empty() || passwd == nullptr || auth_mode == nullptr) {
    ESP_LOGE(TAG,
             "Invalid arguments: ssid is empty or passwd/auth_mode is null");
    return ESP_ERR_INVALID_ARG;
  }

  WiFiEntry entry;
  ESP_RETURN_ON_ERROR(wifi_storage_.GetBySSID(ssid.c_str(), &entry), TAG,
                      "Wi-Fi entry not found for SSID: %s", ssid.c_str());

  *passwd = entry.passwd;
  *auth_mode = entry.auth_mode;
  return ESP_OK;
}

bool WiFiManager::HasSSID(const std::string& ssid) {
  return !ssid.empty() && wifi_storage_.HasSSID(ssid.c_str());
}

esp_err_t WiFiManager::GetWiFiByIndex(uint8_t index, std::string* ssid,
                                      std::string* passwd,
                                      uint8_t* auth_mode) const {
  if (ssid == nullptr || passwd == nullptr || auth_mode == nullptr) {
    ESP_LOGE(TAG, "Invalid arguments: ssid/passwd/auth_mode is null");
    return ESP_ERR_INVALID_ARG;
  }

  WiFiEntry entry;
  ESP_RETURN_ON_ERROR(wifi_storage_.Get(index, &entry), TAG,
                      "Wi-Fi entry not found for index: %d", index);

  *ssid = entry.ssid;
  *passwd = entry.passwd;
  *auth_mode = entry.auth_mode;
  return ESP_OK;
}

void WiFiManager::OnConnected() {
  association_count_++;

  wifi_config_t wifi_config;
  wifi_ap_record_t ap_info;

  ESP_RETURN_VOID_ON_ERROR(esp_wifi_get_config(WIFI_IF_STA, &wifi_config), TAG,
                           "Failed to get Wi-Fi config");
  if (strcmp((char*)wifi_config.sta.ssid, WIFI_DEFAULT_SSID) == 0 &&
      strcmp((char*)wifi_config.sta.password, WIFI_DEFAULT_PASSWD) == 0) {
    return;
  }
  ESP_RETURN_VOID_ON_ERROR(esp_wifi_sta_get_ap_info(&ap_info), TAG,
                           "Failed to get AP info");
  this->Add((char*)ap_info.ssid, (char*)wifi_config.sta.password,
            ap_info.authmode);
}

void WiFiManager::OnDisconnected() { disassociation_count_++; }

void WiFiManager::Reset() {
  use_index_ = 0;
  found_count_ = 0;
  found_default_wifi_ = false;
  wifi_rssi_ = {};
  association_count_ = 0;
  disassociation_count_ = 0;
  wifi_storage_.Reset();
}

bool WiFiManager::HasStoredWiFi() const {
  std::array<uint32_t, WIFI_SLOT_COUNT> hash = wifi_storage_.GetHash();
  return std::any_of(hash.begin(), hash.end(),
                     [](uint32_t hash) { return hash != 0; });
}

void WiFiManager::GetScanAps(ssid_info_t* aps) const {
  std::copy(aps_.begin(), aps_.end(), aps);
}
