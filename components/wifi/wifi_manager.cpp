#include "wifi_manager.h"

#include <string.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>  // for std::memcpy
#include <map>
#include <set>
#include <vector>

#include "esp_check.h"
#include "esp_crc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_wifi_types_generic.h"
#include "ionbridge.h"
#include "machine_info.h"
#include "nvs.h"
#include "utils.h"
#include "sdkconfig.h"
#include "wifi.h"

#define WIFI_DEFAULT_SSID CONFIG_IONBRIDGE_WIFI_DEFAULT_SSID
#define WIFI_DEFAULT_PASSWD CONFIG_IONBRIDGE_WIFI_DEFAULT_PASSWORD

// Define constants for scan attempts and delays
#define MAX_SCAN_ATTEMPTS 5
#define SCAN_RETRY_DELAY_MS 500

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
  return ESP_OK;
}

esp_err_t WiFiManager::Remove(const std::string& ssid) {
  esp_err_t err = wifi_storage_.Remove(ssid);
  ESP_ERROR_COMPLAIN(err, "Failed to remove Wi-Fi entry");
  if (err != ESP_OK) {
    return err;
  }
  return ESP_OK;
}

esp_err_t WiFiManager::RemoveAll() {
  ESP_RETURN_ON_ERROR(wifi_storage_.RemoveAll(), TAG,
                      "Failed to remove all Wi-Fi entries");
  Reset();
  return ESP_OK;
}

bool WiFiManager::GetWiFi(char* ssid, char* passwd) {
  if (found_count_ > 0 && use_index_ >= found_count_) {
    use_index_ = 0;
    return false;
  }
  if (use_index_ == 0) {
    ESP_RETURN_FALSE_ON_ERROR(UpdateAvailableWiFi(aps_),
                              "Failed to update available Wi-Fi");

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
  use_index_ = 0;
  return GetDefaultWiFiCredentials(ssid, passwd);
}

esp_err_t WiFiManager::UpdateAvailableWiFi(
    std::array<ssid_info_t, WIFI_SCAN_LIST_SIZE>& aps) {
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
  for (; use_index_ < found_count_; ++use_index_) {
    const WiFiRSSI& wr = wifi_rssi_[use_index_];
    WiFiEntry entry;
    esp_err_t err = wifi_storage_.Get(wr.index, &entry);
    if (err == ESP_OK) {
      strlcpy(ssid, entry.ssid, WIFI_SSID_MAXIMUM);
      strlcpy(passwd, entry.passwd, WIFI_PASSWD_MAXIMUM);
      return true;
    }
    if (err == ESP_ERR_NVS_NOT_FOUND) {
      continue;
    }
    ESP_RETURN_FALSE_ON_ERROR(err, "Failed to get Wi-Fi entry at index %d",
                              wr.index);
  }
  return false;
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

esp_err_t WiFiManager::ScanWiFi() {
  esp_err_t ret = ESP_OK;

  aps_.fill(ssid_info_t{});

  // Initialize scan configuration with default parameters
  wifi_scan_config_t scan_config = {
      .show_hidden = true,
      .scan_type = WIFI_SCAN_TYPE_ACTIVE,
      .scan_time =
          {
              .active =
                  {
                      .min = 100,
                      .max = 300,
                  },
          },
  };

  wifi_scan_default_params_t default_params;
  ESP_RETURN_ON_ERROR(esp_wifi_get_scan_parameters(&default_params), TAG,
                      "esp_wifi_get_scan_parameters");

  scan_config.scan_time = default_params.scan_time;
  scan_config.home_chan_dwell_time = default_params.home_chan_dwell_time;

  // Attempt to start the scan with retries
  for (uint8_t attempt = 0; attempt < MAX_SCAN_ATTEMPTS; attempt++) {
    ret = esp_wifi_scan_start(&scan_config, false);
    if (ret == ESP_OK) {
      break;  // Scan started successfully
    } else if (ret == ESP_ERR_WIFI_STATE && attempt < (MAX_SCAN_ATTEMPTS - 1)) {
      ESP_LOGW(TAG,
               "Wi-Fi not in correct state for scanning, retrying (%d/%d)...",
               attempt + 1, MAX_SCAN_ATTEMPTS);
      vTaskDelay(pdMS_TO_TICKS(SCAN_RETRY_DELAY_MS));
      continue;
    } else {
      ESP_LOGE(TAG, "Failed to start Wi-Fi scan: %s", esp_err_to_name(ret));
      return ret;
    }
  }
  return ESP_OK;
}

esp_err_t WiFiManager::GetScanResult() {
  esp_err_t ret = ESP_OK;

  ssid_info_t* aps = aps_.data();
  std::map<std::string, int> country_count;
  std::string most_common_country = WIFI_COUNTRY_UNKNOWN;

  uint16_t ap_count = 0;
  uint16_t scan_result_size = WIFI_SCAN_LIST_SIZE;
  uint8_t valid_ap_count = 0, max_count = 0;
  wifi_ap_record_t ap_info[WIFI_SCAN_LIST_SIZE];
  std::set<std::string> unique_ssids;
  memset(ap_info, 0, sizeof(ap_info));

  ESP_GOTO_ON_ERROR(esp_wifi_scan_get_ap_num(&ap_count), CLEANUP, TAG,
                    "esp_wifi_scan_get_ap_num");

  // Ensure that scan_result_size does not exceed the actual number of APs
  scan_result_size = std::min(scan_result_size, ap_count);

  ESP_GOTO_ON_ERROR(esp_wifi_scan_get_ap_records(&scan_result_size, ap_info),
                    CLEANUP, TAG, "esp_wifi_scan_get_ap_records");

  ESP_LOGI(TAG, "AP scan completed: %u/%u/%u AP found", scan_result_size,
           WIFI_SCAN_LIST_SIZE, ap_count);

  // Sort APs by RSSI in descending order
  std::sort(ap_info, ap_info + scan_result_size,
            [](const wifi_ap_record_t& a, const wifi_ap_record_t& b) {
              return a.rssi > b.rssi;
            });

  // Process each AP and populate the aps array
  for (uint8_t i = 0;
       i < scan_result_size && valid_ap_count < WIFI_SCAN_LIST_SIZE; ++i) {
    const wifi_ap_record_t* ap = &ap_info[i];

    // Safely determine the SSID length
    size_t ssid_length = strnlen((const char*)ap->ssid, sizeof(ap->ssid));
    if (ssid_length == 0 || ssid_length >= WIFI_SSID_MAXIMUM) {
      continue;  // Skip invalid SSIDs
    }
    std::string current_country_code(ap->country.cc, 2);

    ESP_LOGI(TAG, "Found AP: SSID: %s, RSSI: %d, BSSID: %s, Channel: %d, Country: %s",
             ap->ssid, ap->rssi, FORMAT_MAC(ap->bssid), ap->primary,
             current_country_code.c_str());
    std::string ssid_str((const char*)ap->ssid, ssid_length);
    if (unique_ssids.find(ssid_str) != unique_ssids.end()) {
      continue;  // Skip duplicate SSIDs
    }

    unique_ssids.insert(ssid_str);

    // Populate the aps array with AP details
    strlcpy(aps[valid_ap_count].ssid, (const char*)ap->ssid, sizeof(aps[valid_ap_count].ssid));
    aps[valid_ap_count].size = ssid_length;
    aps[valid_ap_count].rssi = ap->rssi;
    aps[valid_ap_count].auth_mode = ap->authmode;
#ifdef WIFI_CONNECT_SET_BSSID
    memcpy(aps[valid_ap_count].bssid, ap->bssid, sizeof(ap->bssid));
#endif
    valid_ap_count++;

    // Country code is not set, skip this AP in country code count
    if (ap->country.cc[0] == '\0') {
      continue;
    }

    country_count[current_country_code]++;

    // Determine the most common country code dynamically
    if (country_count[current_country_code] > max_count) {
      max_count = country_count[current_country_code];
      most_common_country = current_country_code;
    }
  }

  MachineInfo::GetInstance().SetCountryCode(most_common_country);
  ESP_LOGI(TAG, "Current country is %s", MachineInfo::GetInstance().GetCountryCode().c_str());

CLEANUP:
  // Clear the scan results
  esp_wifi_clear_ap_list();

  return ret;
}

int WiFiManager::GetScanAps(ssid_info_t* aps) {
  std::copy(aps_.begin(), aps_.end(), aps);
  return aps_.size();
}

#ifdef WIFI_CONNECT_SET_BSSID
bool WiFiManager::GetBSSIDBySSID(const char* ssid, uint8_t bssid[6]) {
  if (ssid == nullptr || bssid == nullptr) {
    ESP_LOGE(TAG, "Invalid arguments: ssid or bssid is null");
    return false;
  }

  for (const auto& ap : aps_) {
    if (ap.size != 0 && strcmp(ap.ssid, ssid) == 0) {
      memcpy(bssid, ap.bssid, sizeof(ap.bssid));
      return true;
    }
  }

  return false;
}
#endif
