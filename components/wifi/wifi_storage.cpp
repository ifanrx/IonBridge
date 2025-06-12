#include "wifi_storage.h"

#include <string.h>
#include <sys/types.h>

#include <array>
#include <cstdint>
#include <cstring>

#include "esp_check.h"
#include "esp_crc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "ionbridge.h"
#include "nvs.h"
#include "nvs_default.h"
#include "nvs_namespace.h"
#include "sdkconfig.h"

#define WIFI_MAX_ENTRY CONFIG_WIFI_MAX_ENTRY
#define WIFI_DEFAULT_SSID CONFIG_IONBRIDGE_WIFI_DEFAULT_SSID
#define WIFI_DEFAULT_PASSWD CONFIG_IONBRIDGE_WIFI_DEFAULT_PASSWORD

static const char* TAG = "WiFiStorage";

WiFiStorage::WiFiStorage() {
  hash_.fill(0);
  size_t size = hash_.size() * sizeof(uint32_t);
  ESP_ERROR_COMPLAIN(WifiNVSGet(hash_.data(), &size, NVSKey::WIFI_HASH),
                     "Failed to obtain Wi-Fi hash");
}

esp_err_t WiFiStorage::Create(const WiFiEntry& entry) {
  int slot_index = FindEmptySlot();
  if (slot_index == -1) {
    ESP_LOGE(TAG, "No empty slot available for SSID: %s", entry.ssid);
    return ESP_ERR_NOT_FOUND;
  }

  NVSKey key = static_cast<NVSKey>(static_cast<int>(NVSKey::WIFI_CREDENTIAL_1) +
                                   slot_index);
  ESP_RETURN_ON_ERROR(WifiNVSSet(reinterpret_cast<const uint8_t*>(&entry), key,
                                 sizeof(WiFiEntry)),
                      TAG, "Failed to set NVS entry");

  hash_[slot_index] =
      esp_crc32_le(0, reinterpret_cast<const uint8_t*>(entry.ssid),
                   strnlen(entry.ssid, sizeof(entry.ssid)));
  ESP_RETURN_ON_ERROR(WifiNVSSet(reinterpret_cast<const uint8_t*>(hash_.data()),
                                 NVSKey::WIFI_HASH, HashBytes()),
                      TAG, "Failed to save Wi-Fi hash");

  return ESP_OK;
}

esp_err_t WiFiStorage::Add(const WiFiEntry& entry) {
  int slot_index = FindSlot(entry.ssid);
  if (slot_index == -1) {
    return Create(entry);
  }

  WiFiEntry existing_entry;
  esp_err_t err = Get(slot_index, &existing_entry);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGE(TAG, "Failed to get Wi-Fi entry at index %d", slot_index);
    return err;
  } else if (err == ESP_ERR_NVS_NOT_FOUND) {
    // hash_ is out of sync with the actual data, reset it
    hash_[slot_index] = 0;
    return Create(entry);
  }

  if (strcmp(existing_entry.ssid, entry.ssid) != 0) {
    return Create(entry);
  }

  if (strcmp(existing_entry.passwd, entry.passwd) == 0) {
    return ESP_OK;
  }

  NVSKey key = static_cast<NVSKey>(static_cast<int>(NVSKey::WIFI_CREDENTIAL_1) +
                                   slot_index);
  ESP_RETURN_ON_ERROR(WifiNVSSet(reinterpret_cast<const uint8_t*>(&entry), key,
                                 sizeof(WiFiEntry)),
                      TAG, "Failed to update Wi-Fi entry at index %d",
                      slot_index);

  return ESP_OK;
}

esp_err_t WiFiStorage::Remove(const std::string& ssid) {
  int slot_index = FindSlot(ssid);
  if (slot_index == -1) {
    ESP_LOGW(TAG, "SSID not found in storage: %s", ssid.c_str());
    return ESP_ERR_NOT_FOUND;
  }

  NVSKey key = static_cast<NVSKey>(static_cast<int>(NVSKey::WIFI_CREDENTIAL_1) +
                                   slot_index);
  ESP_RETURN_ON_ERROR(WifiNVSEraseKey(key), TAG,
                      "Failed to delete Wi-Fi entry at index %d", slot_index);

  hash_[slot_index] = 0;
  ESP_RETURN_ON_ERROR(WifiNVSSet(reinterpret_cast<const uint8_t*>(hash_.data()),
                                 NVSKey::WIFI_HASH, HashBytes()),
                      TAG, "Failed to save Wi-Fi hash");
  return ESP_OK;
}

esp_err_t WiFiStorage::RemoveAll() {
  Reset();
  return WifiNVSEraseAll();
}

esp_err_t WiFiStorage::Get(uint8_t index, WiFiEntry* entry) const {
  if (index >= WIFI_SLOT_COUNT) {
    return ESP_ERR_INVALID_ARG;
  }
  NVSKey key =
      static_cast<NVSKey>(static_cast<int>(NVSKey::WIFI_CREDENTIAL_1) + index);
  size_t size = sizeof(WiFiEntry);
  esp_err_t err = WifiNVSGet(reinterpret_cast<uint8_t*>(entry), &size, key);
  ESP_RETURN_ON_ERROR(err, TAG, "Failed to get Wi-Fi entry at index %d", index);
  entry->ssid[sizeof(entry->ssid) - 1] = '\0';      // Ensure null-termination
  entry->passwd[sizeof(entry->passwd) - 1] = '\0';  // Ensure null-termination
  return err;
}

bool WiFiStorage::HasSSID(const char* ssid) const {
  int slot_index = FindSlot(ssid);
  WiFiEntry entry;
  return slot_index != -1 && Get(slot_index, &entry) == ESP_OK &&
         strcmp(entry.ssid, ssid) == 0;
}

esp_err_t WiFiStorage::GetBySSID(const char* ssid, WiFiEntry* entry) const {
  int slot_index = FindSlot(ssid);
  if (slot_index == -1) {
    return ESP_ERR_NOT_FOUND;
  }
  ESP_RETURN_ON_ERROR(Get(slot_index, entry), TAG,
                      "Failed to get Wi-Fi entry at index %d", slot_index);
  return strcmp(entry->ssid, ssid) == 0 ? ESP_OK : ESP_ERR_NOT_FOUND;
}

int WiFiStorage::FindEmptySlot() const {
  int used_slot = 0;
  for (size_t i = 0; i < WIFI_SLOT_COUNT; ++i) {
    if (hash_[i] != 0) {
      used_slot++;
    }
  }
  if (used_slot >= WIFI_MAX_ENTRY) {
    return -1;
  }
  for (size_t i = 0; i < WIFI_SLOT_COUNT; ++i) {
    if (hash_[i] == 0) {
      return i;
    }
  }
  return -1;
}

int WiFiStorage::FindSlot(const std::string& ssid) const {
  uint32_t hash = esp_crc32_le(
      0, reinterpret_cast<const uint8_t*>(ssid.c_str()), ssid.size());
  for (size_t i = 0; i < WIFI_SLOT_COUNT; ++i) {
    if (hash_[i] == hash) {
      return i;
    }
  }
  return -1;
}
