#include "nvs_base.h"

#include <string>

#include "esp_check.h"
#include "esp_err.h"
#include "nvs.h"
#include "nvs_default.h"

static const char *TAG = "NVSBase";

esp_err_t NVSBase::Get(char *value, size_t *size, const char *key) const {
  ESP_RETURN_ON_FALSE(nvs_handle_ != 0, ESP_ERR_INVALID_STATE, TAG,
                      "NVS handle is not initialized");
  ESP_RETURN_ON_FALSE(strcmp(key, "") != 0, ESP_ERR_INVALID_ARG, TAG,
                      "key is empty");
  return nvs_get_str(nvs_handle_, key, value, size);
}

esp_err_t NVSBase::EraseKey(const char *key) const {
  esp_err_t ret = ESP_OK;
  ESP_RETURN_ON_FALSE(nvs_handle_ != 0, ESP_ERR_INVALID_STATE, TAG,
                      "NVS handle is not initialized");
  ESP_RETURN_ON_ERROR(nvs_erase_key(nvs_handle_, key), TAG,
                      "Failed to erase key %s: %s", key, esp_err_to_name(ret));
  ESP_RETURN_ON_ERROR(nvs_commit(nvs_handle_), TAG,
                      "Failed to commit after erase key %s: %s", key,
                      esp_err_to_name(ret));
  return ESP_OK;
}

esp_err_t NVSBase::EraseKey(const std::string &key) const {
  return EraseKey(key.c_str());
}

esp_err_t NVSBase::EraseKey(NVSKey key) const {
  return EraseKey(nvs_key_to_string(key));
}

esp_err_t NVSBase::EraseAll() const {
  esp_err_t ret = ESP_OK;
  ESP_RETURN_ON_FALSE(nvs_handle_ != 0, ESP_ERR_INVALID_STATE, TAG,
                      "NVS handle is not initialized");
  ESP_RETURN_ON_ERROR(nvs_erase_all(nvs_handle_), TAG,
                      "Failed to erase all keys: %s", esp_err_to_name(ret));
  ESP_RETURN_ON_ERROR(nvs_commit(nvs_handle_), TAG,
                      "Failed to commit after erase all keys: %s",
                      esp_err_to_name(ret));
  return ret;
}

void NVSBase::setDefaultValue(uint8_t *value, const uint8_t *default_value,
                              size_t *size) const {
  memcpy(value, default_value, *size);
}

void NVSBase::setDefaultValue(char *value, const char *default_value,
                              size_t *size) const {
  strncpy(value, default_value, *size);
  *size = strlen(default_value) + 1;
}
