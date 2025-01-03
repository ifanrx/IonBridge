#ifndef NVS_BASE_H_
#define NVS_BASE_H_

#include <string.h>

#include <cstddef>
#include <cstdint>
#include <string>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_default.h"

#define NVS_BASE_LOG_TAG "NVSBase"

class NVSBase {
 public:
  explicit NVSBase(nvs_handle_t handle) : nvs_handle_(handle) {}
  ~NVSBase() {
    if (nvs_handle_ != 0) {
      nvs_close(nvs_handle_);
    }
  }

  template <typename T>
  esp_err_t Get(T *value, size_t *size, const char *key) const {
    ESP_RETURN_ON_FALSE(nvs_handle_ != 0, ESP_ERR_INVALID_STATE,
                        NVS_BASE_LOG_TAG, "NVS handle is not initialized");
    ESP_RETURN_ON_FALSE(strcmp(key, "") != 0, ESP_ERR_INVALID_ARG,
                        NVS_BASE_LOG_TAG, "key is empty");
    return nvs_get_blob(nvs_handle_, key, value, size);
  }

  esp_err_t Get(char *value, size_t *size, const char *key) const;

  template <typename T>
  esp_err_t GetOrDefault(T *value, size_t *size, NVSKey key) const {
    // Retrieve the default value from the map
    ConfigValue default_value_variant = DEFAULT_CONFIG.at(key);

    // Check if the variant holds the correct type
    if (std::holds_alternative<T *>(default_value_variant)) {
      esp_err_t ret = Get(value, size, nvs_key_to_string(key).c_str());
      if (ret != ESP_OK) {
        setDefaultValue(value, std::get<T *>(default_value_variant), size);
        ret = ESP_OK;
      }
      return ret;
    } else {
      // Handle the case where the type does not match
      ESP_LOGE(NVS_BASE_LOG_TAG, "Type mismatch for key: %s",
               nvs_key_to_string(key).c_str());
      return ESP_ERR_INVALID_ARG;
    }
  }

  template <typename T>
  esp_err_t GetOrDefault(T *value, NVSKey key) const {
    // Retrieve the default value from the map
    ConfigValue default_value_variant = DEFAULT_CONFIG.at(key);

    // Check if the variant holds the correct type
    if (std::holds_alternative<T>(default_value_variant)) {
      size_t size = sizeof(T);
      esp_err_t ret = Get(value, &size, nvs_key_to_string(key).c_str());
      if (ret != ESP_OK) {
        setDefaultValue(value, std::get<T>(default_value_variant));
        ret = ESP_OK;
      }
      return ret;
    } else {
      // Handle the case where the type does not match
      ESP_LOGE(NVS_BASE_LOG_TAG, "Type mismatch for key: %s",
               nvs_key_to_string(key).c_str());
      return ESP_ERR_INVALID_ARG;
    }
  }

  // General template method for setting a value in NVS
  template <typename T>
  esp_err_t Set(const T &value, NVSKey key, size_t size) const {
    esp_err_t ret = ESP_OK;
    std::string s = nvs_key_to_string(key);
    ESP_RETURN_ON_FALSE(nvs_handle_ != 0, ESP_ERR_INVALID_STATE,
                        NVS_BASE_LOG_TAG, "NVS handle is not initialized");
    ESP_RETURN_ON_FALSE(s.length() != 0, ESP_ERR_INVALID_STATE,
                        NVS_BASE_LOG_TAG, "Key is empty");

    const char *key_str = s.c_str();
    ESP_RETURN_ON_ERROR(nvs_set_blob(nvs_handle_, key_str, &value, size),
                        NVS_BASE_LOG_TAG, "set value(%s)", key_str);
    ESP_RETURN_ON_ERROR(nvs_commit(nvs_handle_), NVS_BASE_LOG_TAG,
                        "nvs commit(%s)", key_str);
    return ret;
  }
  template <typename T>
  esp_err_t Set(const T *value, NVSKey key, size_t size) const {
    esp_err_t ret = ESP_OK;
    std::string s = nvs_key_to_string(key);
    ESP_RETURN_ON_FALSE(nvs_handle_ != 0, ESP_ERR_INVALID_STATE,
                        NVS_BASE_LOG_TAG, "NVS handle is not initialized");
    ESP_RETURN_ON_FALSE(s.length() != 0, ESP_ERR_INVALID_STATE,
                        NVS_BASE_LOG_TAG, "Key is empty");

    const char *key_str = s.c_str();
    if constexpr (std::is_same<T, char>::value) {
      ESP_RETURN_ON_ERROR(nvs_set_str(nvs_handle_, key_str, value),
                          NVS_BASE_LOG_TAG, "set str(%s)", key_str);
    } else {
      ESP_RETURN_ON_ERROR(nvs_set_blob(nvs_handle_, key_str, value, size),
                          NVS_BASE_LOG_TAG, "set blob(%s)", key_str);
    }
    ESP_RETURN_ON_ERROR(nvs_commit(nvs_handle_), NVS_BASE_LOG_TAG,
                        "nvs commit(%s)", key_str);
    return ret;
  }

  esp_err_t EraseKey(const std::string &key) const;
  esp_err_t EraseKey(const char *key) const;
  esp_err_t EraseKey(NVSKey key) const;
  esp_err_t EraseAll() const;

 private:
  nvs_handle_t nvs_handle_;

  template <typename T>
  void setDefaultValue(T *value, const T &default_value) const {
    *value = default_value;
  }
  void setDefaultValue(char *value, const char *default_value,
                       size_t *size) const;
  void setDefaultValue(uint8_t *value, const uint8_t *default_value,
                       size_t *size) const;
};
#endif
