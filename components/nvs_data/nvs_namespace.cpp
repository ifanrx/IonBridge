#include "nvs_namespace.h"

#include <cstdint>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "nvs_key.h"
#include "nvs_partition.h"

static const char* TAG = "NVSNamespace";

#ifdef IONBRIDGE_NVS_DATA_HOST_TEST
bool g_reset_ble_token_cache = false;
#endif

using namespace nvs_namespace;
using namespace nvs_wrapper;

// Low-level NVS operations (formerly in Base class)
esp_err_t Namespace::OpenHandle() {
  if (handle_ != 0) {
    ESP_LOGW(TAG, "NVS handle is already open, closing it first");
    nvs_close(handle_);
    handle_ = 0;
  }

  return nvs_open_from_partition(partition_name_.c_str(),
                                 namespace_name_.c_str(), open_mode_, &handle_);
}

esp_err_t Namespace::OpenHandle(nvs_open_mode_t mode) {
  open_mode_ = mode;
  return OpenHandle();
}

bool Namespace::IsWritable() const {
  if (!IsValid()) {
    return false;
  }
  return true;
}

esp_err_t Namespace::GetSize(const Key& key, size_t& size) const {
  ESP_RETURN_ON_FALSE(IsValid(), ESP_ERR_INVALID_STATE, TAG,
                      "NVS handle is invalid");
  const char* key_str = key.c_str();
  esp_err_t err = nvs_get_blob(handle_, key_str, nullptr, &size);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    size = 0;  // Key not found, return size as 0
  } else if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get size for key '%s': %s", key_str,
             esp_err_to_name(err));
  }
  return err;
}

esp_err_t Namespace::GetBlob(const Key& key, void* buffer, size_t& size) const {
  ESP_RETURN_ON_FALSE(IsValid(), ESP_ERR_INVALID_STATE, TAG,
                      "NVS handle is invalid");
  return nvs_get_blob(handle_, key.c_str(), buffer, &size);
}

esp_err_t Namespace::SetBlob(const Key& key, const void* buffer,
                             size_t size) const {
  ESP_RETURN_ON_FALSE(IsWritable(), ESP_ERR_INVALID_STATE, TAG,
                      "NVS handle is not writable or invalid");
  const char* key_str = key.c_str();
  ESP_RETURN_ON_ERROR(nvs_set_blob(handle_, key_str, buffer, size), TAG,
                      "Failed to set blob for key '%s'", key_str);
  return Commit();
}

esp_err_t Namespace::GetString(const Key& key, std::string& value) const {
  ESP_RETURN_ON_FALSE(IsValid(), ESP_ERR_INVALID_STATE, TAG,
                      "NVS handle is invalid");
  size_t size = 0;
  const char* key_str = key.c_str();
  esp_err_t err = nvs_get_str(handle_, key_str, nullptr, &size);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGE(TAG, "Failed to get string size for key '%s': %s", key_str,
             esp_err_to_name(err));
    return err;
  }

  if (err == ESP_ERR_NVS_NOT_FOUND) {
    value.clear();
    return ESP_ERR_NVS_NOT_FOUND;
  }
  if (size == 0) {
    value.clear();
    return ESP_OK;  // Key exists but has no value
  }

  value.resize(size);
  err = nvs_get_str(handle_, key_str, &value[0], &size);
  if (err == ESP_OK && size > 0) {
    value.resize(size - 1);  // Remove null terminator
  }
  return err;
}

esp_err_t Namespace::GetString(const Key& key, char* buffer,
                               size_t& size) const {
  ESP_RETURN_ON_FALSE(IsValid(), ESP_ERR_INVALID_STATE, TAG,
                      "NVS handle is invalid");
  return nvs_get_str(handle_, key.c_str(), buffer, &size);
}

esp_err_t Namespace::SetString(const Key& key, const std::string& value) const {
  ESP_RETURN_ON_FALSE(IsWritable(), ESP_ERR_INVALID_STATE, TAG,
                      "NVS handle is not writable or invalid");
  const char* key_str = key.c_str();
  ESP_RETURN_ON_ERROR(nvs_set_str(handle_, key_str, value.c_str()), TAG,
                      "Failed to set string for key '%s'", key_str);
  return Commit();
}

esp_err_t Namespace::SetString(const Key& key, const char* value) const {
  ESP_RETURN_ON_FALSE(IsWritable(), ESP_ERR_INVALID_STATE, TAG,
                      "NVS handle is not writable or invalid");
  const char* key_str = key.c_str();
  ESP_RETURN_ON_ERROR(nvs_set_str(handle_, key_str, value), TAG,
                      "Failed to set string for key '%s'", key_str);
  return Commit();
}

esp_err_t Namespace::Commit() const {
  ESP_RETURN_ON_FALSE(IsWritable(), ESP_ERR_INVALID_STATE, TAG,
                      "NVS handle is not writable or invalid");
  return nvs_commit(handle_);
}

esp_err_t Namespace::EraseKey(const Key& key) const {
  ESP_RETURN_ON_FALSE(IsWritable(), ESP_ERR_INVALID_STATE, TAG,
                      "NVS handle is not writable or invalid");
  const char* key_str = key.c_str();
  esp_err_t erase_err = nvs_erase_key(handle_, key_str);

  // Don't log error if key doesn't exist - that's a valid scenario
  if (erase_err != ESP_OK && erase_err != ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGE(TAG, "Failed to erase key '%s': %s", key_str,
             esp_err_to_name(erase_err));
    return erase_err;
  }

  // Always commit, even if key wasn't found, to flush any pending operations
  esp_err_t commit_err = Commit();

  // Return success if erase succeeded or key wasn't found
  // (Public API treats "not found" as success)
  if (erase_err == ESP_ERR_NVS_NOT_FOUND) {
    return ESP_OK;
  }
  return commit_err;
}

esp_err_t Namespace::EraseAll() const {
  ESP_RETURN_ON_FALSE(IsWritable(), ESP_ERR_INVALID_STATE, TAG,
                      "NVS handle is not writable or invalid");
  ESP_RETURN_ON_ERROR(nvs_erase_all(handle_), TAG,
                      "Failed to erase all keys in namespace '%s'",
                      namespace_name_.c_str());
  return Commit();
}

esp_err_t Namespace::Reset() {
  ESP_RETURN_ON_ERROR(this->EraseAll(), TAG,
                      "Failed to erase NVS namespace: %s",
                      namespace_name_.c_str());
  if (handle_ != 0) {
    nvs_close(handle_);
    handle_ = 0;
  }
  return OpenHandle();
}

esp_err_t nvs_namespace::Initialize() {
  ESP_RETURN_ON_ERROR(protected_data::Initialize(), TAG,
                      "Failed to initialize protected data namespaces");
  ESP_RETURN_ON_ERROR(user_data::Initialize(), TAG,
                      "Failed to initialize user data namespaces");
  return ESP_OK;
}

namespace nvs_namespace {
namespace protected_data {
// Define namespace objects
Namespace DEVICE_DATA(partition::PROTECTED_DATA_NVS_PARTITION, "device_data");
Namespace CERTS(partition::PROTECTED_DATA_NVS_PARTITION, "certs");
Namespace MQTT(partition::PROTECTED_DATA_NVS_PARTITION, "mqtt");

Namespace* NAMESPACES[] = {
    &DEVICE_DATA,
    &CERTS,
    &MQTT,
};

esp_err_t Initialize() {
  for (Namespace* ns : NAMESPACES) {
    ESP_RETURN_ON_ERROR(ns->Open(), TAG, "Failed to open NVS namespace: %s",
                        ns->GetName().c_str());
  }
  return ESP_OK;
}
}  // namespace protected_data

namespace user_data {
// Define namespace objects
Namespace MQTT_CUSTOM(partition::USER_DATA_NVS_PARTITION, "mqtt_custom");
Namespace BLE(partition::USER_DATA_NVS_PARTITION, "BLE-SERVICE");
Namespace DISPLAY_DATA(partition::USER_DATA_NVS_PARTITION, "display_data");
Namespace POWER(partition::USER_DATA_NVS_PARTITION, "power");
Namespace LICENSE(partition::USER_DATA_NVS_PARTITION, "license");
Namespace SYSLOG(partition::USER_DATA_NVS_PARTITION, "syslog");
Namespace FPGA(partition::USER_DATA_NVS_PARTITION, "fpga");
Namespace WIFI(partition::USER_DATA_NVS_PARTITION, "Wi-Fi");
Namespace OTA(partition::USER_DATA_NVS_PARTITION, "ota");
Namespace SYSTEM(partition::USER_DATA_NVS_PARTITION, "system");

#ifdef CONFIG_ENABLE_RFTEST
Namespace TEST_MODE(partition::USER_DATA_NVS_PARTITION, "test_mode");
#endif

Namespace* NAMESPACES[] = {
    &MQTT_CUSTOM, &BLE,  &DISPLAY_DATA, &POWER, &LICENSE,
    &SYSLOG,      &FPGA, &WIFI,         &OTA,   &SYSTEM,
#ifdef CONFIG_ENABLE_RFTEST
    &TEST_MODE,
#endif
};

esp_err_t Initialize() {
  for (Namespace* ns : NAMESPACES) {
    ESP_RETURN_ON_ERROR(ns->Open(), TAG, "Failed to open NVS namespace: %s",
                        ns->GetName().c_str());
  }
  return ESP_OK;
}
}  // namespace user_data
}  // namespace nvs_namespace

esp_err_t nvs_utils::GetBLEAuthToken(uint8_t* token, bool always_generate_new) {
  if (token == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  static uint8_t cached_token = 0, cached = 0;

#ifdef IONBRIDGE_NVS_DATA_HOST_TEST
  // Check if cache reset was requested
  if (g_reset_ble_token_cache) {
    cached = 0;
    cached_token = 0;
    g_reset_ble_token_cache = false;
  }
#endif

  esp_err_t err = ESP_OK;
  if (!always_generate_new) {
    if (cached) {
      *token = cached_token;
      return ESP_OK;
    }

    err = user_data::BLE.Get(nvs_keys::DEVICE_TOKEN, *token);
    if (err == ESP_OK) {
      cached = 1;
      cached_token = *token;
      return ESP_OK;
    }
    ESP_LOGI(TAG, "Token not found in NVS, generating a new one, err: %d", err);
  }

  uint8_t new_token = esp_random() % 256;
  err = user_data::BLE.Set(nvs_keys::DEVICE_TOKEN, new_token);
  if (err != ESP_OK) {
    return err;
  }

  *token = new_token;
  cached = 1;
  cached_token = *token;

  return ESP_OK;
}

#ifdef IONBRIDGE_NVS_DATA_HOST_TEST
void nvs_utils::ResetBLEAuthTokenCache() {
  // Set the global flag to reset the cache on next call
  g_reset_ble_token_cache = true;
}
#endif

#ifdef CONFIG_ENABLE_RFTEST
esp_err_t nvs_utils::GetAndEraseTestModeAArgs(uint32_t args[6]) {
  uint8_t* data_ptr = reinterpret_cast<uint8_t*>(args);
  ESP_RETURN_ON_ERROR(
      user_data::TEST_MODE.Get(nvs_keys::TEST_MODE_A, data_ptr, 6), TAG,
      "GetTestModeAArgs");
  return user_data::TEST_MODE.EraseKey(nvs_keys::TEST_MODE_A);
}

esp_err_t nvs_utils::SaveTestModeAArgs(uint32_t args[6]) {
  uint8_t* data_ptr = reinterpret_cast<uint8_t*>(args);
  return user_data::TEST_MODE.Set(nvs_keys::TEST_MODE_A, data_ptr, 6);
}
#endif
