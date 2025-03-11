#include "nvs_namespace.h"

#include <cstdint>
#include <memory>
#include <string>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "nvs.h"
#include "nvs_default.h"
#include "nvs_partition.h"

static const char *TAG = "NVSNamespace";

std::unique_ptr<NVSNamespace> NVSNamespace::Open(
    const std::string &partition_name, const std::string &namespace_name,
    nvs_open_mode open_mode) {
  NVSPartition partition(partition_name);

  // Open the namespace within the partition
  std::unique_ptr<NVSNamespace> ns =
      partition.OpenNamespace(namespace_name, open_mode);

  // If the namespace could not be opened, return nullptr
  if (!ns) {
    esp_err_t err = partition.GetErrorCode();
    ESP_LOGE(TAG, "Failed to open NVS namespace: %s/%s, error: %s(%d)",
             partition_name.c_str(), namespace_name.c_str(),
             esp_err_to_name(err), err);
    return nullptr;
  }

  // Return the opened namespace
  return ns;
}

esp_err_t NVSNamespace::SEraseKey(NVSKey key, const std::string &partition_name,
                                  const std::string &namespace_name) {
  std::unique_ptr<NVSNamespace> ns =
      NVSNamespace::Open(partition_name, namespace_name, NVS_READWRITE);
  ESP_RETURN_ON_FALSE(ns != nullptr, ESP_FAIL, TAG, "open %s namespace failed",
                      namespace_name.c_str());
  return ns->EraseKey(key);
}

esp_err_t NVSNamespace::SEraseAll(const std::string &partition_name,
                                  const std::string &namespace_name) {
  std::unique_ptr<NVSNamespace> ns =
      NVSNamespace::Open(partition_name, namespace_name, NVS_READWRITE);
  ESP_RETURN_ON_FALSE(ns != nullptr, ESP_FAIL, TAG, "open %s namespace failed",
                      namespace_name.c_str());
  return ns->EraseAll();
}

esp_err_t NVSGetAuthToken(uint8_t *token, bool always_generate_new) {
  static uint8_t cached_token = 0, cached = 0;
  esp_err_t err = ESP_OK;
  if (!always_generate_new) {
    if (cached) {
      *token = cached_token;
      return ESP_OK;
    }

    err = BleNVSGet(token, NVSKey::DEVICE_TOKEN);
    if (err == ESP_OK) {
      cached = 1;
      cached_token = *token;
      return ESP_OK;
    }
    ESP_LOGI(TAG, "Token not found in NVS, generating a new one, err: %d", err);
  }

  uint8_t new_token = esp_random() % 256;
  err = BleNVSSet(new_token, NVSKey::DEVICE_TOKEN);
  if (err != ESP_OK) {
    return err;
  }

  *token = new_token;
  cached = 1;
  cached_token = *token;

  return ESP_OK;
}

esp_err_t ResetUserData() {
  NVSPartition partition(USER_DATA_NVS_PARTITION);
  ESP_RETURN_ON_ERROR(partition.EraseAll(), TAG, "erase_user_data");
  ESP_RETURN_ON_ERROR(NVSPartition::InitUserData(), TAG, "init_user_data");
  return ESP_OK;
}

#ifdef CONFIG_ENABLE_RFTEST
esp_err_t TestModeNVSData::GetAndEraseTestModeAArgs(uint32_t args[6]) {
  uint8_t *data_ptr = reinterpret_cast<uint8_t *>(args);
  size_t data_len = sizeof(uint32_t) * 6;
  ESP_RETURN_ON_ERROR(TestModeNVSGet(&data_ptr, NVSKey::TEST_MODE_A, &data_len),
                      TAG, "GetTestModeAArgs");
  memcpy(args, data, data_len);
  return TestModeNVSEraseKey(NVSKey::TEST_MODE_A);
}

esp_err_t TestModeNVSData::SaveTestModeAArgs(uint32_t args[6]) {
  size_t data_len = sizeof(uint32_t) * 6;
  return TestModeNVSSet(reinterpret_cast<uint8_t *>(args), NVSKey::TEST_MODE_A,
                        data_len);
}

#endif
