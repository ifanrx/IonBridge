#include "nvs_partition.h"

#include <memory>
#include <string>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_base.h"
#include "nvs_flash.h"
#include "nvs_namespace.h"

static const char *TAG = "NVSPartition";

#ifdef CONFIG_SECURE_FLASH_ENC_ENABLED
#define NVS_KEY_PARTITION_NAME "nvs_key"
#endif

#define PROTECTED_DATA_NVS_PARTITION "protected_data"
#define USER_DATA_NVS_PARTITION "nvs"

bool NVSPartition::initialized_ = false;

esp_err_t NVSPartition::Init() {
  esp_err_t ret = ESP_OK;
  if (NVSPartition::initialized_) {
    return ret;
  }

  ESP_LOGI(TAG, "Initializing NVS");
#ifdef CONFIG_SECURE_FLASH_ENC_ENABLED
  const esp_partition_t *partition = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS_KEYS,
      NVS_KEY_PARTITION_NAME);
  nvs_sec_cfg_t cfg = {};

  ESP_RETURN_ON_FALSE(partition != nullptr, ESP_ERR_NOT_FOUND, TAG,
                      "Failed to find NVS key partition");
  ESP_RETURN_ON_ERROR(nvs_flash_read_security_cfg(partition, &cfg), TAG,
                      "Failed to read NVS key partition security config");
  ESP_RETURN_ON_ERROR(
      nvs_flash_secure_init_partition(PROTECTED_DATA_NVS_PARTITION, &cfg), TAG,
      "Failed to initialize protected data NVS partition");

  ret = nvs_flash_secure_init_partition(USER_DATA_NVS_PARTITION, &cfg);
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "NVS partition \"%s\" is full or has a newer version. ",
             USER_DATA_NVS_PARTITION);
    ESP_RETURN_ON_ERROR(nvs_flash_erase_partition(USER_DATA_NVS_PARTITION), TAG,
                        "Failed to erase NVS partition: %s",
                        USER_DATA_NVS_PARTITION);
    ESP_RETURN_ON_ERROR(
        nvs_flash_secure_init_partition(USER_DATA_NVS_PARTITION, &cfg), TAG,
        "Failed to initialize NVS partition: %s", USER_DATA_NVS_PARTITION);
  }
#else
  ESP_RETURN_ON_ERROR(nvs_flash_init_partition(PROTECTED_DATA_NVS_PARTITION),
                      TAG, "Failed to initialize protected data NVS partition");
  ret = nvs_flash_init_partition(USER_DATA_NVS_PARTITION);
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "NVS partition \"%s\" is full or has a newer version. ",
             USER_DATA_NVS_PARTITION);
    ESP_RETURN_ON_ERROR(nvs_flash_erase_partition(USER_DATA_NVS_PARTITION), TAG,
                        "Failed to erase NVS partition: %s",
                        USER_DATA_NVS_PARTITION);
    ESP_RETURN_ON_ERROR(nvs_flash_init_partition(USER_DATA_NVS_PARTITION), TAG,
                        "Failed to initialize NVS partition: %s",
                        USER_DATA_NVS_PARTITION);
  }
#endif

  ESP_LOGI(TAG, "NVS initialized");
  NVSPartition::initialized_ = true;
  return ret;
}

esp_err_t NVSPartition::InitUserData() {
  esp_err_t ret = ESP_OK;
#ifdef CONFIG_SECURE_FLASH_ENC_ENABLED
  const esp_partition_t *partition = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS_KEYS,
      NVS_KEY_PARTITION_NAME);
  nvs_sec_cfg_t cfg = {};

  ESP_RETURN_ON_FALSE(partition != nullptr, ESP_ERR_NOT_FOUND, TAG,
                      "Failed to find NVS key partition");
  ESP_RETURN_ON_ERROR(nvs_flash_read_security_cfg(partition, &cfg), TAG,
                      "Failed to read NVS key partition security config");
  ret = nvs_flash_secure_init_partition(USER_DATA_NVS_PARTITION, &cfg);
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "NVS partition \"%s\" is full or has a newer version. ",
             USER_DATA_NVS_PARTITION);
    ESP_RETURN_ON_ERROR(nvs_flash_erase_partition(USER_DATA_NVS_PARTITION), TAG,
                        "Failed to erase NVS partition: %s",
                        USER_DATA_NVS_PARTITION);
    ESP_RETURN_ON_ERROR(
        nvs_flash_secure_init_partition(USER_DATA_NVS_PARTITION, &cfg), TAG,
        "Failed to initialize NVS partition: %s", USER_DATA_NVS_PARTITION);
  }
#else
  ret = nvs_flash_init_partition(USER_DATA_NVS_PARTITION);
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "NVS partition \"%s\" is full or has a newer version. ",
             USER_DATA_NVS_PARTITION);
    ESP_RETURN_ON_ERROR(nvs_flash_erase_partition(USER_DATA_NVS_PARTITION), TAG,
                        "Failed to erase NVS partition: %s",
                        USER_DATA_NVS_PARTITION);
    ESP_RETURN_ON_ERROR(nvs_flash_init_partition(USER_DATA_NVS_PARTITION), TAG,
                        "Failed to initialize NVS partition: %s",
                        USER_DATA_NVS_PARTITION);
  }
#endif

  return ret;
}

NVSPartition::NVSPartition(const std::string &name) {
  if (name != PROTECTED_DATA_NVS_PARTITION && name != USER_DATA_NVS_PARTITION) {
    ESP_LOGE(TAG, "Invalid NVS partition name: %s", name.c_str());
    error_code_ = ESP_ERR_NOT_FOUND;
    return;
  }
  name_ = name;
}

std::unique_ptr<NVSNamespace> NVSPartition::OpenNamespace(
    const std::string &namespace_name, nvs_open_mode open_mode) {
  if (error_code_ != ESP_OK) {
    return nullptr;
  }
  error_code_ = Init();
  if (error_code_ != ESP_OK) {
    return nullptr;
  }

  nvs_handle_t handle;
  error_code_ = nvs_open_from_partition(name_.c_str(), namespace_name.c_str(),
                                        open_mode, &handle);
  if (error_code_ != ESP_OK) {
    return nullptr;
  }
  return std::make_unique<NVSNamespace>(name_, namespace_name,
                                        std::make_unique<NVSBase>(handle));
}

esp_err_t NVSPartition::EraseAll() {
  if (error_code_ != ESP_OK) {
    return error_code_;
  }
  error_code_ = Init();
  if (error_code_ != ESP_OK) {
    return error_code_;
  }

  error_code_ = nvs_flash_erase_partition(name_.c_str());
  if (error_code_ != ESP_OK) {
    return error_code_;
  }
  return ESP_OK;
}
