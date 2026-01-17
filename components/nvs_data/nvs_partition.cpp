#include "nvs_partition.h"

#include <string>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "nvs_namespace.h"

#ifdef CONFIG_SECURE_FLASH_ENC_ENABLED
#define NVS_KEY_PARTITION_NAME "nvs_key"
#endif

using namespace nvs_wrapper;

static const char *TAG = "NVSPartition";

static bool initialized = false;

esp_err_t nvs_wrapper::Initialize() {
  esp_err_t ret = ESP_OK;
  if (initialized) {
    return ret;
  }

  ESP_LOGI(TAG, "Initializing NVS");
  ESP_RETURN_ON_ERROR(partition::InitializeProtectedData(), TAG,
                      "Failed to initialize protected data NVS partition");
  ESP_RETURN_ON_ERROR(partition::InitializeUserData(), TAG,
                      "Failed to initialize user data NVS partition");
  ESP_RETURN_ON_ERROR(nvs_namespace::Initialize(), TAG,
                      "Failed to initialize NVS namespaces");
  ESP_LOGI(TAG, "NVS initialized");
  initialized = true;
  return ret;
}

esp_err_t partition::Initialize(const std::string &name,
                                bool erase_at_failure) {
  ESP_RETURN_ON_FALSE(ValidateName(name), ESP_ERR_INVALID_ARG, TAG,
                      "Invalid NVS partition name: %s", name.c_str());

  const char *partition_name = name.c_str();
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

  ret = nvs_flash_secure_init_partition(partition_name, &cfg);
#else
  ret = nvs_flash_init_partition(partition_name);
#endif

  if (erase_at_failure && (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
                           ret == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
    ESP_LOGW(TAG, "NVS partition \"%s\" is full or has a newer version. ",
             partition_name);
    ESP_RETURN_ON_ERROR(nvs_flash_erase_partition(partition_name), TAG,
                        "Failed to erase NVS partition: %s", partition_name);
    ESP_RETURN_ON_ERROR(
#ifdef CONFIG_SECURE_FLASH_ENC_ENABLED
        nvs_flash_secure_init_partition(partition_name, &cfg),
#else
        nvs_flash_init_partition(partition_name),
#endif
        TAG, "Failed to initialize NVS partition: %s", partition_name);
  }
  return ret;
}

esp_err_t partition::ReinitalizeUserData() {
  ESP_LOGW(TAG, "Reinitializing user data NVS partition");
  ESP_RETURN_ON_ERROR(EraseUserData(), TAG,
                      "Failed to erase user data NVS partition");
  ESP_RETURN_ON_ERROR(InitializeUserData(), TAG,
                      "Failed to reinitialize user data NVS partition");
  return nvs_namespace::user_data::Initialize();
}

esp_err_t partition::EraseUserData() {
  ESP_LOGW(TAG, "Erasing user data NVS partition");
  esp_err_t ret = nvs_flash_erase_partition(USER_DATA_NVS_PARTITION);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to erase user data NVS partition: %s",
             esp_err_to_name(ret));
  }
  return ret;
}
