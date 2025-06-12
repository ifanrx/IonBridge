#include "controller.h"

#include <sys/types.h>

#include <cstdint>

#include "display_manager.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_flash_partitions.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "ionbridge.h"
#include "machine_info.h"
#include "nvs_namespace.h"
#include "port.h"
#include "port_manager.h"
#include "power_allocator.h"
#include "sdkconfig.h"

#define OTA_CONFIRM_REQUEST_TIMEOUT CONFIG_OTA_CONFIRM_REQUEST_TIMEOUT
#define OTA_CONFIRM_REQUEST_INTERVAL CONFIG_OTA_CONFIRM_REQUEST_INTERVAL
#define OTA_CONFIRM_TIMEOUT CONFIG_OTA_CONFIRM_TIMEOUT
#define HASH_LENGTH 32

static const char *TAG = "Controller";

void DeviceController::rollback() {
  ESP_LOGI(TAG, "Rolling back to previous app");
  if (esp_ota_mark_app_invalid_rollback_and_reboot() != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_mark_app_invalid_rollback_and_reboot failed");
  }
}

bool DeviceController::is_ota_pending_verify() {
  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t ota_state;
  esp_err_t ret = esp_ota_get_state_partition(running, &ota_state);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_get_state_partition: %d, rolling back", ret);
    rollback();
    return false;
  }
  return ota_state == ESP_OTA_IMG_PENDING_VERIFY;
}

void DeviceController::try_rollback() {
  if (this->is_in_ota()) {
    this->rollback();
  } else {
    ESP_LOGW(TAG, "Not in OTA state, not rolling back");
  }
}

DeviceController::DeviceController() {
  if (is_ota_pending_verify()) {
    status_ = DeviceControllerStatus::REQUESTING_CONFIRMATION;
    return;
  }
  ESP_LOGD(TAG, "No OTA flag found");
  status_ = DeviceControllerStatus::POWER_UP;
}

static void save_confirm_result(bool success) {
  ConfirmResult confirm_result = {.success = success};
#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
  std::array<uint8_t, 3> bp_version =
      MachineInfo::GetInstance().GetMCUVersion();
  std::copy(bp_version.begin(), bp_version.end(),
            confirm_result.sw3566_version);
  std::array<uint8_t, 3> fpga_version =
      MachineInfo::GetInstance().GetFPGAVersion();
  std::copy(fpga_version.begin(), fpga_version.end(),
            confirm_result.fpga_version);
#endif
  std::array<uint8_t, 3> esp32_version =
      MachineInfo::GetInstance().GetESP32Version();
  std::copy(esp32_version.begin(), esp32_version.end(),
            confirm_result.esp32_version);
  ESP_ERROR_COMPLAIN(
      OTANVSSet(reinterpret_cast<uint8_t *>(&confirm_result),
                NVSKey::OTA_CONFIRM_RESULT, sizeof(ConfirmResult)),
      "Failed to save OTA confirm result");
}

esp_err_t DeviceController::confirm() {
  ESP_LOGI(TAG, "Confirming OTA");
  esp_err_t ret = ESP_OK;
  uint8_t running_partition_hash[HASH_LENGTH];
  const esp_partition_t *partition = esp_ota_get_running_partition();
  ESP_GOTO_ON_ERROR(esp_partition_get_sha256(partition, running_partition_hash),
                    ROLLBACK, TAG, "esp_partition_get_sha256");
  ESP_GOTO_ON_ERROR(esp_ota_mark_app_valid_cancel_rollback(), ROLLBACK, TAG,
                    "esp_ota_mark_app_valid_cancel_rollback");
  ESP_LOGI(TAG, "OTA confirmed, powering up");
  save_confirm_result(true);
  status_ = DeviceControllerStatus::POWER_UP;
  return ret;

ROLLBACK:
  ESP_LOGE(TAG, "Failed to confirm OTA, rolling back");
  save_confirm_result(false);
  rollback();
  return ret;
}

void DeviceController::try_confirm() {
  if (this->is_in_ota()) {
    this->confirm();
  }
}

void DeviceController::set_upgrading(bool upgrading) {
  upgrading_ = upgrading;
  if (upgrading_) {
    PowerAllocator &allocator = PowerAllocator::GetInstance();
    PortManager &pm = PortManager::GetInstance();
    pm.ForEach([](Port &port) {
      port.Shutdown();
      return true;
    });
    allocator.Stop();
  }
}

esp_err_t DeviceController::power_off() {
  if (status_ == DeviceControllerStatus::POWER_DOWN) {
    return ESP_OK;
  }
  ESP_RETURN_ON_FALSE(status_ == DeviceControllerStatus::POWER_UP,
                      ESP_ERR_INVALID_STATE, TAG,
                      "Attempted to power off while not in POWER_UP state");
  status_ = DeviceControllerStatus::POWER_DOWN;
  ESP_ERROR_COMPLAIN(DisplayManager::GetInstance().DisplayOff(), "DisplayOff");
  PortManager &pm = PortManager::GetInstance();
  pm.ForEach([](Port &port) {
    port.Close();
    return true;
  });
  return ESP_OK;
}

esp_err_t DeviceController::power_on() {
  if (status_ == DeviceControllerStatus::POWER_UP) {
    return ESP_OK;
  }
  status_ = DeviceControllerStatus::POWER_UP;
  ESP_ERROR_COMPLAIN(DisplayManager::GetInstance().DisplayOn(), "DisplayOn");
  PortManager &pm = PortManager::GetInstance();
  pm.ForEach([](Port &port) {
    port.Open();
    return true;
  });
  return ESP_OK;
}

bool DeviceController::should_reboot() {
  return reboot_at_ > 0 && esp_timer_get_time() > reboot_at_;
}

void DeviceController::reboot_after(int delay_ms) {
  reboot_at_ = esp_timer_get_time() + delay_ms * 1e3;
}

void DeviceController::reboot() {
  ESP_LOGI(TAG, "Rebooting");
  esp_restart();
}
