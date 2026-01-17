#include "upgrade.h"

#include <cstdio>
#include <cstring>
#include <vector>

#include "controller.h"
#include "display_animation.h"
#include "display_manager.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "firmware.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "https_ota.h"
#include "machine_info.h"
#include "nvs_key.h"
#include "nvs_namespace.h"
#include "portmacro.h"
#include "sdkconfig.h"
#include "telemetry_task.h"
#include "version.h"

#define UPGRADE_TASK_STACK_SIZE CONFIG_UPGRADE_TASK_STACK_SIZE
#define UPGRADE_TASK_PRIORITY CONFIG_UPGRADE_TASK_PRIORITY

static const char *TAG = "Upgrade";

using nvs_namespace::protected_data::CERTS;

// Firmware configuration table (Linus: "This is data, not code")
struct FirmwareConfig {
  FirmwareType type;
  const char *name;
  OTAStage ota_stage;
  const std::array<unsigned char, 3> &(
      MachineInfo::*get_current_version)() const;
};

// Compile-time firmware table selection
static constexpr FirmwareConfig FIRMWARE_TABLE[] = {
#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
#if !defined(CONFIG_DEVICE_HUMMING_BOARD)
    {FIRMWARE_TYPE_SW3566, "SW3566", OTAStage::DOWNLOAD_SW3566,
     &MachineInfo::GetMCUVersion},
#endif
    {FIRMWARE_TYPE_FPGA, "FPGA", OTAStage::DOWNLOAD_FPGA,
     &MachineInfo::GetFPGAVersion},
#endif
    {FIRMWARE_TYPE_ESP32, "ESP32", OTAStage::DOWNLOAD_ESP32,
     &MachineInfo::GetESP32Version},
};

// Helper to get new version from FirmwareVersions based on type
static Version GetNewVersion(const FirmwareVersions &versions,
                             FirmwareType type) {
  switch (type) {
#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
    case FIRMWARE_TYPE_SW3566:
      return versions.sw3566;
    case FIRMWARE_TYPE_FPGA:
      return versions.fpga;
#endif
    case FIRMWARE_TYPE_ESP32:
      return versions.esp32;
    default:
      return Version("0.0.0");
  }
}

// Task wrapper functions
static void UpgradeTaskWrapper(void *arg) {
  UpgradeManager &mgr = UpgradeManager::GetInstance();
  mgr.ExecuteUpgrade();
  vTaskDelete(nullptr);
}

static void DownloadTaskWrapper(void *arg) {
  UpgradeManager &mgr = UpgradeManager::GetInstance();
  mgr.ExecuteDownload();
  vTaskDelete(nullptr);
}

UpgradeManager::UpgradeManager() {
  cert_buffer[0] = '\0';
  url_buffer[0] = '\0';
}

// Common operation execution (Linus: "Write once, not twice")
esp_err_t UpgradeManager::ExecuteOperation(
    bool &operation_flag, const char *operation_name,
    std::function<void()> pre_process, std::function<void()> on_success,
    std::function<void(esp_err_t)> on_error) {
  const auto &new_version = new_version_storage;

  // RAII guard to automatically clear operation flag
  struct OperationGuard {
    bool &flag;
    ~OperationGuard() { flag = false; }
  } guard(operation_flag);

  LogVersions(new_version);

  // Error cleanup wrapper
  auto cleanup_on_error = [&](esp_err_t err) {
    on_error(err);
    return err;
  };

  // Execute pre-processing hook
  if (pre_process) {
    pre_process();
  }

  TelemetryTask::GetInstance().ReportUpgradeStart();

  esp_err_t ret = LoadCertificate();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "LoadCertificate failed");
    return cleanup_on_error(ret);
  }

  // Process firmware list
  auto firmware_list = GetFirmwareList(new_version);
  if (firmware_list.empty()) {
    ESP_LOGW(TAG, "No new firmware available for %s", operation_name);
    return cleanup_on_error(ESP_ERR_INVALID_VERSION);
  }

  for (const auto &fw : firmware_list) {
    ret = ProcessFirmware(fw, new_version.force);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "ProcessFirmware failed");
      return cleanup_on_error(ret);
    }
  }

  ESP_LOGI(TAG, "%s completed successfully", operation_name);
  if (on_success) {
    on_success();
  }
  return ESP_OK;
}

std::vector<UpgradeManager::FirmwareInfo> UpgradeManager::GetFirmwareList(
    const FirmwareVersions &new_version) {
  MachineInfo &machine_info = MachineInfo::GetInstance();
  std::vector<FirmwareInfo> firmwares;

  // Iterate firmware table (Linus: "Zero conditional logic")
  for (const auto &config : FIRMWARE_TABLE) {
    firmwares.push_back(
        {GetNewVersion(new_version, config.type),
         Version((machine_info.*(config.get_current_version))()), config.type,
         config.name, config.ota_stage});
  }

  return firmwares;
}

void UpgradeManager::LogVersions(const FirmwareVersions &new_version) {}

esp_err_t UpgradeManager::LoadCertificate() { return ESP_OK; }

esp_err_t UpgradeManager::ProcessFirmware(const FirmwareInfo &fw, bool force) {
  return ESP_OK;
}

esp_err_t UpgradeManager::StartUpgrade(const FirmwareVersions &new_version) {
  return ESP_OK;
}

esp_err_t UpgradeManager::StartDownload(const FirmwareVersions &new_version) {
  return ESP_OK;
}

esp_err_t UpgradeManager::ExecuteUpgrade() { return ESP_OK; }

esp_err_t UpgradeManager::ExecuteDownload() { return ESP_OK; }

esp_err_t UpgradeManager::ExecuteProgrammerDownload(
    Version version, const char *product_family) {
  return ESP_OK;
}
