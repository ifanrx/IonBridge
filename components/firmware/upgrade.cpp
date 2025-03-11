#include "upgrade.h"

#include <cstdio>
#include <cstring>

#include "animation.h"
#include "download.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "firmware.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "https_ota.h"
#include "ionbridge.h"
#include "machine_info.h"
#include "mqtt_app.h"
#include "nvs_namespace.h"
#include "portmacro.h"
#include "sdkconfig.h"
#include "telemetry_task.h"
#include "version.h"

#define UPGRADE_TASK_STACK_SIZE CONFIG_UPGRADE_TASK_STACK_SIZE
#define UPGRADE_TASK_PRIORITY CONFIG_UPGRADE_TASK_PRIORITY
#define MAX_URL_LEN 256

static const char *TAG = "Upgrade";

#define GET_FIRMWARE_URL(type, version, url, goto_tag)                   \
  do {                                                                   \
    ESP_GOTO_ON_ERROR(get_firmware_url(type, version, url, sizeof(url)), \
                      goto_tag, TAG, "get_firmware_url");                \
  } while (0)

static bool upgrade_in_progress = false;

void upgrade_task(void *arg);

esp_err_t start_upgrade_task(const firmware_version *new_version) {
  static firmware_version new_version_storage;
  ESP_RETURN_ON_FALSE(!upgrade_in_progress, ESP_ERR_INVALID_STATE, TAG,
                      "Upgrade already in progress");
  memcpy(&new_version_storage, new_version, sizeof(firmware_version));
  BaseType_t created =
      xTaskCreate(upgrade_task, "upgrade", UPGRADE_TASK_STACK_SIZE,
                  (void *)&new_version_storage, UPGRADE_TASK_PRIORITY, nullptr);
  if (created == pdPASS) {
    upgrade_in_progress = true;
    return ESP_OK;
  }

  ESP_LOGE(TAG, "Failed to create upgrade task");
  return ESP_FAIL;
}

void upgrade_task(void *arg) {
  esp_err_t ret = ESP_OK;
  firmware_version new_version;
  memcpy(&new_version, (firmware_version *)arg, sizeof(firmware_version));

  MachineInfo &machine_info = MachineInfo::GetInstance();
  Version new_esp32_version(new_version.esp32_version),
      current_esp32_version(machine_info.GetESP32Version());
  FirmwareType type = FIRMWARE_TYPE_ESP32;

#ifdef CONFIG_MCU_MODEL_SW3566
  Version new_sw3566_version(new_version.sw3566_version),
      new_fpga_version(new_version.fpga_version),
      current_sw3566_version(machine_info.GetMCUVersion()),
      current_fpga_version(machine_info.GetFPGAVersion());

  type = FIRMWARE_TYPE_SW3566;
#endif

  ESP_LOGI(
      TAG,
#ifdef CONFIG_MCU_MODEL_SW3566
      "Starting upgrade task, new versions: SW3566: %s, FPGA: %s, ESP32: %s",
      new_sw3566_version.toString().c_str(),
      new_fpga_version.toString().c_str(),
#else
      "Starting upgrade task, new version: ESP32: %s",
#endif
      new_esp32_version.toString().c_str());

  bool is_upgrade_required = false;
  char cert_buffer[2048], url[MAX_URL_LEN];
  size_t cert_len = sizeof(cert_buffer);

  // Report the new version to the MQTT broker
  MQTTClient *client = MQTTClient::GetInstance();
  TelemetryTask *telemetry_task = TelemetryTask::GetInstance();

  WAIT_FOR_CONDITION_OR_GOTO(20, client->Connected(), 100, UPGRADE_FAILED,
                             "MQTT connection timeout for TelemetryTask");

  if (telemetry_task) {
    telemetry_task->ReportUpgradeStart();
  }

  // Stop MQTT client to free heap memory
  client->Stop(true);

  ESP_GOTO_ON_ERROR(CertsNVSGet(cert_buffer, &cert_len, NVSKey::CA_CERT),
                    UPGRADE_FAILED, TAG, "Failed to get CA certificate");

#ifdef CONFIG_MCU_MODEL_SW3566
  if (new_version.force || new_sw3566_version != current_sw3566_version) {
    ESP_LOGI(TAG, "Upgrading SW3566 from %s to %s, force: %d",
             current_sw3566_version.toString().c_str(),
             new_sw3566_version.toString().c_str(), new_version.force);

    AnimationController::GetInstance().StartAnimation(
        AnimationId::DOWNLOAD_STAGE1, true);

    is_upgrade_required = true;
    type = FIRMWARE_TYPE_SW3566;

    GET_FIRMWARE_URL(type, new_sw3566_version, url, UPGRADE_FAILED);

    ESP_GOTO_ON_ERROR(
        download_firmware(url, cert_buffer,
                          current_sw3566_version.toString().c_str(),
                          new_sw3566_version.toString().c_str(), type),
        UPGRADE_FAILED, TAG, "SW3566 firmware download failed");
  }

  if (new_version.force || new_fpga_version != current_fpga_version) {
    ESP_LOGI(TAG, "Upgrading FPGA from %s to %s, force: %d",
             current_fpga_version.toString().c_str(),
             new_fpga_version.toString().c_str(), new_version.force);

    AnimationController::GetInstance().StartAnimation(
        AnimationId::DOWNLOAD_STAGE2, true);

    is_upgrade_required = true;
    type = FIRMWARE_TYPE_FPGA;

    GET_FIRMWARE_URL(type, new_fpga_version, url, UPGRADE_FAILED);

    ESP_GOTO_ON_ERROR(
        download_firmware(url, cert_buffer,
                          current_fpga_version.toString().c_str(),
                          new_fpga_version.toString().c_str(), type),
        UPGRADE_FAILED, TAG, "FPGA firmware download failed");
  }
#endif

  if (new_version.force || new_esp32_version != current_esp32_version) {
    ESP_LOGI(TAG, "Upgrading ESP32 from %s to %s, force: %d",
             current_esp32_version.toString().c_str(),
             new_esp32_version.toString().c_str(), new_version.force);

    AnimationController::GetInstance().StartAnimation(
        AnimationId::DOWNLOAD_STAGE3, true);

    is_upgrade_required = true;
    type = FIRMWARE_TYPE_ESP32;

    GET_FIRMWARE_URL(type, new_esp32_version, url, UPGRADE_FAILED);

    ESP_GOTO_ON_ERROR(
        https_ota(url, cert_buffer, current_esp32_version.toString().c_str(),
                  new_esp32_version.toString().c_str(), new_version.force),
        UPGRADE_FAILED, TAG, "ESP32 OTA failed");

    ESP_LOGI(TAG, "ESP32 upgraded successfully");
  }

  if (is_upgrade_required) {
    ESP_LOGI(TAG, "Upgrade completed successfully");
    goto UPGRADE_FINISHED;
  }

  ESP_LOGW(TAG, "No new firmware available for upgrade");
  ret = ESP_ERR_INVALID_VERSION;

UPGRADE_FAILED:
  if (client && telemetry_task) {
    ESP_LOGI(TAG, "Reporting upgrade error");
    client->ForceStart();

    WAIT_FOR_CONDITION_OR_GOTO(
        10, client->Connected(), 100, UPGRADE_FINISHED,
        "Connection timeout while reporting upgrade error");

    telemetry_task->ReportUpgradeError(ret);
    client->Stop();
  }

UPGRADE_FINISHED:
  ESP_LOGW(TAG, "Upgrade process finished. Restarting.");
  esp_restart();
  vTaskDelete(nullptr);
}
