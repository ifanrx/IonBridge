#include "ota_handler.h"

#include <sys/param.h>

#include "admin.pb.h"
#include "ble.h"
#include "command.pb.h"
#include "controller.h"
#include "data_types.pb.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "sdkconfig.h"
#include "upgrade.h"
#include "version.h"
#include "wifi.h"

static const char *TAG = "OTAHandler";

esp_err_t OTAHandler::FromMQTT::StartOTA(const pb_CommandRequest &request,
                                         pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_start_ota_tag;

  const pb_StartOTAReq &ota_request = request.payload.start_ota;
  const pb_Version &esp32_version = ota_request.esp32_version;
  FirmwareVersions new_version;

#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
  const pb_Version &sw3566_version = ota_request.sw3566_version,
                   &fpga_version = ota_request.fpga_version;
  new_version.sw3566 =
      Version(sw3566_version.major, sw3566_version.minor, sw3566_version.patch);
  new_version.fpga =
      Version(fpga_version.major, fpga_version.minor, fpga_version.patch);
#endif
  new_version.esp32 =
      Version(esp32_version.major, esp32_version.minor, esp32_version.patch);
  new_version.force = request.payload.start_ota.force;

#ifdef CONFIG_MCU_MODEL_SW3566
  ESP_LOGI(TAG, "New version - SW3566 %s, FPGA %s, ESP32 %s, force: %d",
           new_version.sw3566.ToString().c_str(),
           new_version.fpga.ToString().c_str(),
#else
  ESP_LOGI(TAG, "New version - ESP32 %s, force: %d",
#endif
           new_version.esp32.ToString().c_str(), new_version.force);

  // Ensure WiFi is connected before proceeding with OTA
  ESP_RETURN_ON_FALSE(wifi_is_connected(), ESP_ERR_WIFI_NOT_CONNECT, TAG,
                      "WiFi is not connected");

  // De-initialize BLE stack to release resources
  ble_adv_stop();
  esp_err_t err = UpgradeManager::GetInstance().StartUpgrade(new_version);
  DeviceController &controller = DeviceController::GetInstance();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "Failed to start upgrade task, rebooting in 2 seconds");
    controller.reboot_after(2 * 1000);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "OTA precheck passed, starting actual upgrade");
  controller.set_upgrading(true);
  return ESP_OK;
}

