#include "ota_handler.h"

#include <sys/param.h>

#include <cstring>
#include <vector>

#include "ble.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "sdkconfig.h"
#include "upgrade.h"
#include "wifi.h"

static const char *TAG = "OTAHandler";

esp_err_t OTAHandler::StartOTA(AppContext &ctx,
                               const std::vector<uint8_t> &request,
                               std::vector<uint8_t> &response) {
  firmware_version new_version;

  // Log the incoming request data
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, request.data(), request.size(), ESP_LOG_DEBUG);
  // Check if the payload size matches the expected size for a new version
  ESP_RETURN_ON_FALSE(request.size() == sizeof(new_version),
                      ESP_ERR_INVALID_ARG, TAG,
                      "Invalid payload size: want %d, got %d",
                      static_cast<int>(sizeof(new_version)), request.size());

  // Copy request data into the new_version structure
  memcpy(&new_version, request.data(), sizeof(new_version));

#ifdef CONFIG_MCU_MODEL_SW3566
  ESP_LOGI(
      TAG,
      "New version - SW3566 %d.%d.%d, FPGA %d.%d.%d, ESP32 %d.%d.%d, force: %d",
      new_version.sw3566_version[0], new_version.sw3566_version[1],
      new_version.sw3566_version[2], new_version.fpga_version[0],
      new_version.fpga_version[1], new_version.fpga_version[2],
#else
  ESP_LOGI(TAG, "New version - ESP32 %d.%d.%d, force: %d",
#endif
      new_version.esp32_version[0], new_version.esp32_version[1],
      new_version.esp32_version[2], new_version.force);

  // Ensure WiFi is connected before proceeding with OTA
  ESP_RETURN_ON_FALSE(wifi_is_connected(), ESP_ERR_WIFI_NOT_CONNECT, TAG,
                      "WiFi is not connected");

  // De-initialize BLE stack to release resources
  ble_deinit();
  if (start_upgrade_task(&new_version) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start upgrade task, rebooting in 2 seconds");
    ctx.controller.reboot_after(2 * 1000);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "OTA precheck passed, starting actual upgrade");
  ctx.controller.set_upgrading(true);
  return ESP_OK;
}
