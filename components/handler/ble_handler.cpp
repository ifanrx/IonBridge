#include "ble_handler.h"

#include <cstdint>
#include <vector>

#include "ble.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "ionbridge.h"
#include "machine_info.h"
#include "utils.h"

static const char *TAG = "BleHandler";

esp_err_t BleHandler::BLEEchoTest(const std::vector<uint8_t> &request,
                                  std::vector<uint8_t> &response) {
  ESP_LOGI(TAG, "BLE echo test, request size: %d", request.size());
  response.insert(response.end(), request.begin(), request.end());
  return ESP_OK;
}

esp_err_t BleHandler::GetDeviceBleAddress(const std::vector<uint8_t> &request,
                                          std::vector<uint8_t> &response) {
  const uint8_t *addr = MachineInfo::GetInstance().GetBleMac().data();
  ESP_LOGD(TAG, "BLE address: %s", FORMAT_MAC(addr));
  response.insert(response.end(), addr, addr + sizeof(addr));
  return ESP_OK;
}

esp_err_t BleHandler::SetBleState(const std::vector<uint8_t> &request,
                                  std::vector<uint8_t> &response) {
  if (request.size() != 1) {
    ESP_LOGE(TAG, "SetBleState invalid request size: %d", request.size());
    return ESP_ERR_INVALID_ARG;
  }
  bool advertising = request[0];
  if (advertising) {
    ESP_LOGI(TAG, "Starting BLE advertisement");
    ESP_RETURN_ON_FALSE(ble_start_advertising(true), ESP_FAIL, TAG,
                        "ble_start_advertising");
  } else {
    ESP_LOGI(TAG, "Stopping BLE advertisement");
    ESP_RETURN_ON_FALSE(ble_stop_advertising(), ESP_FAIL, TAG,
                        "ble_stop_advertising");
  }
  return ESP_OK;
}

esp_err_t BleHandler::GetBleRSSI(const std::vector<uint8_t> &request,
                                 std::vector<uint8_t> &response) {
  int8_t rssi = 0;
  ESP_RETURN_ON_ERROR(get_ble_conn_rssi(&rssi), TAG, "get_ble_conn_rssi");
  response.emplace_back(rssi);
  return ESP_OK;
}

esp_err_t BleHandler::GetBleMTU(const std::vector<uint8_t> &request,
                                std::vector<uint8_t> &response) {
  EMPLACE_BACK_INT16(response, get_ble_mtu());
  return ESP_OK;
}
