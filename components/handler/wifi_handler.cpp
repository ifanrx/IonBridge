#include "wifi_handler.h"

#include <cstdint>
#include <vector>

#include "controller.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi_types_generic.h"
#include "ionbridge.h"
#include "machine_info.h"
#include "nvs.h"
#include "sdkconfig.h"
#include "telemetry_task.h"
#include "utils.h"
#include "wifi.h"
#include "wifi_manager.h"

static const char *TAG = "WiFiHandler";
static std::string wifi_ssid_, wifi_password_;

esp_err_t WiFiHandler::ScanWifi(const std::vector<uint8_t> &request,
                                std::vector<uint8_t> &response) {
  static bool first_scan = true;
  constexpr int kScanWaitMS = 3000;
  constexpr int kResponseMaxSize = 500;

  ESP_LOGI(TAG, "Scanning for WiFi networks");
  WiFiManager &wifi_manager = WiFiManager::GetInstance();
  ssid_info_t ssids[CONFIG_ESP_WIFI_SCAN_LIST_SIZE] = {};
  if (first_scan && DeviceController::GetInstance().is_associated()) {
    ESP_LOGI(TAG, "Device is associated, use previous scan result");
    wifi_manager.GetScanAps(ssids);
  } else {
    TelemetryTask &telemetry_task = TelemetryTask::GetInstance();
    // stop wifi data transfer
    telemetry_task.Pause();
    wifi_controller.StartScan();
    DELAY_MS(kScanWaitMS);
    if (wifi_controller.GetStateType() == WiFiStateType::SCANNING) {
      DELAY_MS(kScanWaitMS);
    }
    wifi_manager.GetScanAps(ssids);
    telemetry_task.Resume();
  }
  first_scan = false;

  int ap_count = 0;
  response.emplace_back(0);

  // Prepare the response vector with initial size to include the count byte
  size_t bytes_written = 2;  // start after count bytes

  for (int i = 0; i < CONFIG_ESP_WIFI_SCAN_LIST_SIZE; i++) {
    if (ssids[i].size == 0) {
      continue;
    }
    ap_count++;
    ssid_info_t ssid = ssids[i];

    // +3 for the length byte, RSSI and Auth mode
    size_t total_length = 3 + ssid.size;
    if (bytes_written + total_length > kResponseMaxSize) {
      break;  // Stop if we cannot fit more SSIDs
    }

    // Store SSID length
    response.emplace_back(ssid.size);
    // Store SSID string
    response.insert(response.end(), ssid.ssid, ssid.ssid + ssid.size);
    response.emplace_back(ssid.rssi);
    response.emplace_back(ssid.auth_mode);
    response.emplace_back(wifi_manager.HasSSID(ssid.ssid));
    bytes_written += total_length;
  }
  response[0] = ap_count;

  return ESP_OK;  // Indicate success
}

esp_err_t WiFiHandler::SetWiFiSSID(const std::vector<uint8_t> &request,
                                   std::vector<uint8_t> &response) {
  if (request.size() < 1) {
    ESP_LOGW(TAG, "Invalid request size");
    return ESP_FAIL;
  }

  wifi_ssid_.assign(request.begin(), request.end());
  ESP_LOGI(TAG, "Selected SSID: %s", wifi_ssid_.c_str());
  return ESP_OK;
}

esp_err_t WiFiHandler::SetWiFiPassword(const std::vector<uint8_t> &request,
                                       std::vector<uint8_t> &response) {
  if (request.size() < 1) {
    ESP_LOGW(TAG, "Invalid request size");
    return ESP_FAIL;
  }

  if (wifi_ssid_.empty()) {
    ESP_LOGW(TAG, "SSID not set");
    return ESP_FAIL;
  }

  wifi_password_.assign(request.begin(), request.end());
  if (wifi_controller.SwitchWiFi(wifi_ssid_.c_str(), wifi_password_.c_str())) {
    return ESP_OK;
  }

  return ESP_FAIL;
}

esp_err_t WiFiHandler::ResetWiFi(const std::vector<uint8_t> &request,
                                 std::vector<uint8_t> &response) {
  wifi_ssid_.clear();
  wifi_password_.clear();
  ESP_ERROR_COMPLAIN(WiFiManager::GetInstance().RemoveAll(),
                     "WiFiManager RemoveAll");
  return wifi_disconnect();
}

esp_err_t WiFiHandler::GetWiFiStatus(const std::vector<uint8_t> &request,
                                     std::vector<uint8_t> &response) {
  ssid_detail_t ssid_detail;
  ESP_RETURN_ON_ERROR(get_connected_ssid_detail(&ssid_detail), TAG,
                      "get_connected_ssid_detail");
  response.resize(sizeof(ssid_detail_t));
  size_t write_size = serialize_ssid_detail(&ssid_detail, response.data(),
                                            sizeof(ssid_detail_t));
  response.resize(write_size);
  return ESP_OK;
}

esp_err_t WiFiHandler::GetDeviceWiFiAddress(const std::vector<uint8_t> &request,
                                            std::vector<uint8_t> &response) {
  const uint8_t *addr = MachineInfo::GetInstance().GetWifiMac().data();
  response.insert(response.end(), addr, addr + 6);
  return ESP_OK;
}

esp_err_t WiFiHandler::SetWiFiSSIDAndPassword(
    const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  if (request.size() < 3) {
    ESP_LOGW(TAG, "Request size is too small");
    return ESP_FAIL;
  }

  WiFiManager &wifi_manager = WiFiManager::GetInstance();
  uint8_t ssid_length = request[0];
  uint8_t auth_mode = request[1];
  bool has_password = (request.size() - ssid_length - 2) > 0;

  ESP_RETURN_ON_FALSE(ssid_length > 0 && ssid_length <= WIFI_SSID_MAXIMUM,
                      ESP_ERR_INVALID_ARG, TAG, "SSID length is invalid");
  ESP_RETURN_ON_FALSE(request.size() >= ssid_length + 2, ESP_ERR_INVALID_ARG,
                      TAG, "Request size is invalid");

  wifi_ssid_.assign(request.begin() + 2, request.begin() + 2 + ssid_length);

  if (auth_mode == WIFI_AUTH_OPEN) {
    ESP_LOGI(TAG, "No password required for open network");
    wifi_password_ = "";
  } else if (has_password) {
    wifi_password_.assign(request.begin() + 2 + ssid_length, request.end());
  } else {
    ESP_LOGI(TAG, "Using stored password");
    uint8_t stored_auth_mode;
    ESP_RETURN_ON_ERROR(wifi_manager.GetWiFiBySSID(wifi_ssid_, &wifi_password_,
                                                   &stored_auth_mode),
                        TAG, "Failed to get WiFi by SSID: %s",
                        wifi_ssid_.c_str());
  }

  ESP_RETURN_ON_FALSE(wifi_password_.length() <= WIFI_PASSWD_MAXIMUM,
                      ESP_ERR_INVALID_ARG, TAG, "Password length is invalid");

  ESP_LOGI(TAG, "Selected SSID: %s", wifi_ssid_.c_str());

  if (wifi_controller.SwitchWiFi(wifi_ssid_.c_str(), wifi_password_.c_str())) {
    return ESP_OK;
  }

  return ESP_FAIL;
}

esp_err_t WiFiHandler::GetWiFiRecords(const std::vector<uint8_t> &request,
                                      std::vector<uint8_t> &response) {
  WiFiManager &wifi_manager = WiFiManager::GetInstance();

  // WiFiStorage wifi_storage;
  uint8_t count = 0;
  response.emplace_back(0);  // Placeholder for count

  size_t index = 0;
  for (const uint32_t &h : wifi_manager.GetHash()) {
    if (h == 0) {
      index++;
      continue;
    }

    std::string ssid, passwd;
    uint8_t auth_mode;
    esp_err_t err = wifi_manager.GetWiFiByIndex(static_cast<uint8_t>(index),
                                                &ssid, &passwd, &auth_mode);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
      ESP_LOGE(TAG, "Failed to get WiFi entry at index %d", index);
      return err;
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
      index++;
      continue;
    }

    size_t ssid_length = ssid.length();
    if (ssid_length >= WIFI_SSID_MAXIMUM) {
      ESP_LOGE(TAG, "SSID length at slot %zu exceeds maximum", index);
      return ESP_ERR_INVALID_SIZE;
    }

    // Increment the count as a valid entry is found
    count++;

    // Append SSID length as a single byte
    response.emplace_back(static_cast<uint8_t>(ssid_length));

    // Append the SSID bytes to the response
    const uint8_t *ssid_bytes = reinterpret_cast<const uint8_t *>(ssid.c_str());
    response.insert(response.end(), ssid_bytes, ssid_bytes + ssid_length);

    // Append the authentication mode as a single byte
    response.emplace_back(static_cast<uint8_t>(auth_mode));
    index++;
  }

  // Update the first byte of the response with the actual count
  response[0] = count;
  return ESP_OK;
}

enum Operation {
  CREATE = 0,
  UPDATE = 1,
  DELETE = 2,
};

esp_err_t WiFiHandler::OperateWiFiRecord(const std::vector<uint8_t> &request,
                                         std::vector<uint8_t> &response) {
  static constexpr size_t REQUEST_HEADER_SIZE = 3;
  ESP_RETURN_ON_FALSE(request.size() >= REQUEST_HEADER_SIZE,
                      ESP_ERR_INVALID_ARG, TAG, "Invalid request size");

  WiFiManager &wifi_manager = WiFiManager::GetInstance();
  size_t param_offset = 0;
  uint8_t operation = request[param_offset++];
  uint8_t ssid_length = request[param_offset++];
  uint8_t authmode = request[param_offset++];

  ESP_RETURN_ON_FALSE(ssid_length > 0 && ssid_length <= WIFI_SSID_MAXIMUM,
                      ESP_ERR_INVALID_ARG, TAG, "Invalid SSID length");
  ESP_RETURN_ON_FALSE(request.size() >= ssid_length + REQUEST_HEADER_SIZE,
                      ESP_ERR_INVALID_ARG, TAG, "Invalid request size");

  std::string ssid(
      reinterpret_cast<const char *>(request.data()) + param_offset,
      ssid_length);
  param_offset += ssid_length;
  size_t passwd_length = request.size() - param_offset;
  ESP_RETURN_ON_FALSE(passwd_length <= WIFI_PASSWD_MAXIMUM, ESP_ERR_INVALID_ARG,
                      TAG, "Invalid password length");
  std::string passwd(
      reinterpret_cast<const char *>(request.data()) + param_offset,
      passwd_length);

  uint8_t entry_auth_mode =
      (authmode != WIFI_AUTH_OPEN) ? WIFI_AUTH_WPA3_PSK : WIFI_AUTH_OPEN;

  switch (operation) {
    case Operation::CREATE:
    case Operation::UPDATE: {
      ESP_RETURN_ON_ERROR(wifi_manager.Add(ssid, passwd, entry_auth_mode), TAG,
                          "Failed to create/update WiFi credentials");
      break;
    }
    case Operation::DELETE: {
      ESP_RETURN_ON_ERROR(wifi_manager.Remove(ssid), TAG,
                          "Failed to remove WiFi credentials");
      break;
    }
    default: {
      ESP_LOGW(TAG, "Invalid operation");
      return ESP_FAIL;
    }
  }

  return ESP_OK;
}

esp_err_t WiFiHandler::GetWiFiStateMachine(const std::vector<uint8_t> &request,
                                           std::vector<uint8_t> &response) {
  response.emplace_back(static_cast<uint8_t>(wifi_controller.GetStateType()));
  return ESP_OK;
}

esp_err_t WiFiHandler::SetWiFiStateMachine(const std::vector<uint8_t> &request,
                                           std::vector<uint8_t> &response) {
  ESP_RETURN_ON_FALSE(request.size() == 1, ESP_ERR_INVALID_ARG, TAG,
                      "Invalid request size");
  uint8_t state = request[0];
  if (state >= static_cast<uint8_t>(WiFiStateType::COUNT)) {
    ESP_LOGW(TAG, "Invalid state");
    return ESP_FAIL;
  }
  wifi_controller.SetState(static_cast<WiFiStateType>(state));
  return ESP_OK;
}
