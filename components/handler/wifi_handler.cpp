#include "wifi_handler.h"

#include <cstdint>
#include <cstring>
#include <vector>

#include "command.pb.h"
#include "controller.h"
#include "display_manager.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi_types_generic.h"
#include "ionbridge.h"
#include "machine_info.h"
#include "nvs.h"
#include "pb_encode.h"
#include "sdkconfig.h"
#include "telemetry_task.h"
#include "utils.h"
#include "wifi.h"
#include "wifi_manager.h"

#define SCAN_WAIT_MS 3000

using namespace WiFiHandler;

static const char *TAG = "WiFiHandler";
static std::string wifi_ssid_, wifi_password_;

esp_err_t FromBLE::ScanWiFi(const std::vector<uint8_t> &request,
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

esp_err_t FromBLE::SetWiFiSSID(const std::vector<uint8_t> &request,
                               std::vector<uint8_t> &response) {
  if (request.size() < 1) {
    ESP_LOGW(TAG, "Invalid request size");
    return ESP_FAIL;
  }

  wifi_ssid_.assign(request.begin(), request.end());
  ESP_LOGI(TAG, "Selected SSID: %s", wifi_ssid_.c_str());
  return ESP_OK;
}

esp_err_t FromBLE::SetWiFiPassword(const std::vector<uint8_t> &request,
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

static esp_err_t reset_wifi() {
  wifi_ssid_.clear();
  wifi_password_.clear();
  ESP_ERROR_COMPLAIN(WiFiManager::GetInstance().RemoveAll(),
                     "WiFiManager RemoveAll");
  return wifi_disconnect();
}

esp_err_t FromBLE::ResetWiFi(const std::vector<uint8_t> &request,
                             std::vector<uint8_t> &response) {
  return reset_wifi();
}

esp_err_t FromBLE::GetWiFiStatus(const std::vector<uint8_t> &request,
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

esp_err_t FromBLE::GetDeviceWiFiAddress(const std::vector<uint8_t> &request,
                                        std::vector<uint8_t> &response) {
  const uint8_t *addr = MachineInfo::GetInstance().GetWifiMac().data();
  response.insert(response.end(), addr, addr + 6);
  return ESP_OK;
}

esp_err_t FromBLE::SetWiFiSSIDAndPassword(const std::vector<uint8_t> &request,
                                          std::vector<uint8_t> &response) {
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

esp_err_t FromBLE::GetWiFiStateMachine(const std::vector<uint8_t> &request,
                                       std::vector<uint8_t> &response) {
  uint8_t state = static_cast<uint8_t>(wifi_controller.GetStateType());
  response.emplace_back(state);
  return ESP_OK;
}

static bool scan_wifi_encode_callback(pb_ostream_t *stream,
                                      const pb_field_t *field,
                                      void *const *arg) {
  WiFiManager &wifi_manager = WiFiManager::GetInstance();
  ssid_info_t ssids[CONFIG_ESP_WIFI_SCAN_LIST_SIZE] = {};
  int ap_count = wifi_manager.GetScanAps(ssids);
  for (int i = 0; i < ap_count; i++) {
    if (ssids[i].size == 0) {
      continue;
    }
    if (!pb_encode_tag_for_field(stream, field)) {
      return false;
    }
    pb_SSIDInfo pb_ssid = pb_SSIDInfo_init_zero;
    ssid_info_t ssid = ssids[i];

    strlcpy(pb_ssid.ssid, ssid.ssid, sizeof(pb_ssid.ssid));
    pb_ssid.rssi = ssid.rssi;
    pb_ssid.auth_mode = (pb_WiFiAuthMode)ssid.auth_mode;
    pb_ssid.saved = wifi_manager.HasSSID(ssid.ssid);

    if (!pb_encode_submessage(stream, pb_SSIDInfo_fields, &pb_ssid)) {
      return false;
    }
  }
  return true;
}

esp_err_t FromMQTT::ScanWiFi(const pb_CommandRequest &request,
                             pb_CommandResponse &response) {
  ESP_LOGI(TAG, "Scanning for WiFi networks");
  response.which_payload = pb_CommandResponse_scan_wifi_tag;
  TelemetryTask &telemetry_task = TelemetryTask::GetInstance();
  // stop wifi data transfer
  telemetry_task.Pause();
  wifi_controller.StartScan();
  DELAY_MS(SCAN_WAIT_MS);
  if (wifi_controller.GetStateType() == WiFiStateType::SCANNING) {
    DELAY_MS(SCAN_WAIT_MS);
  }
  telemetry_task.Resume();

  response.payload.scan_wifi.ssid_list.funcs.encode = scan_wifi_encode_callback;
  return ESP_OK;
}

esp_err_t FromMQTT::ResetWiFi(const pb_CommandRequest &request,
                              pb_CommandResponse &response) {
  return reset_wifi();
}

esp_err_t FromMQTT::GetWiFiStatus(const pb_CommandRequest &request,
                                  pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_get_wifi_status_tag;
  ssid_detail_t ssid_detail;
  ESP_RETURN_ON_ERROR(get_connected_ssid_detail(&ssid_detail), TAG,
                      "get_connected_ssid_detail");
  hexlify(ssid_detail.bssid, sizeof(ssid_detail.bssid),
          response.payload.get_wifi_status.bssid,
          sizeof(response.payload.get_wifi_status.bssid));
  response.payload.get_wifi_status.rssi = ssid_detail.rssi;
  response.payload.get_wifi_status.channel = ssid_detail.channel;
  response.payload.get_wifi_status.protocol =
      ssid_detail.phy_11b | (ssid_detail.phy_11g << 1) |
      (ssid_detail.phy_11n << 2) | (ssid_detail.phy_lr << 3) |
      (ssid_detail.phy_11ax << 4) | (ssid_detail.wps << 5) |
      (ssid_detail.ftm_responder << 6) | (ssid_detail.ftm_initiator << 7);
  std::memcpy(response.payload.get_wifi_status.ssid, ssid_detail.ssid,
              sizeof(ssid_detail.ssid));
  return ESP_OK;
}

esp_err_t FromMQTT::GetDeviceWiFiAddress(const pb_CommandRequest &request,
                                         pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_get_wifi_mac_tag;
  const std::array<uint8_t, 6> addr = MachineInfo::GetInstance().GetWifiMac();
  hexlify(addr.data(), addr.size(), response.payload.get_wifi_mac.mac,
          sizeof(response.payload.get_wifi_mac.mac));
  return ESP_OK;
}

esp_err_t FromMQTT::SetWiFiSSIDAndPassword(const pb_CommandRequest &request,
                                           pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_set_wifi_ssid_and_password_tag;

  WiFiManager &wifi_manager = WiFiManager::GetInstance();
  wifi_ssid_.assign(request.payload.set_wifi_ssid_and_password.ssid);
  if (request.payload.set_wifi_ssid_and_password.auth_mode ==
      pb_WiFiAuthMode_WIFI_AUTH_OPEN) {
    ESP_LOGI(TAG, "No password required for open network");
    wifi_password_ = "";
  } else if (strlen(request.payload.set_wifi_ssid_and_password.password) > 0) {
    wifi_password_.assign(request.payload.set_wifi_ssid_and_password.password);
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

static bool get_wifi_records_encode_callback(pb_ostream_t *stream,
                                             const pb_field_t *field,
                                             void *const *arg) {
  WiFiManager &wifi_manager = WiFiManager::GetInstance();
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
      return false;
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
      index++;
      continue;
    }

    size_t ssid_length = ssid.length();
    if (ssid_length >= WIFI_SSID_MAXIMUM) {
      ESP_LOGE(TAG, "SSID length at slot %zu exceeds maximum", index);
      return false;
    }
    index++;

    if (!pb_encode_tag_for_field(stream, field)) {
      return false;
    }
    pb_WiFiRecord pb_wifi_record = pb_WiFiRecord_init_zero;
    strlcpy(pb_wifi_record.ssid, ssid.c_str(), sizeof(pb_wifi_record.ssid));
    pb_wifi_record.auth_mode = (pb_WiFiAuthMode)auth_mode;
    if (!pb_encode_submessage(stream, pb_WiFiRecord_fields, &pb_wifi_record)) {
      return false;
    }
  }
  return true;
}

esp_err_t FromMQTT::GetWiFiRecords(const pb_CommandRequest &request,
                                   pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_get_wifi_records_tag;
  response.payload.get_wifi_records.records.funcs.encode =
      get_wifi_records_encode_callback;
  return ESP_OK;
}

esp_err_t FromMQTT::OperateWiFiRecord(const pb_CommandRequest &request,
                                      pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_operate_wifi_record_tag;

  std::string ssid(request.payload.operate_wifi_record.ssid);
  std::string passwd(request.payload.operate_wifi_record.password);
  uint8_t auth_mode = request.payload.operate_wifi_record.auth_mode;
  uint8_t entry_auth_mode =
      (auth_mode != WIFI_AUTH_OPEN) ? WIFI_AUTH_WPA3_PSK : WIFI_AUTH_OPEN;

  switch (request.payload.operate_wifi_record.action) {
    case pb_ManageAction_CREATE:
    case pb_ManageAction_UPDATE: {
      ESP_RETURN_ON_ERROR(
          WiFiManager::GetInstance().Add(ssid, passwd, entry_auth_mode), TAG,
          "Failed to create/update WiFi credentials");
      break;
    }
    case pb_ManageAction_DELETE: {
      ESP_RETURN_ON_ERROR(WiFiManager::GetInstance().Remove(ssid), TAG,
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

esp_err_t FromMQTT::GetWiFiStateMachine(const pb_CommandRequest &request,
                                        pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_get_wifi_state_machine_tag;
  response.payload.get_wifi_state_machine.state =
      static_cast<pb_WiFiState>(wifi_controller.GetStateType());
  return ESP_OK;
}

esp_err_t FromMQTT::SetWiFiStateMachine(const pb_CommandRequest &request,
                                        pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_set_wifi_state_machine_tag;
  WiFiStateType state =
      static_cast<WiFiStateType>(request.payload.set_wifi_state_machine.state);
  ESP_RETURN_ON_FALSE(state < WiFiStateType::COUNT, ESP_ERR_INVALID_ARG, TAG,
                      "Invalid state");
  wifi_controller.SetState(state);
  return ESP_OK;
}
