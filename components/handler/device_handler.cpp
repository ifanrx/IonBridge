#include "device_handler.h"

#include <sys/stat.h>
#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "controller.h"
#include "esp_check.h"
#include "esp_efuse.h"
#include "esp_err.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ionbridge.h"
#include "logging.h"
#include "machine_info.h"
#include "nvs_default.h"
#include "nvs_namespace.h"
#include "nvs_partition.h"
#include "port_manager.h"
#include "power_allocator.h"
#include "power_config.h"
#include "sdkconfig.h"
#include "syslog.h"
#include "telemetry_task.h"
#include "utils.h"
#include "wifi.h"
#include "wifi_manager.h"

static const char *TAG = "DeviceHandler";

enum ManageActionType {
  GET = 0,
  SET,
  RESET,
};

enum ManageFeature {
  FPGA_POWER_CONTROL = 0,
};

esp_err_t DeviceHandler::GetAPVersion(const std::vector<uint8_t> &request,
                                      std::vector<uint8_t> &response) {
  std::array<uint8_t, 3> esp32_version =
      MachineInfo::GetInstance().GetESP32Version();
  response.emplace_back(esp32_version[0]);
  response.emplace_back(esp32_version[1]);
  response.emplace_back(esp32_version[2]);

  ESP_LOGI(TAG, "GetAPVersion: %s",
           MachineInfo::GetInstance().FormatVersion(esp32_version).c_str());
  return ESP_OK;
}

esp_err_t DeviceHandler::AssociateDevice(const std::vector<uint8_t> &request,
                                         std::vector<uint8_t> &response) {
  size_t password_len = 0;
  bool reset_data = true;
  if (request.size() > 4) {
    password_len = request.size() - 1;
    reset_data = !request[request.size() - 1];
  } else {
    password_len = request.size();
  }
  ESP_RETURN_ON_FALSE(
      DeviceAuth::validatePassword(request.data(), password_len), ESP_FAIL, TAG,
      "Invalid password");

  if (reset_data) {
    ESP_LOGI(TAG, "Resetting user data");
    ESP_RETURN_ON_ERROR(ResetUserData(), TAG, "reset_user_data");
    WiFiManager::GetInstance().Reset();
    wifi_controller.Abort();
    PowerAllocator &allocator = PowerAllocator::GetInstance();
    if (allocator.Type() == PowerAllocatorType::TEMPORARY_ALLOCATOR) {
      allocator.SetStrategy<PowerSlowChargingStrategy>();
    }
  }
  uint8_t token;
  ESP_RETURN_ON_ERROR(NVSGetAuthToken(&token, reset_data), TAG,
                      "Failed to get token");

  DeviceController::GetInstance().mark_associated();
  response.emplace_back(token);
  return ESP_OK;
}

esp_err_t DeviceHandler::ResetDevice(const std::vector<uint8_t> &request,
                                     std::vector<uint8_t> &response) {
  ESP_LOGW(TAG, "Client request to reset device");
  NVSPartition partition(USER_DATA_NVS_PARTITION);
  if (partition.EraseAll() != ESP_OK) {
    return ESP_FAIL;
  }

  DeviceController::GetInstance().reboot_after(1000);
  return ESP_OK;
}

esp_err_t DeviceHandler::RebootDevice(const std::vector<uint8_t> &request,
                                      std::vector<uint8_t> &response) {
  ESP_LOGW(TAG, "Client request to reboot device");
  DeviceController::GetInstance().reboot_after(1000);
  return ESP_OK;
}

esp_err_t DeviceHandler::GetDeviceSerialNumber(
    const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  const std::string &serial_number = MachineInfo::GetInstance().GetPSN();
  ESP_RETURN_ON_FALSE(!serial_number.empty(), ESP_FAIL, TAG,
                      "Serial number is empty");
  response.insert(response.end(), serial_number.begin(), serial_number.end());
  return ESP_OK;
}

esp_err_t DeviceHandler::GetDeviceUpTime(const std::vector<uint8_t> &request,
                                         std::vector<uint8_t> &response) {
  int64_t uptime = esp_timer_get_time();
  EMPLACE_BACK_INT64(response, uptime);
  return ESP_OK;
}

#ifdef CONFIG_MCU_MODEL_SW3566
esp_err_t DeviceHandler::GetBPVersion(const std::vector<uint8_t> &request,
                                      std::vector<uint8_t> &response) {
  std::array<uint8_t, 3> bp_version =
      MachineInfo::GetInstance().GetMCUVersion();
  ESP_LOGI(TAG, "GetBPVersion: %s",
           MachineInfo::GetInstance().FormatVersion(bp_version).c_str());
  std::copy(bp_version.begin(), bp_version.end(), std::back_inserter(response));
  return ESP_OK;
}

esp_err_t DeviceHandler::GetFPGAVersion(const std::vector<uint8_t> &request,
                                        std::vector<uint8_t> &response) {
  std::array<uint8_t, 3> fpga_version =
      MachineInfo::GetInstance().GetFPGAVersion();
  ESP_LOGI(TAG, "GetFPGAVersion: %s",
           MachineInfo::GetInstance().FormatVersion(fpga_version).c_str());
  std::copy(fpga_version.begin(), fpga_version.end(),
            std::back_inserter(response));
  return ESP_OK;
}

esp_err_t DeviceHandler::GetZRLIBVersion(const std::vector<uint8_t> &request,
                                         std::vector<uint8_t> &response) {
  std::array<uint8_t, 3> zrlib_version =
      MachineInfo::GetInstance().GetZRLIBVersion();
  ESP_LOGI(TAG, "GetZRLIBVersion: %s",
           MachineInfo::GetInstance().FormatVersion(zrlib_version).c_str());
  std::copy(zrlib_version.begin(), zrlib_version.end(),
            std::back_inserter(response));
  return ESP_OK;
}

#endif

esp_err_t DeviceHandler::SwitchDevice(const std::vector<uint8_t> &request,
                                      std::vector<uint8_t> &response) {
#define CLOSE 0
#define OPEN 1
  if (request.size() != 1) {
    ESP_LOGW(TAG, "Invalid device switch length: %d", request.size());
    return ESP_FAIL;
  }
  uint8_t device_switch = request[0];
  DeviceController &controller = DeviceController::GetInstance();
  switch (device_switch) {
    case OPEN: {
      ESP_LOGI(TAG, "Switching device on");
      return controller.power_on();
      break;
    }
    case CLOSE: {
      ESP_LOGI(TAG, "Switching device off");
      return controller.power_off();
      break;
    }
    default: {
      ESP_LOGW(TAG, "Invalid device power state: %d", device_switch);
      return ESP_FAIL;
    }
  }
  return ESP_OK;

#undef CLOSE
#undef OPEN
}

esp_err_t DeviceHandler::GetDeviceSwitch(const std::vector<uint8_t> &request,
                                         std::vector<uint8_t> &response) {
  bool opened = DeviceController::GetInstance().is_power_on();
  response.emplace_back(static_cast<uint8_t>(opened));
  return ESP_OK;
}

esp_err_t DeviceHandler::GetDeviceModel(const std::vector<uint8_t> &request,
                                        std::vector<uint8_t> &response) {
  const std::string &model = MachineInfo::GetInstance().GetDeviceModel();
  response.insert(response.end(), model.begin(), model.end());
  return ESP_OK;
}

esp_err_t DeviceHandler::GetSecureBootDigest(
    const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  uint8_t key[32];
  ESP_RETURN_ON_ERROR(
      esp_efuse_read_block(EFUSE_BLK_KEY1, key, 0, sizeof(key) * 8), TAG,
      "esp_efuse_read_block");
  for (size_t i = 0; i < sizeof(key); i++) {
    response.emplace_back(key[i]);
  }
  return ESP_OK;
}

esp_err_t DeviceHandler::PingMQTTTelemetry(const std::vector<uint8_t> &request,
                                           std::vector<uint8_t> &response) {
  return TelemetryTask::GetInstance().ReportPingInfo();
}

esp_err_t DeviceHandler::PushLicense(const std::vector<uint8_t> &request,
                                     std::vector<uint8_t> &response) {
  if (request.size() <= 0) {
    ESP_LOGW(TAG, "Invalid license length: %d", request.size());
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "Push license length: %d", request.size());
  ESP_RETURN_ON_ERROR(
      LicenseNVSSet(request.data(), NVSKey::LICENSE, request.size()), TAG,
      "License::SaveLicense");
  return ESP_OK;
}

static esp_err_t _ping_http_event_handler(esp_http_client_event_t *evt) {
  switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
      ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
      break;
    case HTTP_EVENT_ON_CONNECTED:
      ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
      break;
    case HTTP_EVENT_HEADER_SENT:
      ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
      break;
    case HTTP_EVENT_ON_HEADER:
      ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key,
               evt->header_value);
      break;
    case HTTP_EVENT_ON_DATA:
      ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
      break;
    case HTTP_EVENT_ON_FINISH:
      ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
      break;
    case HTTP_EVENT_DISCONNECTED:
      ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
      break;
    case HTTP_EVENT_REDIRECT:
      ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
      break;
  }
  return ESP_OK;
}

esp_err_t DeviceHandler::PingHTTP(const std::vector<uint8_t> &request,
                                  std::vector<uint8_t> &response) {
  if (request.size() == 0) {
    ESP_LOGE(TAG, "Invalid PingHTTP request length: %d", request.size());
    return ESP_FAIL;
  }
  std::string url(request.begin(), request.end());
  ESP_LOGI(TAG, "PingHTTP request: %s", url.c_str());
  esp_http_client_config_t config;
  memset(&config, 0, sizeof(config));
  config.url = url.c_str();
  config.disable_auto_redirect = true;
  config.event_handler = _ping_http_event_handler;
  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == NULL) {
    ESP_LOGE(TAG, "Failed to initialize HTTP connection");
    return ESP_FAIL;
  }

  // GET
  return esp_http_client_perform(client);
}

struct DeviceInfo {
  char psn[32];
  char model[8];
  uint8_t ap_version[3];
  uint8_t bp_version[3];
  uint8_t fpga_version[3];
  uint8_t zrlib_version[3];
  uint8_t ble_mac[6];
  uint8_t wifi_mac[6];
  uint8_t bssid[6];
  char ssid[64];
  int8_t rssi;
  uint8_t wifi_protocol;
  uint8_t channel;
};

esp_err_t DeviceHandler::GetDeviceInfo(const std::vector<uint8_t> &request,
                                       std::vector<uint8_t> &response) {
  MachineInfo &info = MachineInfo::GetInstance();
  struct DeviceInfo data = {};
  size_t length = 17;
  memcpy(data.psn, info.GetPSN().c_str(), length);

  strncpy(data.model, "UNKNOWN", sizeof(data.model));
  length = sizeof(data.model);
  ESP_ERROR_COMPLAIN(DeviceNVSGet(data.model, &length, NVSKey::DEVICE_MODEL),
                     "get_device_model");

  memcpy(data.ap_version, info.GetESP32Version().data(), 3);
  memcpy(data.bp_version, info.GetMCUVersion().data(), 3);
  memcpy(data.fpga_version, info.GetFPGAVersion().data(), 3);
  memcpy(data.zrlib_version, info.GetZRLIBVersion().data(), 3);

  memcpy(data.ble_mac, info.GetBleMac().data(), sizeof(data.ble_mac));
  memcpy(data.wifi_mac, info.GetWifiMac().data(), sizeof(data.wifi_mac));

  ssid_detail_t ssid_detail;
  ESP_ERROR_COMPLAIN(get_connected_ssid_detail(&ssid_detail),
                     "get_connected_ssid_detail");
  memcpy(data.bssid, ssid_detail.bssid, sizeof(ssid_detail.bssid));
  memcpy(data.ssid, ssid_detail.ssid, sizeof(ssid_detail.ssid));
  data.channel = ssid_detail.channel;
  data.rssi = ssid_detail.rssi;
  data.wifi_protocol = ssid_detail.phy_11b | (ssid_detail.phy_11g << 1) |
                       (ssid_detail.phy_11n << 2) | (ssid_detail.phy_lr << 3) |
                       (ssid_detail.phy_11ax << 4) | (ssid_detail.wps << 5) |
                       (ssid_detail.ftm_responder << 6) |
                       (ssid_detail.ftm_initiator << 7);

  const uint8_t *pData = reinterpret_cast<const uint8_t *>(&data);
  size_t size = sizeof(DeviceInfo);
  response.insert(response.end(), pData, pData + size);
  return ESP_OK;
}

esp_err_t DeviceHandler::SetSyslogState(const std::vector<uint8_t> &request,
                                        std::vector<uint8_t> &response) {
  if (request.size() != 1) {
    ESP_LOGE(TAG, "Invalid SetSyslogState request length: %d", request.size());
    return ESP_FAIL;
  }
  return SyslogClient::GetInstance().SetReportEnabled(request[0]);
}

esp_err_t DeviceHandler::GetDevicePassword(const std::vector<uint8_t> &request,
                                           std::vector<uint8_t> &response) {
  uint8_t password[4];
  ESP_RETURN_ON_ERROR(DeviceAuth::getPassword(password, sizeof(password)), TAG,
                      "DeviceAuth::getPassword");
  for (size_t i = 0; i < sizeof(password); i++) {
    response.emplace_back(password[i]);
  }
  return ESP_OK;
}

esp_err_t DeviceHandler::ManagePowerAllocatorEnabled(
    const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  if (request.size() == 0) {
    ESP_LOGE(TAG, "Invalid ManagePowerAllocatorEnabled request length: %d",
             request.size());
    return ESP_FAIL;
  }
  uint8_t action = request[0];
  switch (action) {
    case ManageActionType::GET: {
      bool enabled;
      PowerNVSGetOrDefault(&enabled, NVSKey::POWER_ALLOCATOR_ENABLED);
      response.emplace_back(static_cast<uint8_t>(enabled));
      return ESP_OK;
      break;
    }
    case ManageActionType::SET: {
      if (request.size() != 2) {
        ESP_LOGE(TAG, "Invalid SetPowerAllocatorEnabled request length: %d",
                 request.size());
        return ESP_FAIL;
      }
      ESP_LOGI(TAG, "SetPowerAllocatorEnabled: %d", request[1]);
      return PowerNVSSet(request[1], NVSKey::POWER_ALLOCATOR_ENABLED);
      break;
    }
    case ManageActionType::RESET: {
      return PowerNVSEraseKey(NVSKey::POWER_ALLOCATOR_ENABLED);
    }
    default: {
      ESP_LOGE(TAG, "Invalid ManagePowerAllocatorEnabled action: %d", action);
      return ESP_FAIL;
    }
  }
}

esp_err_t DeviceHandler::ManagePowerConfig(const std::vector<uint8_t> &request,
                                           std::vector<uint8_t> &response) {
  if (request.size() < 2) {
    ESP_LOGE(TAG, "Invalid ManagePowerConfig request length: %d",
             request.size());
    return ESP_ERR_INVALID_SIZE;
  }
  uint8_t action = request[0];
  switch (action) {
    case ManageActionType::GET: {
      PowerConfig config;
      ESP_RETURN_ON_ERROR(get_power_config(&config), TAG, "get_power_config");
      response.insert(response.end(), reinterpret_cast<uint8_t *>(&config),
                      reinterpret_cast<uint8_t *>(&config) + sizeof(config));
      return ESP_OK;
      break;
    }
    case ManageActionType::SET: {
      if (request.size() != 1 + sizeof(PowerConfig)) {
        ESP_LOGE(TAG, "Invalid SetPowerConfig request length: %d",
                 request.size());
        return ESP_ERR_INVALID_SIZE;
      }
      ESP_LOGI(TAG, "SetPowerConfig");
      uint8_t version = request[1];
      if (version != kPowerConfigVersion) {
        ESP_LOGE(TAG, "Invalid PowerConfig version: %d", version);
        return ESP_ERR_INVALID_VERSION;
      }
      PowerConfig config;
      std::memcpy(&config, &request[1], sizeof(PowerConfig));
      return set_power_config(config);
      break;
    }
    case ManageActionType::RESET: {
      ESP_LOGI(TAG, "ResetPowerConfig");
      return reset_power_config();
    }
    default: {
      ESP_LOGE(TAG, "Invalid ManagePowerConfig action: %d", action);
      return ESP_FAIL;
    }
  }
  return ESP_OK;
}

esp_err_t DeviceHandler::SetSystemTime(const std::vector<uint8_t> &request,
                                       std::vector<uint8_t> &response) {
  if (request.size() != 4) {
    ESP_LOGE(TAG, "Invalid SetSystemTime request length: %d", request.size());
    return ESP_ERR_INVALID_SIZE;
  }
  uint32_t sec =
      (request[3] << 24) | (request[2] << 16) | (request[1] << 8) | request[0];
  struct timeval tv = {.tv_sec = sec, .tv_usec = 0};
  settimeofday(&tv, NULL);
  return ESP_OK;
}

esp_err_t DeviceHandler::GetDebugLog(const std::vector<uint8_t> &request,
                                     std::vector<uint8_t> &response) {
  size_t size = 1024;
  response.resize(size);
  LogCollector::GetInstance().Pop((char *)response.data(), &size);
  response.resize(size);
  return ESP_OK;
}

static esp_err_t manage_fpga_power_control(const std::vector<uint8_t> &request,
                                           std::vector<uint8_t> &response,
                                           uint8_t action, uint8_t enabled) {
  switch (action) {
    case ManageActionType::GET: {
      bool enabled = true;
      FPGANVSGetOrDefault(&enabled, NVSKey::FPGA_POWER_CONTROL);
      response.emplace_back(enabled);
      return ESP_OK;
    }
    case ManageActionType::SET: {
      FPGANVSSet(enabled, NVSKey::FPGA_POWER_CONTROL);
      return ESP_OK;
    }
    case ManageActionType::RESET: {
      FPGANVSEraseKey(NVSKey::FPGA_POWER_CONTROL);
      return ESP_OK;
    }
    default: {
      ESP_LOGE(TAG, "Invalid action: %d", action);
      return ESP_FAIL;
    }
  }
}

esp_err_t DeviceHandler::ManageFeatureToggle(
    const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  if (request.size() < 3) {
    ESP_LOGE(TAG, "Invalid ManageFeatureToggle request length: %d",
             request.size());
    return ESP_ERR_INVALID_SIZE;
  }
  uint8_t feature = request[0];
  uint8_t action = request[1];
  uint8_t enabled = request[2];
  response.emplace_back(feature);
  response.emplace_back(action);
  switch (feature) {
    case ManageFeature::FPGA_POWER_CONTROL: {
      return manage_fpga_power_control(request, response, action, enabled);
    }
  }
  return ESP_OK;
}

esp_err_t DeviceHandler::EnableReleaseMode(const std::vector<uint8_t> &request,
                                           std::vector<uint8_t> &response) {
  return esp_efuse_disable_rom_download_mode();
}

esp_err_t DeviceAuth::getPassword(uint8_t *password, size_t passwordSize) {
  if (password == nullptr || passwordSize != 4) {
    ESP_LOGW(TAG, "Invalid password buffer");
    return ESP_ERR_INVALID_ARG;
  }
  const std::array<uint8_t, 4> pin = MachineInfo::GetInstance().GetPin();
  for (size_t i = 0; i < pin.size(); i++) {
    password[i] = pin[i];
  }
  return ESP_OK;
}

bool DeviceAuth::validatePassword(const uint8_t *payload, size_t payloadSize) {
  uint8_t password[4];
  if (payload == nullptr || payloadSize != sizeof(password)) {
    ESP_LOGW(TAG, "Invalid payload, expected %d bytes, got %d bytes",
             sizeof(password), payloadSize);
    ESP_LOG_BUFFER_HEXDUMP(TAG, payload, payloadSize, ESP_LOG_WARN);
    return false;
  }
  ESP_RETURN_FALSE_ON_ERROR(getPassword(password, sizeof(password)),
                            "getPassword");

  for (size_t i = 0; i < payloadSize; i++) {
    if (payload[i] != password[i]) {
      return false;
    }
  }
  return true;
}
