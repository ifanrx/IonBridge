#include "device_handler.h"

#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "admin.pb.h"
#include "command.pb.h"
#include "config_sync_manager.h"
#include "controller.h"
#include "data_types.pb.h"
#include "device_data.h"
#include "display_manager.h"
#include "display_test.h"
#include "esp_check.h"
#include "esp_crc.h"
#include "esp_efuse.h"
#include "esp_err.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "ionbridge.h"
#include "license.h"
#include "logging.h"
#include "machine_info.h"
#include "mqtt_app.h"
#include "nvs_key.h"
#include "nvs_namespace.h"
#include "nvs_partition.h"
#include "port_manager.h"
#include "port_state.h"
#include "power_allocator.h"
#include "power_config.h"
#include "sdkconfig.h"
#include "syslog.h"
#include "telemetry_task.h"
#include "utils.h"
#include "version.h"
#include "wifi.h"
#include "wifi_manager.h"

#ifdef CONFIG_ENABLE_PROGRAMMER
#include "firmware_programmer.h"
#endif

#ifdef CONFIG_INTERFACE_TYPE_UART
#include "fpga_data_types.h"
#endif

#ifdef CONFIG_ACDC_MODULE
#include "acdc.h"
#endif

static const char *TAG = "DeviceHandler";

using namespace DeviceHandler;
using namespace nvs_wrapper;
using namespace nvs_namespace::user_data;
using namespace nvs_namespace;

#define FEATURE_COUNT 256
#define PROGRAMMER_PORT_ID 4

namespace {

typedef union {
  uint32_t word;
  struct {
    bool wifi : 1;          // Keep WiFi NVS namespace
    bool mqtt : 1;          // Keep MQTT NVS namespace
    bool ble : 1;           // Keep BLE NVS namespace
    bool display : 1;       // Keep Display NVS namespace
    bool power : 1;         // Keep Power NVS namespace
    bool system : 1;        // Keep System NVS namespace
    bool license : 1;       // Keep License NVS namespace
    bool syslog : 1;        // Keep Syslog NVS namespace
    bool fpga : 1;          // Keep FPGA NVS namespace
    bool ota : 1;           // Keep OTA NVS namespace
    uint8_t reserved0 : 6;  // Reserved bits
    uint16_t reserved1;
  } bits;
} KeepNVSNamespaces;

typedef union {
  uint32_t word;
  struct {
    bool prevent_reboot : 1;            // Prevent reboot after reset
    bool reset_ports : 1;               // Toggle ports
    bool reset_power_strategy : 1;      // Reset power allocator strategy only
    bool reset_wifi : 1;                // Reset WiFi settings
    bool reset_display : 1;             // Reset Display settings
    bool reset_fc_protocol : 1;         // Reset fc protocol
    bool reset_temperature_mode : 1;    // Reset temperature mode
    bool reset_cable_compensation : 1;  // Reset cable compensation
    uint8_t reserved1[3];
  } bits;
} ResetOperations;

typedef union {
  uint32_t words[2];
  struct {
    KeepNVSNamespaces nvs;  // NVS namespaces to keep
    ResetOperations op;     // Reset operations
  } fields;
} ResetDeviceArgs;

esp_err_t DoReset(const ResetDeviceArgs &args);

esp_err_t PingHTTPEventHandler(esp_http_client_event_t *evt);

esp_err_t SetPowerStateCommon(bool power_on);

}  // Anonymous namespace

esp_err_t FromBLE::GetAPVersion(const std::vector<uint8_t> &request,
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

esp_err_t FromBLE::AssociateDevice(const std::vector<uint8_t> &request,
                                   std::vector<uint8_t> &response) {
  ESP_RETURN_ON_FALSE(request.size() >= 4, ESP_ERR_INVALID_ARG, TAG,
                      "Invalid request length: %d", request.size());

  size_t password_len = 0;
  bool reset_data = true;
  password_len = request.size();
  if (request.size() > 4) {
    password_len = request.size() - 1;
    reset_data = !request[request.size() - 1];
  }

  const auto pin = MachineInfo::GetInstance().GetPin();
  if (password_len != 4 ||
      !std::equal(pin.begin(), pin.end(), request.data())) {
    ESP_LOGW(TAG, "Invalid password provided");
    return ESP_ERR_INVALID_ARG;
  }

  if (reset_data) {
    ESP_LOGI(TAG, "Resetting user data");
    ESP_RETURN_ON_ERROR(partition::ReinitalizeUserData(), TAG,
                        "reset_user_data");
    WiFiManager::GetInstance().Reset();
    wifi_controller.Abort();
    PowerAllocator &allocator = PowerAllocator::GetInstance();
    if (allocator.Type() == PowerAllocatorType::TEMPORARY_ALLOCATOR) {
      allocator.SetDefaultStrategy();
    }
  }

  uint8_t token;
  ESP_RETURN_ON_ERROR(nvs_utils::GetBLEAuthToken(&token, reset_data), TAG,
                      "Failed to get token");

  DeviceController::GetInstance().mark_associated();
  response.emplace_back(token);
  return ESP_OK;
}

esp_err_t FromBLE::ResetDevice(const std::vector<uint8_t> &request,
                               std::vector<uint8_t> &response) {
  ESP_LOGW(TAG, "Client request to reset device via BLE");
  ResetDeviceArgs args = {};
  switch (request.size()) {
    case 0: {
      // No arguments, reset all
      memset(&args, 0, sizeof(args));
    } break;
    case sizeof(ResetDeviceArgs): {
      memcpy(&args, request.data(), sizeof(ResetDeviceArgs));
    } break;
    default: {
      ESP_LOGW(TAG, "Invalid reset device request length: %d", request.size());
      return ESP_ERR_INVALID_ARG;
    }
  }
  return DoReset(args);
}

esp_err_t FromBLE::RebootDevice(const std::vector<uint8_t> &request,
                                std::vector<uint8_t> &response) {
  ESP_LOGW(TAG, "Client request to reboot device");
  DeviceController::GetInstance().reboot_after(1000);
  return ESP_OK;
}

esp_err_t FromBLE::GetDeviceSerialNumber(const std::vector<uint8_t> &request,
                                         std::vector<uint8_t> &response) {
  const std::string &serial_number = MachineInfo::GetInstance().GetPSN();
  ESP_RETURN_ON_FALSE(!serial_number.empty(), ESP_FAIL, TAG,
                      "Serial number is empty");
  response.insert(response.end(), serial_number.begin(), serial_number.end());
  return ESP_OK;
}

esp_err_t FromBLE::SetPowerState(const std::vector<uint8_t> &request,
                                 std::vector<uint8_t> &response) {
  if (request.size() != 1) {
    ESP_LOGW(TAG, "Invalid device switch length: %d", request.size());
    return ESP_FAIL;
  }
  uint8_t device_switch = request[0];
  if (device_switch > 1) {
    ESP_LOGW(TAG, "Invalid device power state: %d", device_switch);
    return ESP_FAIL;
  }
  return SetPowerStateCommon(device_switch == 1);
}

esp_err_t FromBLE::GetPowerState(const std::vector<uint8_t> &request,
                                 std::vector<uint8_t> &response) {
  bool opened = DeviceController::GetInstance().is_power_on();
  response.emplace_back(static_cast<uint8_t>(opened));
  return ESP_OK;
}

esp_err_t FromBLE::GetSecureBootDigest(const std::vector<uint8_t> &request,
                                       std::vector<uint8_t> &response) {
  uint8_t key[32];
  ESP_RETURN_ON_ERROR(
      esp_efuse_read_block(EFUSE_BLK_KEY1, key, 0, sizeof(key) * 8), TAG,
      "esp_efuse_read_block");
  for (size_t i = 0; i < sizeof(key); i++) {
    response.emplace_back(key[i]);
  }
  return ESP_OK;
}

esp_err_t FromBLE::PingMQTTTelemetry(const std::vector<uint8_t> &request,
                                     std::vector<uint8_t> &response) {
  return TelemetryTask::GetInstance().ReportPingInfo();
}

esp_err_t FromBLE::PingHTTP(const std::vector<uint8_t> &request,
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
  config.event_handler = PingHTTPEventHandler;
  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == NULL) {
    ESP_LOGE(TAG, "Failed to initialize HTTP connection");
    return ESP_FAIL;
  }

  // GET
  return esp_http_client_perform(client);
}

esp_err_t FromBLE::SetDisplayTest(const std::vector<uint8_t> &request,
                                  std::vector<uint8_t> &response) {
  return ESP_OK;
}

esp_err_t FromBLE::SwitchFPGAFirmware(const std::vector<uint8_t> &request,
                                      std::vector<uint8_t> &response) {
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t FromBLE::GetDebugLog(const std::vector<uint8_t> &request,
                               std::vector<uint8_t> &response) {
  size_t size = 1024;
  response.resize(size);
  LogCollector::GetInstance().Pop((char *)response.data(), &size);
  response.resize(size);
  return ESP_OK;
}

esp_err_t FromBLE::EnableReleaseMode(const std::vector<uint8_t> &request,
                                     std::vector<uint8_t> &response) {
  return ESP_OK;
}

esp_err_t FromBLE::GetHardwareRevision(const std::vector<uint8_t> &request,
                                       std::vector<uint8_t> &response) {
  std::string revision = MachineInfo::GetInstance().GetHwRev();
  response.insert(response.end(), revision.begin(), revision.end());
  return ESP_OK;
}

esp_err_t FromBLE::GetDeviceUpTime(const std::vector<uint8_t> &request,
                                   std::vector<uint8_t> &response) {
  int64_t uptime = esp_timer_get_time();
  EMPLACE_BACK_INT64(response, uptime);
  return ESP_OK;
}

esp_err_t FromBLE::GetACDCSelfTestResult(const std::vector<uint8_t> &request,
                                         std::vector<uint8_t> &response) {
  return ESP_OK;
}

esp_err_t FromBLE::RunACDCSelfTest(const std::vector<uint8_t> &request,
                                   std::vector<uint8_t> &response) {
  return ESP_OK;
}

esp_err_t FromMQTT::GetDeviceInfo(const pb_CommandRequest &request,
                                  pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_get_device_info_tag;
  MachineInfo &info = MachineInfo::GetInstance();
  pb_DeviceInfo &data = response.payload.get_device_info.info;
  strlcpy(data.psn, info.GetPSN().c_str(), sizeof(data.psn));
  strlcpy(data.model, info.GetDeviceModel().c_str(), sizeof(data.model));

  memcpy(&data.ap_version, info.GetESP32Version().data(),
         sizeof(data.ap_version));
  memcpy(&data.bp_version, info.GetMCUVersion().data(),
         sizeof(data.bp_version));
  memcpy(&data.fpga_version, info.GetFPGAVersion().data(),
         sizeof(data.fpga_version));
  memcpy(&data.zrlib_version, info.GetZRLIBVersion().data(),
         sizeof(data.zrlib_version));

  std::array<uint8_t, 6> ble_mac = info.GetBleMac();
  hexlify(ble_mac.data(), ble_mac.size(), data.ble_mac, sizeof(data.ble_mac));
  std::array<uint8_t, 6> wifi_mac = info.GetWifiMac();
  hexlify(wifi_mac.data(), wifi_mac.size(), data.wifi_mac,
          sizeof(data.wifi_mac));

  ssid_detail_t ssid_detail;
  ESP_ERROR_COMPLAIN(get_connected_ssid_detail(&ssid_detail),
                     "get_connected_ssid_detail");
  hexlify(ssid_detail.bssid, sizeof(ssid_detail.bssid), data.bssid,
          sizeof(data.bssid));
  std::memcpy(data.ssid, ssid_detail.ssid, sizeof(ssid_detail.ssid));
  data.channel = ssid_detail.channel;
  data.rssi = ssid_detail.rssi;
  data.wifi_protocol = ssid_detail.phy_11b | (ssid_detail.phy_11g << 1) |
                       (ssid_detail.phy_11n << 2) | (ssid_detail.phy_lr << 3) |
                       (ssid_detail.phy_11ax << 4) | (ssid_detail.wps << 5) |
                       (ssid_detail.ftm_responder << 6) |
                       (ssid_detail.ftm_initiator << 7);
  return ESP_OK;
}

esp_err_t FromMQTT::SetSyslogState(const pb_AdminCommandRequest &request,
                                   pb_AdminCommandResponse &response) {
  response.which_payload = pb_AdminCommandResponse_set_syslog_state_tag;
  return SyslogClient::GetInstance().EnableRemoteLogging(
      request.payload.set_syslog_state.enable);
}

esp_err_t FromMQTT::SetSystemTime(const pb_AdminCommandRequest &request,
                                  pb_AdminCommandResponse &response) {
  response.which_payload = pb_AdminCommandResponse_set_system_time_tag;
  ESP_RETURN_ON_ERROR(MachineInfo::GetInstance().SetSystemTime(
                          request.payload.set_system_time.timestamp),
                      TAG, "Failed to set system time");
  return ESP_OK;
}

esp_err_t FromMQTT::ManageFPGAConfig(const pb_AdminCommandRequest &request,
                                     pb_AdminCommandResponse &response) {
  response.which_payload = pb_AdminCommandResponse_manage_fpga_config_tag;
#ifdef CONFIG_INTERFACE_TYPE_UART
  pb_ManageAction action = request.payload.manage_fpga_config.action;
  uint8_t level = request.payload.manage_fpga_config.level;
  auto cfg = FPGAADCConfig::Create(level);
  ESP_RETURN_ON_FALSE(cfg != nullptr, ESP_ERR_INVALID_ARG, TAG,
                      "Failed to create FPGAADCConfig for level: %d", level);

  esp_err_t err;
  switch (action) {
    case pb_ManageAction_READ: {
      ESP_RETURN_ON_ERROR(cfg->Load(), TAG, "Failed to load FPGA ADC config");
      response.payload.manage_fpga_config.config.adc_threshold_low =
          cfg->GetThresholdLow();
      response.payload.manage_fpga_config.config.adc_threshold_high =
          cfg->GetThresholdHigh();
      response.payload.manage_fpga_config.config.action_deadzone =
          cfg->GetActionDeadzone();
      ESP_LOGD(TAG,
               "GetFPGAConfig: adc_threshold_low=%d adc_threshold_high=%d "
               "action_deadzone=%d",
               response.payload.manage_fpga_config.config.adc_threshold_low,
               response.payload.manage_fpga_config.config.adc_threshold_high,
               response.payload.manage_fpga_config.config.action_deadzone);
      err = ESP_OK;
    } break;
    case pb_ManageAction_UPDATE: {
      cfg->Update(request.payload.manage_fpga_config.config.adc_threshold_low,
                  request.payload.manage_fpga_config.config.adc_threshold_high,
                  request.payload.manage_fpga_config.config.action_deadzone);
      ESP_LOGI(TAG,
               "SetFPGAConfig: adc_threshold_low=%d adc_threshold_high=%d "
               "action_deadzone=%d",
               cfg->GetThresholdLow(), cfg->GetThresholdHigh(),
               cfg->GetActionDeadzone());
      err = cfg->Save();
    } break;
    case pb_ManageAction_DELETE: {
      err = cfg->Reset();
    } break;
    default: {
      ESP_LOGE(TAG, "Invalid ManageFPGAConfig action: %d", action);
      err = ESP_ERR_INVALID_ARG;
    } break;
  }

  return err;
#else
  return ESP_ERR_NOT_SUPPORTED;
#endif  // CONFIG_INTERFACE_TYPE_UART
}

esp_err_t FromMQTT::ManagePowerConfig(const pb_AdminCommandRequest &request,
                                      pb_AdminCommandResponse &response) {
  response.which_payload = pb_AdminCommandResponse_manage_power_config_tag;
  pb_ManageAction action = request.payload.manage_power_config.action;
  switch (action) {
    case pb_ManageAction_READ: {
      PowerConfig config;
      ESP_RETURN_ON_ERROR(get_power_config(&config), TAG, "get_power_config");
      std::memcpy(&response.payload.manage_power_config.config, &config,
                  sizeof(PowerConfig));

      return ESP_OK;
      break;
    }
    case pb_ManageAction_UPDATE: {
      ESP_LOGI(TAG, "SetPowerConfig");
      uint8_t version = request.payload.manage_power_config.config.version;
      if (version != kPowerConfigVersion) {
        ESP_LOGE(TAG, "Invalid PowerConfig version: %d", version);
        return ESP_ERR_INVALID_VERSION;
      }
      PowerConfig config;
      std::memcpy(&config, &request.payload.manage_power_config.config,
                  sizeof(PowerConfig));
      return set_power_config(config);
    }
    case pb_ManageAction_DELETE: {
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

static esp_err_t manage_fpga_power_control(
    const pb_AdminCommandRequest &request, pb_AdminCommandResponse &response) {
  const pb_ManageAction &action = request.payload.manage_feature_toggle.action;
  bool enabled = request.payload.manage_feature_toggle.enabled;
  switch (action) {
    case pb_ManageAction_READ: {
      ESP_RETURN_ON_ERROR(
          FPGA.GetOrDefault(nvs_keys::FPGA_POWER_CONTROL, enabled), TAG,
          "Failed to get FPGA power control state");
      response.payload.manage_feature_toggle.enabled = enabled;
      return ESP_OK;
    }
    case pb_ManageAction_UPDATE: {
      return FPGA.Set(nvs_keys::FPGA_POWER_CONTROL, enabled);
    }
    case pb_ManageAction_DELETE: {
      return FPGA.EraseKey(nvs_keys::FPGA_POWER_CONTROL);
    }
    default: {
      ESP_LOGE(TAG, "Invalid action: %d", action);
      return ESP_FAIL;
    }
  }
}

static esp_err_t manage_power_allocator(const pb_AdminCommandRequest &request,
                                        pb_AdminCommandResponse &response) {
  const pb_ManageAction &action = request.payload.manage_feature_toggle.action;
  bool enabled = request.payload.manage_feature_toggle.enabled;
  switch (action) {
    case pb_ManageAction_READ: {
      bool enabled;
      ESP_RETURN_ON_ERROR(
          POWER.GetOrDefault(nvs_keys::POWER_ALLOCATOR_ENABLED, enabled), TAG,
          "Failed to get Power Allocator enabled state");
      response.payload.manage_feature_toggle.enabled = enabled;
      return ESP_OK;
    }
    case pb_ManageAction_UPDATE: {
      ESP_LOGI(TAG, "SetPowerAllocatorEnabled: %d", enabled);
      return POWER.Set(nvs_keys::POWER_ALLOCATOR_ENABLED, enabled);
    }
    case pb_ManageAction_DELETE: {
      return POWER.EraseKey(nvs_keys::POWER_ALLOCATOR_ENABLED);
    }
    default: {
      ESP_LOGE(TAG, "Invalid ManagePowerAllocatorEnabled action: %d", action);
      return ESP_FAIL;
    }
  }
}

esp_err_t FromMQTT::ManageFeatureToggle(const pb_AdminCommandRequest &request,
                                        pb_AdminCommandResponse &response) {
  response.which_payload = pb_AdminCommandResponse_manage_feature_toggle_tag;

  pb_ManageFeature feature = request.payload.manage_feature_toggle.feature;
  response.payload.manage_feature_toggle.feature =
      request.payload.manage_feature_toggle.feature;
  response.payload.manage_feature_toggle.action =
      request.payload.manage_feature_toggle.action;
  switch (feature) {
    case pb_ManageFeature_FPGA_POWER_CONTROL: {
      return manage_fpga_power_control(request, response);
    }
    case pb_ManageFeature_POWER_ALLOCATOR: {
      return manage_power_allocator(request, response);
    }
  }
  return ESP_OK;
}

static esp_err_t ValidateDeviceConfigValue(pb_DeviceConfigEntity entity,
                                           const std::string &value) {
  if (value.empty()) {
    ESP_LOGE(TAG, "Empty value not allowed for device config");
    return ESP_ERR_INVALID_ARG;
  }

  switch (entity) {
    case pb_DeviceConfigEntity_ENTITY_HARDWARE_REVISION:
      if (value.length() > 8) {
        ESP_LOGE(TAG, "Hardware revision too long: max 8 chars");
        return ESP_ERR_INVALID_ARG;
      }
      return ESP_OK;

    case pb_DeviceConfigEntity_ENTITY_PRODUCT_FAMILY:
      if (value != "CP02" && value != "PA768s" && value != "CP02s") {
        ESP_LOGE(TAG,
                 "Invalid product family: %s (expected: CP02, PA768s, CP02s)",
                 value.c_str());
        return ESP_ERR_INVALID_ARG;
      }
      return ESP_OK;

    case pb_DeviceConfigEntity_ENTITY_DEVICE_MODEL: {
      DeviceData::DeviceModel::Model model;
      return DeviceData::DeviceModel::FromString(value, model);
    }

    case pb_DeviceConfigEntity_ENTITY_DEVICE_COLOR:
      if (value != "white" && value != "gray" && value != "black") {
        ESP_LOGE(TAG, "Invalid color: %s (expected: white, gray, black)",
                 value.c_str());
        return ESP_ERR_INVALID_ARG;
      }
      return ESP_OK;

    case pb_DeviceConfigEntity_ENTITY_DEVICE_NAME:
      if (value.length() > 15) {
        ESP_LOGE(TAG, "Device name too long: max 15 chars");
        return ESP_ERR_INVALID_ARG;
      }
      return ESP_OK;

    default:
      ESP_LOGE(TAG, "Invalid entity type: %d", entity);
      return ESP_ERR_INVALID_ARG;
  }
}

esp_err_t FromMQTT::EnableReleaseMode(const pb_CommandRequest &request,
                                      pb_CommandResponse &response) {
  return esp_efuse_disable_rom_download_mode();
}

esp_err_t FromMQTT::GetHardwareRevision(const pb_CommandRequest &request,
                                        pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_get_hardware_revision_tag;
  std::string revision = MachineInfo::GetInstance().GetHwRev();
  strlcpy(response.payload.get_hardware_revision.revision, revision.c_str(),
          sizeof(response.payload.get_hardware_revision.revision));
  return ESP_OK;
}

esp_err_t FromMQTT::GetDevicePassword(const pb_AdminCommandRequest &request,
                                      pb_AdminCommandResponse &response) {
  response.which_payload = pb_AdminCommandResponse_get_device_password_tag;
  std::array<uint8_t, 4> pin = MachineInfo::GetInstance().GetPin();
  memcpy(response.payload.get_device_password.password, pin.data(), pin.size());
  return ESP_OK;
}

esp_err_t FromMQTT::ResetDevice(const pb_CommandRequest &request,
                                pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_reset_device_tag;
  ESP_LOGW(TAG, "Client request to reset device via MQTT");
  ResetDeviceArgs args = {};
  args.fields.nvs.bits = {
      .wifi = request.payload.reset_device.keep_wifi_nvs,
      .mqtt = request.payload.reset_device.keep_mqtt_nvs,
      .ble = request.payload.reset_device.keep_ble_nvs,
      .display = request.payload.reset_device.keep_display_nvs,
      .power = request.payload.reset_device.keep_power_nvs,
      .system = request.payload.reset_device.keep_system_nvs,
      .license = request.payload.reset_device.keep_license_nvs,
      .syslog = request.payload.reset_device.keep_syslog_nvs,
      .fpga = request.payload.reset_device.keep_fpga_nvs,
      .ota = request.payload.reset_device.keep_ota_nvs,
      .reserved0 = 0,
      .reserved1 = 0};
  args.fields.op.bits = {
      .prevent_reboot = request.payload.reset_device.prevent_reboot,
      .reset_ports = request.payload.reset_device.reset_ports,
      .reset_power_strategy = request.payload.reset_device.reset_power_strategy,
      .reset_wifi = request.payload.reset_device.reset_wifi,
      .reset_display = request.payload.reset_device.reset_display,
      .reset_fc_protocol = request.payload.reset_device.reset_fc_protocol,
      .reset_temperature_mode =
          request.payload.reset_device.reset_temperature_mode,
      .reset_cable_compensation =
          request.payload.reset_device.reset_cable_compensation,
      .reserved1 = {0, 0, 0}};
  return DoReset(args);
}

esp_err_t FromMQTT::RebootDevice(const pb_CommandRequest &request,
                                 pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_reboot_device_tag;
  ESP_LOGW(TAG, "Client request to reboot device");
  DeviceController::GetInstance().reboot_after(1000);
  return ESP_OK;
}

esp_err_t FromMQTT::GetAPVersion(const pb_CommandRequest &request,
                                 pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_get_ap_version_tag;
  std::array<uint8_t, 3> esp32_version =
      MachineInfo::GetInstance().GetESP32Version();

  memcpy(&response.payload.get_ap_version.version, esp32_version.data(),
         sizeof(response.payload.get_ap_version.version));

  ESP_LOGI(TAG, "GetAPVersion: %s",
           MachineInfo::GetInstance().FormatVersion(esp32_version).c_str());
  return ESP_OK;
}

esp_err_t FromMQTT::GetDeviceSerialNumber(const pb_CommandRequest &request,
                                          pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_get_device_psn_tag;
  const std::string &serial_number = MachineInfo::GetInstance().GetPSN();
  ESP_RETURN_ON_FALSE(!serial_number.empty(), ESP_FAIL, TAG,
                      "Serial number is empty");
  strlcpy(response.payload.get_device_psn.psn, serial_number.c_str(),
          sizeof(response.payload.get_device_psn.psn));
  return ESP_OK;
}

esp_err_t FromMQTT::SetPowerState(const pb_CommandRequest &request,
                                  pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_set_power_state_tag;
  pb_PowerState state = request.payload.set_power_state.state;
  if (state != pb_PowerState_POWER_ON && state != pb_PowerState_POWER_OFF) {
    ESP_LOGW(TAG, "Invalid device power state: %d", state);
    return ESP_FAIL;
  }
  return SetPowerStateCommon(state == pb_PowerState_POWER_ON);
}

esp_err_t FromMQTT::GetPowerState(const pb_CommandRequest &request,
                                  pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_get_power_state_tag;
  bool opened = DeviceController::GetInstance().is_power_on();
  response.payload.get_power_state.state =
      opened ? pb_PowerState_POWER_ON : pb_PowerState_POWER_OFF;
  return ESP_OK;
}

esp_err_t FromMQTT::PushLicense(const pb_AdminCommandRequest &request,
                                pb_AdminCommandResponse &response) {
  response.which_payload = pb_AdminCommandRequest_set_license_tag;
  if (request.payload.set_license.data_count == 0) {
    ESP_LOGW(TAG, "No license request payload found");
    return ESP_FAIL;
  }

  if (MQTTClient::GetInstance()->IsCustomBroker()) {
    ESP_LOGI(TAG, "Using MQTT custom broker, ignore license push");
    return ESP_OK;
  }

  return LicenseController::GetInstance().ProcessLicenses(
      request.payload.set_license.data, request.payload.set_license.data_count);
}

namespace {

esp_err_t DoReset(const ResetDeviceArgs &args) {
  // 1. Erase NVS namespaces
  if (args.fields.nvs.word == 0) {
    ESP_LOGW(TAG, "Resetting all NVS namespaces");
    ESP_RETURN_ON_ERROR(partition::ReinitalizeUserData(), TAG,
                        "Failed to erase user data partition");
  } else {
    const std::pair<Namespace &, const char *> namespaces[] = {
        {WIFI, "WiFi"},             // bit 0
        {MQTT_CUSTOM, "MQTT"},      // bit 1
        {BLE, "BLE"},               // bit 2
        {DISPLAY_DATA, "Display"},  // bit 3
        {POWER, "Power"},           // bit 4
        {SYSTEM, "System"},         // bit 5
        {LICENSE, "License"},       // bit 6
        {SYSLOG, "Syslog"},         // bit 7
        {FPGA, "FPGA"},             // bit 8
        {OTA, "OTA"}                // bit 9
    };
    uint32_t keep_mask = args.fields.nvs.word;
    ESP_LOGD(TAG, "Resetting NVS namespaces: 0x%08" PRIx32, keep_mask);
    for (const auto &ns : namespaces) {
      if (!(keep_mask & 1)) {  // Check LSB
        ESP_LOGW(TAG, "Resetting NVS namespace: %s", ns.second);
        ESP_RETURN_ON_ERROR(ns.first.Reset(), TAG,
                            "Failed to erase %s NVS namespace", ns.second);
      }
      keep_mask >>= 1;  // Shift to check next bit
    }
  }

  // 2. Perform reset operations
  if (!args.fields.op.bits.prevent_reboot) {
    DeviceController::GetInstance().reboot_after(1000);
    return ESP_OK;
  }
  if (args.fields.op.bits.reset_ports) {
    // Reset ports completely (hardware reset)
    PortManager &pm = PortManager::GetInstance();
    pm.ForEach([](Port &port) { port.ResetState(); });
  }
  if (args.fields.op.bits.reset_fc_protocol) {
    // Reset fc protocol (original reset_ports behavior)
    PortManager &pm = PortManager::GetInstance();
    pm.ForEach([](Port &port) {
      port.ResetConfig();
      port.ApplyConfig();
    });
  }
  if (args.fields.op.bits.reset_power_strategy) {
    // Reset power strategy only

    // if ports are also being reset, wait for pd handshake to settle
    if (args.fields.op.bits.reset_ports) {
      DELAY_MS(300);
    }
    PowerAllocator &allocator = PowerAllocator::GetInstance();
    allocator.SetDefaultStrategy();
  }
  if (args.fields.op.bits.reset_temperature_mode) {
    // Reset temperature mode only
    PowerAllocator &allocator = PowerAllocator::GetInstance();
    allocator.SetDefaultTemperatureMode();
  }
  if (args.fields.op.bits.reset_wifi) {
    // Reset WiFi settings
    WiFiManager::GetInstance().Reset();
    wifi_controller.Abort();
  }
  if (args.fields.op.bits.reset_display) {
    // Reset Display settings
    DisplayManager::GetInstance().Reset();
  }
  if (args.fields.op.bits.reset_cable_compensation) {
    // Reset cable compensation to default
#ifdef CONFIG_MCU_MODEL_SW3566
    PortManager &pm = PortManager::GetInstance();
    pm.ForEach([](Port &port) {
      port.SetCableCompensation(true, CableCompensation::VALUE_65MOHM,
                                FixedVoltageOffset::OFFSET_0MV, false);
    });
#endif
  }

  // Notify config sync if not rebooting (since reboot will reload configs
  // anyway)
  ConfigSyncManager::GetInstance().NotifyConfigChange();
  return ESP_OK;
}

esp_err_t PingHTTPEventHandler(esp_http_client_event_t *evt) {
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

esp_err_t SetPowerStateCommon(bool power_on) {
  DeviceController &controller = DeviceController::GetInstance();
  ESP_LOGI(TAG, "Switching device %s", power_on ? "on" : "off");

  if (power_on) {
    esp_err_t result = controller.power_on();
    if (result == ESP_OK) {
      // Critical: Only sync power_on state to cloud, never power_off.
      // Reason: If we sync power_off state, the following scenario breaks:
      // 1. User sends power_off command -> state synced to cloud
      // 2. User unplugs and replugs device -> ESP32 boots with ports ON
      // 3. Cloud sync applies saved power_off -> ports turn OFF
      // Result: Device is powered but ports are disabled (broken UX)
      // Power state must be transient - physical power determines truth
      ConfigSyncManager::GetInstance().NotifyConfigChange();
    }
    return result;
  }

  return controller.power_off();  // Intentionally no sync
}

}  // Anonymous namespace
