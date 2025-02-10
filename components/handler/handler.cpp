#include "handler.h"

#include "app.h"
#include "ble_handler.h"
#include "device_handler.h"
#include "display_handler.h"
#include "esp_log.h"
#include "ota_handler.h"
#include "power_handler.h"
#include "sdkconfig.h"
#include "service.h"
#include "telemetry_stream_handler.h"
#include "wifi_handler.h"

static const char *TAG = "Handler";

// 在不同的 handler 文件内实现逻辑，然后在这里注册所有的服务
void Handler::RegisterAllServices(App &app) {
  // Device internal
  app.Srv(ServiceCommand::BLE_ECHO_TEST, BleHandler::BLEEchoTest,
          ServiceScope::SERVICE_SCOPE_BLE);
  app.Srv(ServiceCommand::GET_DEBUG_LOG, DeviceHandler::GetDebugLog,
          ServiceScope::SERVICE_SCOPE_BLE);
  app.Srv(ServiceCommand::GET_SECURE_BOOT_DIGEST,
          DeviceHandler::GetSecureBootDigest, ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::PING_MQTT_TELEMETRY, DeviceHandler::PingMQTTTelemetry,
          ServiceScope::SERVICE_SCOPE_BLE);
  app.Srv(ServiceCommand::PING_HTTP, DeviceHandler::PingHTTP,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_DEVICE_PASSWORD, DeviceHandler::GetDevicePassword,
          ServiceScope::SERVICE_SCOPE_MQTT);

  app.Srv(ServiceCommand::MANAGE_POWER_ALLOCATOR_ENABLED,
          DeviceHandler::ManagePowerAllocatorEnabled,
          ServiceScope::SERVICE_SCOPE_MQTT);
  app.Srv(ServiceCommand::MANAGE_POWER_CONFIG, DeviceHandler::ManagePowerConfig,
          ServiceScope::SERVICE_SCOPE_MQTT);
  app.Srv(ServiceCommand::MANAGE_FEATURE_TOGGLE,
          DeviceHandler::ManageFeatureToggle, ServiceScope::SERVICE_SCOPE_MQTT);
  app.Srv(ServiceCommand::ENABLE_RELEASE_MODE, DeviceHandler::EnableReleaseMode,
          ServiceScope::SERVICE_SCOPE_ALL);

  // Device
  app.Srv(ServiceCommand::GET_AP_VERSION, DeviceHandler::GetAPVersion,
          ServiceScope::SERVICE_SCOPE_ALL);
#ifdef CONFIG_MCU_MODEL_SW3566
  app.Srv(ServiceCommand::GET_BP_VERSION, DeviceHandler::GetBPVersion,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_FPGA_VERSION, DeviceHandler::GetFPGAVersion,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_ZRLIB_VERSION, DeviceHandler::GetZRLIBVersion,
          ServiceScope::SERVICE_SCOPE_ALL);
#endif
  app.Srv(ServiceCommand::REBOOT_DEVICE, DeviceHandler::RebootDevice,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::RESET_DEVICE, DeviceHandler::ResetDevice,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_DEVICE_SERIAL_NO,
          DeviceHandler::GetDeviceSerialNumber,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_DEVICE_UPTIME, DeviceHandler::GetDeviceUpTime,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::ASSOCIATE_DEVICE, DeviceHandler::AssociateDevice,
          ServiceScope::SERVICE_SCOPE_BLE);
  app.Srv(ServiceCommand::SWITCH_DEVICE, DeviceHandler::SwitchDevice,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_DEVICE_SWITCH, DeviceHandler::GetDeviceSwitch,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_DEVICE_MODEL, DeviceHandler::GetDeviceModel,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::PUSH_LICENSE, DeviceHandler::PushLicense,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_DEVICE_INFO, DeviceHandler::GetDeviceInfo,
          ServiceScope::SERVICE_SCOPE_MQTT);
  app.Srv(ServiceCommand::SET_SYSLOG_STATE, DeviceHandler::SetSyslogState,
          ServiceScope::SERVICE_SCOPE_MQTT);

  // OTA
  app.Srv(ServiceCommand::START_OTA, OTAHandler::StartOTA,
          ServiceScope::SERVICE_SCOPE_ALL);

  // Ble
  app.Srv(ServiceCommand::GET_DEVICE_BLE_ADDR, BleHandler::GetDeviceBleAddress,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::SET_BLE_STATE, BleHandler::SetBleState,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_BLE_RSSI, BleHandler::GetBleRSSI,
          ServiceScope::SERVICE_SCOPE_BLE);
  app.Srv(ServiceCommand::GET_BLE_MTU, BleHandler::GetBleMTU,
          ServiceScope::SERVICE_SCOPE_BLE);

  // Wi-Fi
  app.Srv(ServiceCommand::SCAN_WIFI, WiFiHandler::ScanWifi,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::SET_WIFI_SSID, WiFiHandler::SetWiFiSSID,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::SET_WIFI_PASSWORD, WiFiHandler::SetWiFiPassword,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::RESET_WIFI, WiFiHandler::ResetWiFi,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_WIFI_STATUS, WiFiHandler::GetWiFiStatus,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_DEVICE_WIFI_ADDR,
          WiFiHandler::GetDeviceWiFiAddress, ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::SET_WIFI_SSID_AND_PASSWORD,
          WiFiHandler::SetWiFiSSIDAndPassword, ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_WIFI_RECORDS, WiFiHandler::GetWiFiRecords,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::OPERATE_WIFI_RECORD, WiFiHandler::OperateWiFiRecord,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_WIFI_STATE_MACHINE,
          WiFiHandler::GetWiFiStateMachine, ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::SET_WIFI_STATE_MACHINE,
          WiFiHandler::SetWiFiStateMachine, ServiceScope::SERVICE_SCOPE_ALL);

  // Power
  app.Srv(ServiceCommand::TOGGLE_PORT_POWER, PowerHandler::TogglePortPower,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_POWER_STATISTICS, PowerHandler::GetPowerStats,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_POWER_HISTORICAL_STATS,
          PowerHandler::GetPowerHistoricalStats,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_POWER_SUPPLY_STATUS,
          PowerHandler::GetPowerSupplyStatus, ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_CHARGING_STATUS, PowerHandler::GetChargingStatus,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::SET_CHARGING_STRATEGY,
          PowerHandler::SetChargingStrategy, ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::SET_PORT_PRIORITY, PowerHandler::SetPortPriority,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_PORT_PRIORITY, PowerHandler::GetPortPriority,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_CHARGING_STRATEGY,
          PowerHandler::GetChargingStrategy, ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_PORT_PD_STATUS, PowerHandler::GetPortPDStatus,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_ALL_POWER_STATISTICS,
          PowerHandler::GetAllPowerStats, ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_START_CHARGE_TIMESTAMP,
          PowerHandler::GetStartChargeTimestamp,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::TURN_ON_PORT, PowerHandler::TurnOnPort,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::TURN_OFF_PORT, PowerHandler::TurnOffPort,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::SET_STATIC_ALLOCATOR,
          PowerHandler::SetStaticAllocator, ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_STATIC_ALLOCATOR,
          PowerHandler::GetStaticAllocator, ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::SET_PORT_CONFIG, PowerHandler::SetPortConfig,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_PORT_CONFIG, PowerHandler::GetPortConfig,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::SET_PORT_COMPATIBILITY_SETTINGS,
          PowerHandler::SetPortCompatibilitySettings,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_PORT_COMPATIBILITY_SETTINGS,
          PowerHandler::GetPortCompatibilitySettings,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::SET_TEMPERATURE_MODE,
          PowerHandler::SetTemperatureMode, ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::SET_TEMPORARY_ALLOCATOR,
          PowerHandler::SetTemporaryAllocator, ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::SET_PORT_CONFIG1, PowerHandler::SetPortConfig,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_PORT_CONFIG1, PowerHandler::GetPortConfig,
          ServiceScope::SERVICE_SCOPE_ALL);

  // Display
  app.Srv(ServiceCommand::SET_DISPLAY_INTENSITY,
          DisplayHandler::SetDisplayIntensity, ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_DISPLAY_INTENSITY,
          DisplayHandler::GetDisplayIntensity, ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::SET_DISPLAY_MODE, DisplayHandler::SetDisplayMode,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_DISPLAY_MODE, DisplayHandler::GetDisplayMode,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::SET_DISPLAY_FLIP, DisplayHandler::SetDisplayFlip,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_DISPLAY_FLIP, DisplayHandler::GetDisplayFlip,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::SET_DISPLAY_CONFIG, DisplayHandler::SetDisplayConfig,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::SET_DISPLAY_STATE, DisplayHandler::SetDisplayState,
          ServiceScope::SERVICE_SCOPE_ALL);
  app.Srv(ServiceCommand::GET_DISPLAY_STATE, DisplayHandler::GetDisplayState,
          ServiceScope::SERVICE_SCOPE_ALL);

  app.Srv(ServiceCommand::START_TELEMETRY_STREAM,
          TelemetryStreamHandler::StartTelemetryStream,
          ServiceScope::SERVICE_SCOPE_MQTT);
  app.Srv(ServiceCommand::STOP_TELEMETRY_STREAM,
          TelemetryStreamHandler::StopTelemetryStream,
          ServiceScope::SERVICE_SCOPE_MQTT);

  app.Srv(ServiceCommand::SET_SYSTEM_TIME, DeviceHandler::SetSystemTime,
          ServiceScope::SERVICE_SCOPE_MQTT);

  ESP_LOGI(TAG, "All services registered");
}
