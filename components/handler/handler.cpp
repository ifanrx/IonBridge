#include "handler.h"

#include "app.h"
#include "ble_handler.h"
#include "config_sync_handler.h"
#include "device_handler.h"
#include "display_handler.h"
#include "esp_log.h"
#include "ota_handler.h"
#include "power_handler.h"
#include "sdkconfig.h"
#include "service.h"
#include "task_handler.h"
#include "telemetry_stream_handler.h"
#include "wifi_handler.h"

static const char *TAG = "Handler";

#define REGISTER_BLE_SERVICE(cmd, ns, handler) \
  app.SrvBLE(ServiceCommand::cmd, ns::FromBLE::handler)
#define REGISTER_MQTT_SERVICE(cmd, ns, handler) \
  app.SrvMQTT(ServiceCommand::cmd, ns::FromMQTT::handler)
#define REGISTER_MQTT_ADMIN_SERVICE(cmd, ns, handler) \
  app.SrvMQTT(AdminCommand::cmd, ns::FromMQTT::handler)
#define REGISTER_SERVICE(cmd, ns, handler)                   \
  do {                                                       \
    app.SrvBLE(ServiceCommand::cmd, ns::FromBLE::handler);   \
    app.SrvMQTT(ServiceCommand::cmd, ns::FromMQTT::handler); \
  } while (0)

void Handler::RegisterAllServices() {
  App &app = App::GetInstance();

  // Device internal
  REGISTER_BLE_SERVICE(BLE_ECHO_TEST, BleHandler, EchoTest);
  REGISTER_BLE_SERVICE(GET_DEBUG_LOG, DeviceHandler, GetDebugLog);
  REGISTER_BLE_SERVICE(GET_SECURE_BOOT_DIGEST, DeviceHandler,
                       GetSecureBootDigest);
  REGISTER_BLE_SERVICE(PING_MQTT_TELEMETRY, DeviceHandler, PingMQTTTelemetry);
  REGISTER_BLE_SERVICE(PING_HTTP, DeviceHandler, PingHTTP);
  REGISTER_BLE_SERVICE(SWITCH_FPGA_FIRMWARE, DeviceHandler, SwitchFPGAFirmware);

#ifdef CONFIG_ENABLE_RFTEST
  REGISTER_BLE_SERVICE(SET_TEST_MODE_A, DeviceHandler, SetTestModeA);
  REGISTER_BLE_SERVICE(SET_TEST_MODE_B, DeviceHandler, SetTestModeB);
#endif

  REGISTER_MQTT_ADMIN_SERVICE(GET_DEVICE_PASSWORD, DeviceHandler,
                              GetDevicePassword);
  REGISTER_MQTT_ADMIN_SERVICE(MANAGE_FPGA_CONFIG, DeviceHandler,
                              ManageFPGAConfig);
  REGISTER_MQTT_ADMIN_SERVICE(MANAGE_POWER_CONFIG, DeviceHandler,
                              ManagePowerConfig);
  REGISTER_MQTT_ADMIN_SERVICE(MANAGE_FEATURE_TOGGLE, DeviceHandler,
                              ManageFeatureToggle);
  REGISTER_SERVICE(ENABLE_RELEASE_MODE, DeviceHandler, EnableReleaseMode);
  REGISTER_SERVICE(GET_HARDWARE_REVISION, DeviceHandler, GetHardwareRevision);

  // Device
  REGISTER_BLE_SERVICE(ASSOCIATE_DEVICE, DeviceHandler, AssociateDevice);
  REGISTER_BLE_SERVICE(GET_DEVICE_UPTIME, DeviceHandler, GetDeviceUpTime);

  REGISTER_MQTT_ADMIN_SERVICE(PUSH_LICENSE, DeviceHandler, PushLicense);
  REGISTER_MQTT_SERVICE(GET_DEVICE_INFO, DeviceHandler, GetDeviceInfo);
  REGISTER_MQTT_ADMIN_SERVICE(SET_SYSLOG_STATE, DeviceHandler, SetSyslogState);
  REGISTER_MQTT_ADMIN_SERVICE(SET_SYSTEM_TIME, DeviceHandler, SetSystemTime);
  REGISTER_MQTT_ADMIN_SERVICE(SET_MACHINE_CONFIG, ConfigSyncHandler,
                              SetMachineConfig);
  REGISTER_MQTT_ADMIN_SERVICE(CLEANUP_NVS_CONFIGS, ConfigSyncHandler,
                              CleanupNVSConfigs);

  REGISTER_MQTT_ADMIN_SERVICE(SCHEDULE_TASK, TaskHandler, ScheduleTask);
  REGISTER_MQTT_ADMIN_SERVICE(GET_TASK_STATUS, TaskHandler, GetTaskStatus);

  REGISTER_SERVICE(GET_AP_VERSION, DeviceHandler, GetAPVersion);
  REGISTER_SERVICE(REBOOT_DEVICE, DeviceHandler, RebootDevice);
  REGISTER_SERVICE(RESET_DEVICE, DeviceHandler, ResetDevice);
  REGISTER_SERVICE(GET_DEVICE_SERIAL_NO, DeviceHandler, GetDeviceSerialNumber);
  REGISTER_SERVICE(SET_DEVICE_POWER_STATE, DeviceHandler, SetPowerState);
  REGISTER_SERVICE(GET_DEVICE_POWER_STATE, DeviceHandler, GetPowerState);

  // OTA
  REGISTER_MQTT_SERVICE(START_OTA, OTAHandler, StartOTA);
  // Ble
  REGISTER_BLE_SERVICE(GET_BLE_RSSI, BleHandler, GetBleRSSI);
  REGISTER_BLE_SERVICE(GET_BLE_MTU, BleHandler, GetBleMTU);

  REGISTER_SERVICE(SET_BLE_STATE, BleHandler, SetBleState);

  // Wi-Fi
  REGISTER_BLE_SERVICE(SET_WIFI_SSID, WiFiHandler, SetWiFiSSID);
  REGISTER_BLE_SERVICE(SET_WIFI_PASSWORD, WiFiHandler, SetWiFiPassword);
  REGISTER_BLE_SERVICE(RESET_WIFI, WiFiHandler, ResetWiFi);
  REGISTER_BLE_SERVICE(GET_WIFI_STATUS, WiFiHandler, GetWiFiStatus);

  REGISTER_MQTT_SERVICE(GET_WIFI_RECORDS, WiFiHandler, GetWiFiRecords);
  REGISTER_MQTT_SERVICE(OPERATE_WIFI_RECORD, WiFiHandler, OperateWiFiRecord);
  REGISTER_MQTT_SERVICE(SET_WIFI_STATE_MACHINE, WiFiHandler,
                        SetWiFiStateMachine);

  REGISTER_SERVICE(SCAN_WIFI, WiFiHandler, ScanWiFi);
  REGISTER_SERVICE(SET_WIFI_SSID_AND_PASSWORD, WiFiHandler,
                   SetWiFiSSIDAndPassword);
  REGISTER_SERVICE(GET_WIFI_STATE_MACHINE, WiFiHandler, GetWiFiStateMachine);

  // Power
  REGISTER_BLE_SERVICE(GET_CHARGE_DURATION, PowerHandler, GetChargeDuration);

  REGISTER_MQTT_SERVICE(GET_POWER_HISTORICAL_STATS, PowerHandler,
                        GetPowerHistoricalStats);
  REGISTER_MQTT_SERVICE(SET_CHARGING_STRATEGY, PowerHandler,
                        SetChargingStrategy);
  REGISTER_MQTT_SERVICE(SET_PORT_PRIORITY, PowerHandler, SetPortPriority);
  REGISTER_MQTT_SERVICE(GET_PORT_PRIORITY, PowerHandler, GetPortPriority);
  REGISTER_SERVICE(GET_CHARGING_STRATEGY, PowerHandler, GetChargingStrategy);
  REGISTER_MQTT_SERVICE(GET_PORT_PD_STATUS, PowerHandler, GetPortPDStatus);
  REGISTER_MQTT_SERVICE(TURN_ON_PORT, PowerHandler, TurnOnPort);
  REGISTER_MQTT_SERVICE(TURN_OFF_PORT, PowerHandler, TurnOffPort);
  REGISTER_MQTT_SERVICE(SET_STATIC_ALLOCATOR, PowerHandler, SetStaticAllocator);
  REGISTER_SERVICE(GET_STATIC_ALLOCATOR, PowerHandler, GetStaticAllocator);
  REGISTER_MQTT_SERVICE(SET_PORT_COMPATIBILITY_SETTINGS, PowerHandler,
                        SetPortCompatibilitySettings);
  REGISTER_MQTT_SERVICE(GET_PORT_COMPATIBILITY_SETTINGS, PowerHandler,
                        GetPortCompatibilitySettings);
  REGISTER_MQTT_SERVICE(SET_TEMPERATURE_MODE, PowerHandler, SetTemperatureMode);
  REGISTER_MQTT_SERVICE(SET_TEMPORARY_ALLOCATOR, PowerHandler,
                        SetTemporaryAllocator);
  REGISTER_SERVICE(SET_PORT_CONFIG, PowerHandler, SetPortConfig);
  REGISTER_SERVICE(GET_PORT_CONFIG, PowerHandler, GetPortConfig);
  REGISTER_BLE_SERVICE(GET_PORTS_VIN, PowerHandler, GetPortsVin);
  REGISTER_MQTT_SERVICE(SUBSCRIBE_PD_PCAP_STREAMING, PowerHandler,
                        SubscribePDPcapStreaming);
  REGISTER_MQTT_SERVICE(SET_PORT_TYPE, PowerHandler, SetPortType);
  REGISTER_MQTT_SERVICE(SET_CUSTOM_PDO_VOLTAGE, PowerHandler,
                        SetCustomPdoVoltage);
  REGISTER_MQTT_SERVICE(SET_CABLE_COMPENSATION, PowerHandler,
                        SetCableCompensation);
  REGISTER_MQTT_SERVICE(GET_CABLE_COMPENSATION, PowerHandler,
                        GetCableCompensation);

  REGISTER_SERVICE(GET_CHARGING_STATUS, PowerHandler, GetChargingStatus);
  REGISTER_SERVICE(GET_ALL_POWER_STATISTICS, PowerHandler, GetAllPowerStats);
  REGISTER_BLE_SERVICE(BLE_FORWARD_PORT, PowerHandler, ForwardPort);
  REGISTER_MQTT_ADMIN_SERVICE(FORWARD_PORT, PowerHandler, ForwardPort);
  REGISTER_MQTT_ADMIN_SERVICE(READ_ADC, PowerHandler, ReadADCValue);
  REGISTER_MQTT_ADMIN_SERVICE(TOGGLE_MCU_GPIO, PowerHandler, ToggleMcuGPIO);

  // Display
  REGISTER_BLE_SERVICE(SET_DISPLAY_STATE, DisplayHandler, SetDisplayState);
  REGISTER_BLE_SERVICE(GET_DISPLAY_STATE, DisplayHandler, GetDisplayState);

  REGISTER_SERVICE(SET_DISPLAY_INTENSITY, DisplayHandler, SetDisplayIntensity);
  REGISTER_SERVICE(GET_DISPLAY_INTENSITY, DisplayHandler, GetDisplayIntensity);
  REGISTER_MQTT_SERVICE(SET_DISPLAY_MODE, DisplayHandler, SetDisplayMode);
  REGISTER_MQTT_SERVICE(GET_DISPLAY_MODE, DisplayHandler, GetDisplayMode);
  REGISTER_MQTT_SERVICE(SET_DISPLAY_FLIP, DisplayHandler, SetDisplayFlip);
  REGISTER_MQTT_SERVICE(GET_DISPLAY_FLIP, DisplayHandler, GetDisplayFlip);
  REGISTER_MQTT_SERVICE(SET_DISPLAY_CONFIG, DisplayHandler, SetDisplayConfig);
  REGISTER_SERVICE(GET_DISPLAY_ROTATION, DisplayHandler, GetDisplayRotation);
  REGISTER_SERVICE(SET_DISPLAY_ROTATION, DisplayHandler, SetDisplayRotation);
  REGISTER_MQTT_SERVICE(CLEAR_FACTORY_DM_FLAG, DisplayHandler,
                        ClearFactoryDMFlag);
  REGISTER_MQTT_SERVICE(SET_DISPLAY_SETUP, DisplayHandler, SetDisplaySetup);
  REGISTER_MQTT_SERVICE(GET_DISPLAY_SETUP, DisplayHandler, GetDisplaySetup);

  // Telemetry Stream
  REGISTER_MQTT_SERVICE(START_TELEMETRY_STREAM, TelemetryStreamHandler,
                        StartTelemetryStream);
  REGISTER_MQTT_SERVICE(STOP_TELEMETRY_STREAM, TelemetryStreamHandler,
                        StopTelemetryStream);
  REGISTER_MQTT_ADMIN_SERVICE(SET_TELEMETRY_DEBUG, TelemetryStreamHandler,
                              SetTelemetryDebug);

  ESP_LOGI(TAG, "All services registered");
}
