#ifndef H_SERVICE_
#define H_SERVICE_

#include <cstdint>

#include "sdkconfig.h"

enum ServiceCommand {
  // Internal
  BLE_ECHO_TEST = 0x00,                   // BLE echo test
  GET_DEBUG_LOG = 0x01,                   // 获取调试日志
  GET_SECURE_BOOT_DIGEST = 0x02,          // 获取 secure boot digest
  PING_MQTT_TELEMETRY = 0x03,             // ping MQTT telemetry
  PING_HTTP = 0x04,                       // ping HTTP
  GET_DEVICE_PASSWORD = 0x05,             // 获取密码
  MANAGE_POWER_ALLOCATOR_ENABLED = 0x09,  // 管理 power allocator enabled
  MANAGE_POWER_CONFIG = 0x0a,             // 管理 power config
  MANAGE_FEATURE_TOGGLE = 0x0b,           // 管理 feature toggle
  ENABLE_RELEASE_MODE = 0x0c,             // 启用 release mode

  // Device
  ASSOCIATE_DEVICE = 0x10,      // 关联设备
  REBOOT_DEVICE = 0x11,         // 重启设备
  RESET_DEVICE = 0x12,          // 重置设备
  GET_DEVICE_SERIAL_NO = 0x13,  // 获取设备序列号
  GET_DEVICE_UPTIME = 0x14,     // 获取设备运行时间
  GET_AP_VERSION = 0x15,        // 获取 ESP32 固件版本
#ifdef CONFIG_MCU_MODEL_SW3566
  GET_BP_VERSION = 0x16,     // 获取 SW3566 固件版本
  GET_FPGA_VERSION = 0x17,   // 获取 FPGA 固件版本
  GET_ZRLIB_VERSION = 0x18,  // 获取 ZRLIB 版本
#endif
  GET_DEVICE_BLE_ADDR = 0x19,  // 获取设备BLE地址
  SWITCH_DEVICE = 0x1a,        // 开关设备
  GET_DEVICE_SWITCH = 0x1b,    // 获取开关状态
  GET_DEVICE_MODEL = 0x1c,     // 获取设备型号
  PUSH_LICENSE = 0x1d,         // 推送 license
  GET_BLE_RSSI = 0x1e,         // 获取 BLE RSSI

  // OTA
  PERFORM_BLE_OTA = 0x20,        // 执行BLE OTA
  PERFORM_WIFI_OTA = 0x21,       // 执行WIFI OTA
  GET_WIFI_OTA_PROGRESS = 0x22,  // 获取WIFI OTA进度
  CONFIRM_OTA = 0x23,            // 确认OTA

  // WIFI
  SCAN_WIFI = 0x30,          // 扫描WIFI
  SET_WIFI_SSID = 0x31,      // 设置WIFI SSID
  SET_WIFI_PASSWORD = 0x32,  // 设置WIFI密码
  RESET_WIFI = 0x33,         // 重置WIFI
  GET_WIFI_STATUS =
      0x34,  // 获取连接的 WIFI SSID, BSSID, RSSI, CHANNEL, PROTOCOL
  GET_DEVICE_WIFI_ADDR = 0x35,        // 获取设备WIFI地址
  SET_WIFI_SSID_AND_PASSWORD = 0x36,  // 设置WIFI SSID和密码
  GET_WIFI_RECORDS = 0x37,            // 获取WIFI记录
  OPERATE_WIFI_RECORD = 0x38,         // 操作WIFI记录

  // Power
  TOGGLE_PORT_POWER = 0x40,        // 切换端口
  GET_POWER_STATISTICS = 0x41,     // 获取电源统计
  GET_POWER_SUPPLY_STATUS = 0x42,  // 获取端口供电状态（是否供电）
  SET_CHARGING_STRATEGY =
      0x43,  // 设置充电策略，有两种模式，分别是高速充电和智能慢充
  GET_CHARGING_STATUS = 0x44,         // 获取端口是否正在充电
  GET_POWER_HISTORICAL_STATS = 0x45,  // 获取历史功率统计
  SET_PORT_PRIORITY = 0x46,           // 设置端口优先级
  GET_PORT_PRIORITY = 0x47,           // 获取端口优先级
  GET_CHARGING_STRATEGY =
      0x48,  // 获取充电策略，有两种模式，分别是高速充电和智能慢充
  GET_PORT_PD_STATUS = 0x49,               // 获取端口 PD 信息
  GET_ALL_POWER_STATISTICS = 0x4a,         // 获取全部端口电源统计
  GET_START_CHARGE_TIMESTAMP = 0x4b,       // 获取开始充电的时间戳
  TURN_ON_PORT = 0x4c,                     // 打开端口
  TURN_OFF_PORT = 0x4d,                    // 关闭端口
  SET_OVERCLOCK_FEATURE = 0x4f,            // 开关端口超频
  GET_OVERCLOCK_FEATURE = 0x50,            // 查询端口超频状态
  SET_TFCP_FEATURE = 0x51,                 // 启用/禁用 TFCP
  GET_TFCP_FEATURE = 0x52,                 // 查询 TFCP 状态
  SET_UFCS_FEATURE = 0x53,                 // 启用/禁用 UFCS
  GET_UFCS_FEATURE = 0x54,                 // 查询 UFCS 状态
  SET_STATIC_ALLOCATOR = 0x55,             // 设置老式充充电策略
  GET_STATIC_ALLOCATOR = 0x56,             // 获取老式充充电策略
  SET_PORT_CONFIG = 0x57,                  // 设置端口配置
  GET_PORT_CONFIG = 0x58,                  // 获取端口配置
  SET_PORT_COMPATIBILITY_SETTINGS = 0x59,  // 设置端口兼容性设置
  GET_PORT_COMPATIBILITY_SETTINGS = 0x5a,  // 获取端口兼容性设置
  SET_TEMPERATURE_MODE = 0x5b,             // 设置温度模式
  SET_TEMPORARY_ALLOCATOR = 0x5c,          // 设置临时充电策略

  // Display
  SET_DISPLAY_INTENSITY = 0x70,
  SET_DISPLAY_MODE = 0x71,
  GET_DISPLAY_INTENSITY = 0x72,
  GET_DISPLAY_MODE = 0x73,
  SET_DISPLAY_FLIP = 0x74,    // 设置屏幕/灯正反方向
  GET_DISPLAY_FLIP = 0x75,    // 获取屏幕/灯正反方向
  SET_DISPLAY_CONFIG = 0x76,  // 设置屏幕
  SET_DISPLAY_STATE = 0x77,   // 设置屏幕状态
  GET_DISPLAY_STATE = 0x78,   // 获取屏幕状态

  // Telemetry Stream
  START_TELEMETRY_STREAM = 0x90,
  STOP_TELEMETRY_STREAM = 0x91,

  GET_DEVICE_INFO = 0x92,  // 获取设备信息

  SET_BLE_STATE = 0x98,     // 开启/关闭蓝牙
  SET_SYSLOG_STATE = 0x99,  // 开启/关闭系统日志上报
  SET_SYSTEM_TIME = 0x9A,   // 设置系统时间
  START_OTA = 0x9C,         // 开始升级
};

enum ResponseStatus : uint8_t {
  SUCCESS = 0x0,                                 // 成功
  FAILURE = 0x1,                                 // 失败
  FLASH_NOT_WRITABLE_AT_HIGH_TEMPERATURE = 0x2,  // 高温下 flash 不可写
};

bool token_required(ServiceCommand service);
bool is_service_available_at_high_temp(ServiceCommand service,
                                       bool pre_check = false);

#endif /* H_SERVICE_ */
