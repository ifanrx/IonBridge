#ifndef MQTT_MESSAGE_SCHEMA_H_
#define MQTT_MESSAGE_SCHEMA_H_

#include <endian.h>
#include <stdint.h>

#include <cstdint>
#include <vector>

#include "data_types.h"
#include "port_data.h"
#include "sdkconfig.h"

enum TelemetryServiceCommand : uint16_t {
  STREAM_PORTS_STATUS = 0x0080,
  STREAM_DEVICE_STATUS = 0x0081,
  STREAM_PORT_PD_STATUS = 0x0082,

  PING = 0x0100,

  DEVICE_BOOT_INFO = 0x0110,
  DEVICE_MEMORY_INFO = 0x0111,
  DEVICE_SYSTEM_STATE = 0x0112,

  POWER_HISTORICAL_DATA = 0x0130,
  POWER_CONSUMPTION_DATA = 0x0131,
  POWER_ALLOCATION_DATA = 0x0132,
  PD_PCAP_DATA = 0x133,
  AGG_POWER_HISTORICAL_DATA = 0x0134,

  // OTA
  ESP32_UPGRADE_DATA = 0x0140,
  OTA_CONFIRM_DATA = 0x0143,

  WIFI_STATS_DATA = 0x0150,
  REQUEST_LICENSE = 0x0151,
  UART_METRICS_DATA = 0x0152,
  REQUEST_SYSTEM_TIME = 0x0153,

  // alert
  OUT_OF_MEMORY_ALERT = 0x0160,
  OVER_TEMPERATURE_ALERT = 0x0161,
  PD_CHARGING_ALERT = 0x0162,

  REQUEST_SHUTDOWN_BLE = 0xFF01,
};

class MQTTMessageHeader {
  static uint16_t message_id_counter;
  uint16_t service;
  uint16_t message_id;

 public:
  MQTTMessageHeader(uint16_t service);
  MQTTMessageHeader(uint16_t service, uint16_t message_id)
      : service(service), message_id(message_id) {}
  ~MQTTMessageHeader() = default;

  uint16_t GetService() const { return service; }
  std::vector<uint8_t> Serialize() const;
};

struct MQTTMessage {
  MQTTMessageHeader header;

  MQTTMessage(uint16_t service) : header(service) {}
  virtual ~MQTTMessage() = default;
  virtual std::vector<uint8_t> Serialize() const { return header.Serialize(); }
};

template <uint16_t DefaultService>
struct MQTTMessageType : MQTTMessage {
  MQTTMessageType() : MQTTMessage(DefaultService) {}
};

// 32 bytes
struct PortStatusData {
  uint16_t charging_minutes;  // 2 bytes
  PowerFeatures features;     // 3 bytes
  PortDetails details;        // 21 bytes
  PortType port_type;         // 1 byte
  uint8_t unused1[5];         // 5 bytes
};

// 4 + 32 * 8 = 260 bytes
struct StreamPortStatus : MQTTMessageType<STREAM_PORTS_STATUS> {
  uint8_t status;
  uint8_t port_status_map;  // 0: close, 1: open ; LSB: port0, MSB: port7
  uint16_t padding;  // Padding to align the ports array to 4-byte boundary
  PortStatusData ports[8];

  std::vector<uint8_t> Serialize() const override;
};

struct StreamPortPDStatus : MQTTMessageType<STREAM_PORT_PD_STATUS> {
  uint8_t status;
  uint8_t port_index;
  ClientPDStatus pd_status;

  std::vector<uint8_t> Serialize() const override;
};

struct DisplayConfiguration {
  uint8_t intensity;
  uint8_t flip;  // DisplayFlipMode
  uint8_t mode;  // DisplayMode
  uint16_t mode_param;
};

union FeatureFlag {
  uint16_t value;
  struct {
    uint8_t device_switch : 1;
    uint8_t ble_state : 1;
    uint8_t syslog_state : 1;
    uint8_t reserved : 5;
    uint8_t unused;
  } flags;
};

struct StreamDeviceStatus : MQTTMessageType<STREAM_DEVICE_STATUS> {
  uint8_t status;
  uint8_t allocator;
  DisplayConfiguration display_config;
  uint32_t device_uptime;  // second
  FeatureFlag feature_flag;
  uint16_t charging_minutes;
  uint16_t power_allocator_param;

  uint8_t temperature_mode;

  std::vector<uint8_t> Serialize() const override;
};

struct PingInfo : MQTTMessageType<PING> {};

struct DeviceBootInfo : MQTTMessageType<DEVICE_BOOT_INFO> {
  uint8_t esp32[3];          // version: major minor revision
  uint8_t mcu[3];            // version: major minor revision
  uint8_t fpga[3];           // version: major minor revision
  uint8_t zrlib[3];          // version: major minor revision
  uint8_t ESPIdfVersion[3];  // major minor patch
  uint8_t chipCores;         // number of CPU cores

  uint8_t bleAddress[6];  // ble address
  uint8_t ipv4[4];        // IPv4 address
  uint8_t ipv6[16];       // IPv6 address

  uint64_t uptime;

  uint16_t chipModel;      // chip model
  uint32_t chipFeatures;   // bit mask of CHIP_FEATURE_x feature flags
  uint16_t chipRevision;   // chip revision number (in format MXX; where M -
  int8_t wifiMaxTxPower;   // Maximum WiFi transmitting power, unit is 0.25dBm.
                           // wafer major version, XX - wafer minor version)
  uint8_t activeMCUCount;  // number of active MCUs in the system
  uint32_t fsTotalSize;    // filesystem total bytes
  uint32_t fsUsedSize;     // filesystem used bytes
  int32_t resetReason;     // reason of last reset
  char hardwareRev[8];     // hardware revision
  char deviceModel[8];     // ultrab/prow/devb/ultrag/fake
  char productFamily[8];   // CP02/CP02L

  std::vector<uint8_t> Serialize() const override;
};

struct PortStatsEntry {
  uint8_t port;
  uint16_t length;
  std::vector<PortStatsData> data;
};

struct AggregatedPowerHistoricalData
    : MQTTMessageType<AGG_POWER_HISTORICAL_DATA> {
  std::vector<PortStatsEntry> data;

  std::vector<uint8_t> Serialize() const override;
};

struct PowerConsumptionData : MQTTMessageType<POWER_CONSUMPTION_DATA> {
  uint8_t port;
  uint16_t length;
  uint16_t power_consumption;

  std::vector<uint8_t> Serialize() const override;
};

struct DeviceMemoryInfo : MQTTMessageType<DEVICE_MEMORY_INFO> {
  uint32_t freeHeapSize;          // Available heap size, in bytes.
  uint32_t freeInternalHeapSize;  // Available internal heap size, in bytes.
  uint32_t minimumFreeHeapSize;   // Minimum free heap ever available, in bytes.
  uint8_t initialBoot;            // 是否启动后首次上报

  std::vector<uint8_t> Serialize() const override;
};

struct PowerAllocationData : MQTTMessageType<POWER_ALLOCATION_DATA> {
  uint16_t power_budget;
  uint16_t remaining_power;
  uint16_t adc_value;
  uint8_t temperature;
  uint8_t unused;
  struct {
    uint8_t source_cap;
    uint8_t usage;
  } port_power_allocations[8];

  std::vector<uint8_t> Serialize() const override;
};

struct PDPCapData : MQTTMessageType<PD_PCAP_DATA> {
  uint8_t port;
  std::vector<uint8_t> pcap_entry;

  std::vector<uint8_t> Serialize() const override;
};

enum UpgradeStatus : uint8_t {
  UPGRADE_START = 0,
  UPGRADE_FAIL = 1,
  UPGRADE_SUCCESS = 2,
};

enum UpgradeFailureReason : uint8_t {
  REASON_NONE = 0,
  REASON_ESP_ERROR = 1,
  REASON_CONFIRM_FAILED = 2,
  REASON_CONFIRM_TIMEOUT = 3,
};

struct UpgradeInfo : MQTTMessage {
  uint8_t version[3];  // major minor revision
  UpgradeStatus status;
  UpgradeFailureReason reason;
  int32_t esp_err;

  UpgradeInfo(uint16_t service) : MQTTMessage(service) {}
  std::vector<uint8_t> Serialize() const override;
};

struct ESP32UpgradeInfo : UpgradeInfo {
  ESP32UpgradeInfo() : UpgradeInfo(ESP32_UPGRADE_DATA) {}
};

struct OTAConfirmInfo : MQTTMessageType<OTA_CONFIRM_DATA> {
  uint8_t hash[32];
  uint8_t esp32_version[3];  // major minor revision

  std::vector<uint8_t> Serialize() const override;
};

struct WIFIStatsData : MQTTMessageType<WIFI_STATS_DATA> {
  uint16_t associationCount;
  uint16_t disassociationCount;
  int8_t rssi;
  uint8_t channel;
  uint16_t wifiConnectionTime;  // seconds from start connection to connected
  uint16_t mqttConnectionTime;  // seconds from start connection to connected
  uint16_t connectivityFailureCount;   // count of failed connection test
  uint16_t dnsResolutionFailureCount;  // count of failed dns resolve test
  uint16_t mqttConnectionCount;        // count of mqtt connection
  uint32_t mqttMessageTxCount;         // count of mqtt message tx
  uint32_t mqttMessageRxCount;         // count of mqtt message rx

  std::vector<uint8_t> Serialize() const override;
};

struct RequestLicenseInfo : MQTTMessageType<REQUEST_LICENSE> {};

struct UartMetricsData : MQTTMessageType<UART_METRICS_DATA> {
  uint32_t reset_state_count;
  uint32_t resend_count;
  uint32_t sent_failed_count;
  uint32_t sent_count;

  std::vector<uint8_t> Serialize() const override;
};

struct RequestSystemTimeInfo : MQTTMessageType<REQUEST_SYSTEM_TIME> {};

struct OutOfMemoryAlertInfo : MQTTMessageType<OUT_OF_MEMORY_ALERT> {
  uint32_t freeHeapSize;
  uint32_t freeInternalHeapSize;
  uint32_t minimumFreeHeapSize;
  uint32_t thresholdFreeHeapSize;

  std::vector<uint8_t> Serialize() const override;
};

struct OverTemperatureAlertInfo : MQTTMessageType<OVER_TEMPERATURE_ALERT> {
  uint8_t temperature;
  uint8_t thresholdTemperature;
  std::vector<uint8_t> Serialize() const override;
};

struct RequestShutdownBleInfo : MQTTMessageType<REQUEST_SHUTDOWN_BLE> {};

struct ChargingAlertInfo : MQTTMessageType<PD_CHARGING_ALERT> {
  uint8_t port;
  uint16_t pd_rx_soft_reset_count;
  uint16_t pd_rx_hard_reset_count;
  uint16_t pd_rx_error_count;
  uint16_t pd_rx_cable_reset_count;
  uint8_t unused;

  std::vector<uint8_t> Serialize() const override;
};

#endif
