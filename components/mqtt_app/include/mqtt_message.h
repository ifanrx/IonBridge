#ifndef MQTT_MESSAGE_SCHEMA_H_
#define MQTT_MESSAGE_SCHEMA_H_

#include <endian.h>
#include <stdint.h>

#include <cstdint>
#include <cstring>
#include <vector>

#include "data_types.h"
#include "port_data.h"
#include "sdkconfig.h"
#include "utils.h"

#define EMPLACE_BACK_VERSION(payload, version)                  \
  do {                                                          \
    (payload).emplace_back(static_cast<uint8_t>((version)[0])); \
    (payload).emplace_back(static_cast<uint8_t>((version)[1])); \
    (payload).emplace_back(static_cast<uint8_t>((version)[2])); \
  } while (0)

#ifdef __cplusplus
extern "C" {
#endif

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

  REQUEST_OTA_CONFIRMATION = 0xFF00,
  REQUEST_SHUTDOWN_BLE = 0xFF01,
  REQUEST_OTA_CONFIRMATION2 = 0xFF02,
};

struct MQTTMessageHeader {
  uint16_t service;
  uint16_t message_id;

  ~MQTTMessageHeader() = default;
  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> data(4);

    // Copy the bytes to the data vector using memcpy
    std::memcpy(&data[0], &service, sizeof(service));
    std::memcpy(&data[2], &message_id, sizeof(message_id));
    return data;
  }
};

struct MQTTMessage {
  MQTTMessageHeader header;

  virtual ~MQTTMessage() = default;
  virtual std::vector<uint8_t> serialize() const { return header.serialize(); }
};

// 32 bytes
struct PortStatusData {
  uint16_t charging_minutes;  // 2 bytes
  PowerFeatures features;     // 2 bytes
  uint8_t unused0;            // 1 bytes
  PortDetails details;        // 21 bytes
  uint8_t unused1[6];         // 6 bytes
};

// 4 + 32 * 8 = 260 bytes
struct StreamPortStatus : MQTTMessage {
  uint8_t status;
  uint8_t port_status_map;  // 0: close, 1: open ; LSB: port0, MSB: port7
  uint16_t padding;  // Padding to align the ports array to 4-byte boundary
  PortStatusData ports[8];

  std::vector<uint8_t> serialize() const override {
    std::vector<uint8_t> payload = header.serialize();
    payload.reserve(sizeof(*this));
    payload.emplace_back(status);
    payload.emplace_back(port_status_map);

    // Append padding in Big-Endian format (network byte order)
    EMPLACE_BACK_INT16(payload, padding);

    // Append each PortStatusData
    for (const auto &port : ports) {
      // Create a pointer to the current port's data
      const uint8_t *portData = reinterpret_cast<const uint8_t *>(&port);

      // Append the bytes of PortStatusData to the payload
      payload.insert(payload.end(), portData,
                     portData + sizeof(PortStatusData));
    }
    return payload;
  }
};

struct StreamPortPDStatus : MQTTMessage {
  uint8_t status;
  uint8_t port_index;
  ClientPDStatus pd_status;

  std::vector<uint8_t> serialize() const override {
    std::vector<uint8_t> payload = header.serialize();
    payload.reserve(sizeof(*this));

    payload.emplace_back(status);
    payload.emplace_back(port_index);
    const uint8_t *data = reinterpret_cast<const uint8_t *>(&pd_status);
    payload.insert(payload.end(), data, data + sizeof(ClientPDStatus));
    return payload;
  }
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

struct StreamDeviceStatus : MQTTMessage {
  uint8_t status;
  uint8_t allocator;
  DisplayConfiguration display_config;
  uint32_t device_uptime;  // second
  FeatureFlag feature_flag;
  uint16_t charging_minutes;
  uint16_t power_allocator_param;
  uint8_t temperature_mode;

  std::vector<uint8_t> serialize() const override {
    std::vector<uint8_t> payload = header.serialize();
    payload.reserve(sizeof(*this));
    payload.emplace_back(status);
    payload.emplace_back(allocator);
    payload.emplace_back(display_config.intensity);
    payload.emplace_back(display_config.flip);
    payload.emplace_back(display_config.mode);
    EMPLACE_BACK_INT16(payload, display_config.mode_param);
    EMPLACE_BACK_INT32(payload, device_uptime);
    EMPLACE_BACK_INT16(payload, feature_flag.value);
    EMPLACE_BACK_INT16(payload, charging_minutes);
    EMPLACE_BACK_INT16(payload, power_allocator_param);
    payload.emplace_back(temperature_mode);
    return payload;
  }
};

struct PingInfo : MQTTMessage {};

struct DeviceBootInfo : MQTTMessage {
  uint8_t esp32[3];          // version: major minor revision
  uint8_t mcu[3];            // version: major minor revision
  uint8_t fpga[3];           // version: major minor revision
  uint8_t zrlib[3];          // version: major minor revision
  uint8_t ESPIdfVersion[3];  // major minor patch
  uint8_t chipCores;         // number of CPU cores

  uint8_t bleAddress[6];  // 蓝牙地址
  uint8_t ipv4[4];        // IPv4 地址
  uint8_t ipv6[16];       // IPv6 地址

  uint64_t uptime;

  uint16_t chipModel;      // chip model
  uint32_t chipFeatures;   // bit mask of CHIP_FEATURE_x feature flags
  uint16_t chipRevision;   // chip revision number (in format MXX; where M -
  int8_t wifiMaxTxPower;   // Maximum WiFi transmitting power, unit is 0.25dBm.
                           // wafer major version, XX - wafer minor version)
  uint8_t activeMCUCount;  // 正常启动芯片数量
  uint32_t fsTotalSize;    // filesystem total bytes
  uint32_t fsUsedSize;     // filesystem used bytes
  int32_t resetReason;     // reason of last reset
  char hardwareRev[8];     // hardware revision
  char deviceModel[8];     // ultrab/prow/devb/ultrag/fake
  char productFamily[8];   // CP02/CP02L

  std::vector<uint8_t> serialize() const override {
    std::vector<uint8_t> payload = header.serialize();
    payload.reserve(sizeof(*this));
    EMPLACE_BACK_VERSION(payload, esp32);
    EMPLACE_BACK_VERSION(payload, mcu);
    EMPLACE_BACK_VERSION(payload, fpga);
    EMPLACE_BACK_VERSION(payload, zrlib);
    EMPLACE_BACK_VERSION(payload, ESPIdfVersion);
    payload.emplace_back(chipCores);
    payload.insert(payload.end(), bleAddress, bleAddress + sizeof(bleAddress));
    payload.insert(payload.end(), ipv4, ipv4 + sizeof(ipv4));
    payload.insert(payload.end(), ipv6, ipv6 + sizeof(ipv6));

    EMPLACE_BACK_INT64(payload, uptime);

    EMPLACE_BACK_INT16(payload, chipModel);
    EMPLACE_BACK_INT32(payload, chipFeatures);
    EMPLACE_BACK_INT16(payload, chipRevision);
    payload.emplace_back(wifiMaxTxPower);

    payload.emplace_back(activeMCUCount);
    EMPLACE_BACK_INT32(payload, fsTotalSize);
    EMPLACE_BACK_INT32(payload, fsUsedSize);
    EMPLACE_BACK_INT32(payload, resetReason);

    payload.insert(payload.end(), hardwareRev,
                   hardwareRev + sizeof(hardwareRev));
    payload.insert(payload.end(), deviceModel,
                   deviceModel + sizeof(deviceModel));
    payload.insert(payload.end(), productFamily,
                   productFamily + sizeof(productFamily));
    return payload;
  }
};

struct PowerHistoricalData : MQTTMessage {
  uint8_t port;
  uint16_t length;
  std::vector<PortStatsData> data;

  std::vector<uint8_t> serialize() const override {
    std::vector<uint8_t> payload = header.serialize();
    payload.reserve(sizeof(*this));
    payload.emplace_back(port);
    EMPLACE_BACK_INT16(payload, length);
    for (const PortStatsData &point : data) {
      payload.emplace_back(point.GetAmperage());
      payload.emplace_back(point.GetVoltage());
      payload.emplace_back(point.GetTemperature());
      payload.emplace_back(point.GetVinValue());
    }
    return payload;
  }
};

struct PowerConsumptionData : MQTTMessage {
  uint8_t port;
  uint16_t length;
  uint16_t power_consumption;

  std::vector<uint8_t> serialize() const override {
    std::vector<uint8_t> payload = header.serialize();
    payload.reserve(sizeof(*this));
    payload.emplace_back(port);
    EMPLACE_BACK_INT16(payload, length);
    EMPLACE_BACK_INT16(payload, power_consumption);
    return payload;
  }
};

struct DeviceMemoryInfo : MQTTMessage {
  uint32_t freeHeapSize;          // Available heap size, in bytes.
  uint32_t freeInternalHeapSize;  // Available internal heap size, in bytes.
  uint32_t minimumFreeHeapSize;   // Minimum free heap ever available, in bytes.
  uint8_t initialBoot;            // 是否启动后首次上报

  std::vector<uint8_t> serialize() const override {
    std::vector<uint8_t> payload = header.serialize();
    payload.reserve(sizeof(*this));
    payload.emplace_back(initialBoot);
    EMPLACE_BACK_INT32(payload, freeHeapSize);
    EMPLACE_BACK_INT32(payload, freeInternalHeapSize);
    EMPLACE_BACK_INT32(payload, minimumFreeHeapSize);
    return payload;
  }
};

struct PortPowerAllocation {
  uint8_t source_cap;
  uint8_t usage;
};

struct PowerAllocationData : MQTTMessage {
  uint16_t power_budget;
  uint16_t remaining_power;
  uint16_t adc_value;
  uint8_t temperature;
  uint8_t unused;
  PortPowerAllocation port_power_allocations[8];

  std::vector<uint8_t> serialize() const override {
    std::vector<uint8_t> payload = header.serialize();
    payload.reserve(sizeof(*this));
    EMPLACE_BACK_INT16(payload, power_budget);
    EMPLACE_BACK_INT16(payload, remaining_power);
    EMPLACE_BACK_INT16(payload, adc_value);
    payload.emplace_back(temperature);
    payload.emplace_back(unused);
    for (const PortPowerAllocation &point : port_power_allocations) {
      payload.emplace_back(point.source_cap);
      payload.emplace_back(point.usage);
    }
    return payload;
  }
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

struct ESP32UpgradeInfo : MQTTMessage {
  uint8_t version[3];  // major minor revision
  UpgradeStatus status;
  UpgradeFailureReason reason;
  int32_t esp_err;

  std::vector<uint8_t> serialize() const override {
    std::vector<uint8_t> payload = header.serialize();
    payload.reserve(sizeof(*this));
    EMPLACE_BACK_VERSION(payload, version);
    payload.emplace_back(static_cast<uint8_t>(status));
    payload.emplace_back(static_cast<uint8_t>(reason));
    EMPLACE_BACK_INT32(payload, esp_err);
    return payload;
  }
};

#ifdef CONFIG_MCU_MODEL_SW3566
struct FPGAUpgradeInfo : ESP32UpgradeInfo {};

struct SW3566UpgradeInfo : MQTTMessage {
  uint8_t version[3];       // major minor revision
  uint8_t activePortCount;  // 正常启动芯片数量
  UpgradeStatus status;
  UpgradeFailureReason reason;
  int32_t esp_err;

  std::vector<uint8_t> serialize() const override {
    std::vector<uint8_t> payload = header.serialize();
    payload.reserve(sizeof(*this));
    EMPLACE_BACK_VERSION(payload, version);
    payload.emplace_back(static_cast<uint8_t>(status));
    payload.emplace_back(static_cast<uint8_t>(reason));
    payload.emplace_back(activePortCount);
    EMPLACE_BACK_INT32(payload, esp_err);
    return payload;
  }
};
#endif

struct OTAConfirmInfo : MQTTMessage {
  uint8_t hash[32];

  std::vector<uint8_t> serialize() const override {
    std::vector<uint8_t> payload = header.serialize();
    payload.reserve(sizeof(*this));
    payload.insert(payload.end(), hash, hash + sizeof(hash));
    return payload;
  }
};

struct OTAConfirmInfo2 : MQTTMessage {
  uint8_t hash[32];
  uint8_t esp32_version[3];  // major minor revision

  std::vector<uint8_t> serialize() const override {
    std::vector<uint8_t> payload = header.serialize();
    payload.reserve(sizeof(*this));
    payload.insert(payload.end(), hash, hash + sizeof(hash));
    payload.insert(payload.end(), esp32_version,
                   esp32_version + sizeof(esp32_version));
    return payload;
  }
};

struct WIFIStatsData : MQTTMessage {
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

  std::vector<uint8_t> serialize() const override {
    std::vector<uint8_t> payload = header.serialize();
    payload.reserve(sizeof(*this));
    EMPLACE_BACK_INT16(payload, associationCount);
    EMPLACE_BACK_INT16(payload, disassociationCount);
    payload.emplace_back(rssi);
    payload.emplace_back(channel);
    EMPLACE_BACK_INT16(payload, wifiConnectionTime);
    EMPLACE_BACK_INT16(payload, mqttConnectionTime);
    EMPLACE_BACK_INT16(payload, connectivityFailureCount);
    EMPLACE_BACK_INT16(payload, dnsResolutionFailureCount);
    EMPLACE_BACK_INT16(payload, mqttConnectionCount);
    EMPLACE_BACK_INT32(payload, mqttMessageTxCount);
    EMPLACE_BACK_INT32(payload, mqttMessageRxCount);
    return payload;
  }
};

struct RequestLicenseInfo : MQTTMessage {};

struct UartMetricsData : MQTTMessage {
  uint32_t reset_state_count;
  uint32_t resend_count;
  uint32_t sent_failed_count;
  uint32_t sent_count;

  std::vector<uint8_t> serialize() const override {
    std::vector<uint8_t> payload = header.serialize();
    payload.reserve(sizeof(*this));
    EMPLACE_BACK_INT32(payload, reset_state_count);
    EMPLACE_BACK_INT32(payload, resend_count);
    EMPLACE_BACK_INT32(payload, sent_failed_count);
    EMPLACE_BACK_INT32(payload, sent_count);
    return payload;
  }
};

struct RequestSystemTimeInfo : MQTTMessage {};

struct OutOfMemoryAlertInfo : MQTTMessage {
  uint32_t freeHeapSize;
  uint32_t freeInternalHeapSize;
  uint32_t minimumFreeHeapSize;
  uint32_t thresholdFreeHeapSize;

  std::vector<uint8_t> serialize() const override {
    std::vector<uint8_t> payload = header.serialize();
    payload.reserve(sizeof(*this));
    EMPLACE_BACK_INT32(payload, freeHeapSize);
    EMPLACE_BACK_INT32(payload, freeInternalHeapSize);
    EMPLACE_BACK_INT32(payload, minimumFreeHeapSize);
    EMPLACE_BACK_INT32(payload, thresholdFreeHeapSize);
    return payload;
  }
};

struct OverTemperatureAlertInfo : MQTTMessage {
  uint8_t temperature;
  uint8_t thresholdTemperature;
  std::vector<uint8_t> serialize() const override {
    std::vector<uint8_t> payload = header.serialize();
    payload.reserve(sizeof(*this));
    payload.emplace_back(temperature);
    payload.emplace_back(thresholdTemperature);
    return payload;
  }
};

struct RequestShutdownBleInfo : MQTTMessage {};

struct ChargingAlertInfo : MQTTMessage {
  uint8_t port;
  uint16_t pd_rx_soft_reset_count;
  uint16_t pd_rx_hard_reset_count;
  uint16_t pd_rx_error_count;
  uint16_t pd_rx_cable_reset_count;
  uint8_t unused;

  std::vector<uint8_t> serialize() const override {
    std::vector<uint8_t> payload = header.serialize();
    payload.reserve(sizeof(*this));
    payload.emplace_back(port);
    EMPLACE_BACK_INT16(payload, pd_rx_soft_reset_count);
    EMPLACE_BACK_INT16(payload, pd_rx_hard_reset_count);
    EMPLACE_BACK_INT16(payload, pd_rx_error_count);
    EMPLACE_BACK_INT16(payload, pd_rx_cable_reset_count);
    payload.emplace_back(unused);

    return payload;
  }
};

struct SW3566BootloaderInfo : MQTTMessage {
  struct {
    uint32_t version;
    uint32_t flash_timestamp;
  } ports[8];

  std::vector<uint8_t> serialize() const override {
    std::vector<uint8_t> payload = header.serialize();
    payload.reserve(sizeof(*this));
    for (const auto &port : ports) {
      EMPLACE_BACK_INT32(payload, port.version);
      EMPLACE_BACK_INT32(payload, port.flash_timestamp);
    }
    return payload;
  }
};

#ifdef __cplusplus
}
#endif

#endif
