#ifndef SW3566_COMMAND_TYPES_H_
#define SW3566_COMMAND_TYPES_H_

#include <cstdint>

#include "data_types.h"

#define DEFINE_SW3566_COMMAND(name, code, hex) name = code

typedef enum {
  // General Operations
  DEFINE_SW3566_COMMAND(KEEP_ALIVE, 0, 0x00),
  DEFINE_SW3566_COMMAND(BOOT, 2, 0x02),
  // 功率分配相关
  DEFINE_SW3566_COMMAND(SET_SUBSCRIPTIONS, 26, 0x1A),
  DEFINE_SW3566_COMMAND(SET_MAX_POWER_BUDGET, 27, 0x1B),
  DEFINE_SW3566_COMMAND(GET_MAX_POWER_BUDGET, 28, 0x1C),
  DEFINE_SW3566_COMMAND(SET_PORT_STATE, 29, 0x1D),
  DEFINE_SW3566_COMMAND(GET_PORT_DETAILS, 31, 0x1F),
  DEFINE_SW3566_COMMAND(CHARGING_ALERT, 32, 0x20),
  DEFINE_SW3566_COMMAND(GET_PD_STATUS, 34, 0x22),
  DEFINE_SW3566_COMMAND(SET_POWER_FEATURES, 35, 0x23),
  DEFINE_SW3566_COMMAND(GET_POWER_FEATURES, 36, 0x24),
  DEFINE_SW3566_COMMAND(SET_CUSTOM_PDO_VOLTAGE, 37, 0x25),
  DEFINE_SW3566_COMMAND(SYSTEM_FLAGS, 48, 0x30),
  DEFINE_SW3566_COMMAND(GET_PD_PCAP_DATA, 192, 0xC0),

  // 3566 programmer commands
  DEFINE_SW3566_COMMAND(PROGRAMMER_START_CMD, 240, 0xF0),
  DEFINE_SW3566_COMMAND(PROGRAMMER_DONE_CMD, 241, 0xF1),
  DEFINE_SW3566_COMMAND(PROGRAMMER_ERROR_CMD, 242, 0xF2),

  // Exceptions
  DEFINE_SW3566_COMMAND(INVALID_COMMAND, 255, 0xFF),
  DEFINE_SW3566_COMMAND(UNCORRECTABLE, 65280, 0xFF00),
} SW3566Command;

typedef enum {
  OK = 0,
  FAILED = 1,
} GenericStatus;

typedef struct {
  uint8_t nonce;
} KeepAliveRequest;

typedef enum {
  BOOTLOADER_ALIVE,
  USER_APPLICATION_ALIVE,
  USER_APPLICATION_SAFE_MODE
} KeepAliveStatus;

typedef struct __attribute__((packed)) {
  uint8_t nonce;
  KeepAliveStatus status;
  union {
    struct {
      uint32_t uptimeMS;
      uint32_t rebootReason;
    };
  };
} KeepAliveResponse;

typedef struct {
  uint8_t nonce;
} BootRequest;

typedef struct {
  GenericStatus status;
} BootResponse;

typedef struct __attribute__((packed)) {
  bool EnablePortDetailsUpdate : 1;
  bool EnablePDStatusUpdate : 1;
  bool EnablePDPcapStreaming : 1;
  uint8_t unused : 5;
} Subscriptions;

typedef struct __attribute__((packed)) {
  uint8_t nonce;
  Subscriptions subscriptions;
  uint32_t host_uptime_ms;
} SetSubscriptionsRequest;

typedef struct {
  GenericStatus status;
} SetSubscriptionsResponse;

typedef struct {
  uint8_t nonce;
} GetFastChargingProtocolRequest;

typedef struct __attribute__((packed)) {
  GenericStatus status;
  FastChargingProtocol protocol;
} GetFastChargingProtocolResponse;

typedef struct {
  uint8_t nonce;
} GetUpTimeRequest;

typedef struct {
  uint32_t upTimeMS;
} GetUpTimeResponse;

typedef struct {
  uint8_t nonce;
} GetPortStateRequest;

typedef struct {
  GenericStatus status;
  bool connected;
} GetPortStateResponse;

typedef struct {
  uint32_t unlock_keys[4];
} UnlockBootloaderRequest;

typedef struct {
  GenericStatus status;
} UnlockBootloaderResponse;

typedef struct __attribute__((packed)) {
  bool shutdown;
} SetPortStateRequest;

typedef struct {
  GenericStatus status;
} SetPortStateResponse;

typedef struct {
  uint8_t nonce;
} PDStatusRequest;

typedef struct __attribute__((packed)) {
  GenericStatus status;
  union {
    // No additional data for FAILED status
    ClientPDStatus data;  // Present only if status == STATUS_OK
  };
} PDStatusResponse;

typedef struct {
  uint8_t nonce;
} PortDetailsRequest;

typedef struct __attribute__((packed)) {
  GenericStatus status;
  PortDetails details;
} PortDetailsResponse;

typedef struct __attribute__((packed)) {
  uint8_t nonce;
  PowerFeatures features;
} SetPowerFeaturesRequest;

typedef struct {
  GenericStatus status;
} SetPowerFeaturesResponse;

typedef struct {
  uint8_t nonce;
} GetPowerFeaturesRequest;

typedef struct __attribute__((packed)) {
  GenericStatus status;
  PowerFeatures features;
} GetPowerFeaturesResponse;

typedef struct __attribute__((packed)) {
  uint8_t max_power_budget;
  uint8_t watermark;
  bool force_rebroadcast : 1;
  uint8_t unused : 7;
} SetMaxPowerBudgetRequest;

typedef struct {
  uint8_t set_budget;
} SetMaxPowerBudgetResponse;

typedef struct {
  uint8_t nonce;
} GetMaxPowerBudgetRequest;

typedef struct __attribute__((packed)) {
  GenericStatus status;
  uint8_t max_power_budget;
} GetMaxPowerBudgetResponse;

typedef struct __attribute__((packed)) {
  uint8_t nonce;
  uint16_t voltage_mv;
} SetCustomPdoVoltageRequest;

typedef struct {
  GenericStatus status;
} SetCustomPdoVoltageResponse;

enum class CableCompensation : uint8_t {
  VALUE_100MOHM = 0,
  VALUE_65MOHM = 1,
};

enum class FixedVoltageOffset : uint8_t {
  OFFSET_0MV = 0,
  OFFSET_100MV = 1,
  OFFSET_150MV = 2,
  OFFSET_200MV = 3,
};

typedef struct __attribute__((packed)) {
  uint8_t commit;  // Must be 0x55 to commit, otherwise return current value
  bool enable_flash_cache : 1;  // default is true
  bool port_type : 1;           // 0 - Type A, 1 - Type C, default is Type C
  bool enable_cable_compensation : 1;  // default is true
  uint8_t cable_compensation : 1;  // 0 - 100mOhm, 1 - 65mOhm, default is 65mOhm
  uint8_t fixed_voltage_offset : 2;  // 0 - 0mV, 1 - 100mV, 2 - 150mV, 3 - 200mV
  uint8_t unused : 2;
} SystemFlags;

typedef union __attribute__((packed)) {
  struct {
    uint8_t head;
    uint8_t tail;
    uint8_t overrun;
    bool overrun_flag : 1;
    bool uncorrectable_flag : 1;
    uint8_t unused : 2;
    uint8_t overrun_count : 4;
  };
  uint32_t word;
} UARTHeader;

typedef struct __attribute__((packed)) {
  UARTHeader header;
  uint8_t payload[0x40];
} UncorrectableError;

typedef struct __attribute__((packed)) {
  uint16_t raw_command;
  uint8_t raw_length;
  uint8_t marker;  // should be 0xFF always
  uint8_t raw_payload[0x40];
} InvalidCommandError;

typedef enum {
  POWER_RAIL_12V,
  POWER_RAIL_20V,
  POWER_RAIL_28V,
  POWER_RAIL_MAX,
} VinVoltageLevel;

typedef struct __attribute__((packed)) {
  bool rapid_reconnect : 1;
  bool pd_rx_hard_reset : 1;
  bool pd_rx_error : 1;
  bool pd_rx_cable_reset : 1;

  uint8_t rebroadcast_reason : 2;
  bool rebroadcast : 1;
  bool gpio_toggled : 1;

  VinVoltageLevel vin_voltage_request : 3;

  uint8_t max_power;
  uint8_t watermark;

  uint32_t power_output;
} ChargingAlert;

typedef enum {
  PCAP_DIR_TX,
  PCAP_DIR_RX,
} PcapDirection;

typedef struct __attribute__((packed)) {
  PcapDirection direction : 1;  // 0 - TX, 1 - RX
  uint8_t sop : 2;              // 1 - SOP, 2 - SOP', 3 - SOP''
  uint8_t serial : 3;
  uint8_t reserved : 2;
} PcapPDPacketHeader;

typedef struct __attribute__((packed)) {
  uint16_t timestamp;  // ticks since start of packet capture
  PcapPDPacketHeader header;
  uint8_t length;  // Payload length
  uint16_t vout;
  uint16_t iout;
  // TYPEC_STATE typec_state[2];
  uint8_t payload[];  // Flexible array member for payload
} PcapEntry;

#endif
