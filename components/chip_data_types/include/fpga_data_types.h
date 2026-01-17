#ifndef FPGA_DATA_TYPES_H_
#define FPGA_DATA_TYPES_H_

#include <stdint.h>

#include <memory>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  READ_REG = 0x0000,
  WRITE_REG = 0x0100,
} FPGAOperation;

typedef enum __attribute__((packed)) {
  ENABLE_LED = 0x00,
  DISPLAY_PERCENTAGE = 0x01,
  DISPLAY_INTENSITY = 0x02,
  DISPLAY_OR_LED = 0x03,
  DISPLAY_MODE = 0x04,
  DISPLAY_FLIP = 0x05,
  ENABLE_POWER_CONTROL = 0x10,
  P1_PRIORITY = 0x11,
  P2_PRIORITY = 0x12,
  P3_PRIORITY = 0x13,
  P4_PRIORITY = 0x14,
  P5_PRIORITY = 0x15,
  TOGGLE_SW3566_IO2 = 0x16,
  ADC_THRESHOLD_HIGH = 0x17,
  ADC_THRESHOLD_LOW = 0x18,
  ACTION_DEADZONE = 0x19,
  PORT_STATUS = 0x20,
  ADC_VALUE = 0x21,
} FPGARegAddr;

typedef struct {
  FPGARegAddr addr;
  uint8_t value;
} FPGARequest;

typedef struct {
  FPGARegAddr addr;
  uint8_t data;
} FPGAResponse;

typedef union {
  struct {
    uint8_t thresholdLow;
    uint8_t thresholdHigh;
    uint8_t actionDeadzone;
    uint8_t reserved;
  } fields;
  uint32_t word;
} FPGAADCCfg_t;

#ifdef __cplusplus
}
#endif

class FPGAADCConfig {
  const uint8_t level;
  FPGAADCCfg_t config;
  bool control_enabled = true;

  FPGAADCConfig(uint8_t level) : level(level) { config.word = 0; };
  esp_err_t EnsureNVSLoaded();

 public:
  static std::unique_ptr<FPGAADCConfig> Create(uint8_t level);
  static std::unique_ptr<FPGAADCConfig> Default();
  static esp_err_t ApplyDefaultConfig();

  /* Load the ADC configuration from NVS or default values.
   *
   */
  esp_err_t Load();

  esp_err_t Save();

  esp_err_t Apply();

  esp_err_t LoadAndApply();

  esp_err_t Reset();

  esp_err_t Update(uint8_t low, uint8_t high, uint8_t deadzone) {
    if (low > high) {
      return ESP_ERR_INVALID_ARG;  // Low threshold must be less than or equal
                                   // to high
    }
    config.fields.thresholdLow = low;
    config.fields.thresholdHigh = high;
    config.fields.actionDeadzone = deadzone;
    return ESP_OK;
  };

  esp_err_t PauseControl();

  esp_err_t ResumeControl();

  const FPGAADCCfg_t& GetConfig() const { return config; };
  uint8_t GetLevel() const { return level; };
  uint8_t GetThresholdLow() const { return config.fields.thresholdLow; };
  uint8_t GetThresholdHigh() const { return config.fields.thresholdHigh; };
  uint8_t GetActionDeadzone() const { return config.fields.actionDeadzone; };
};

#endif
