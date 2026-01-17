#include "fpga_data_types.h"

#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_key.h"
#include "nvs_namespace.h"
#include "rpc.h"
#include "sdkconfig.h"

#define FPGA_ADC_CFGS_COUNT CONFIG_FPGA_ADC_CFGS_COUNT
#define FPGA_ADC_CFG_DEFAULT CONFIG_FPGA_ADC_CFG_DEFAULT

using namespace nvs_namespace::user_data;

static const char* TAG = "ADC_CFG";

static const FPGAADCCfg_t DEFAULT_CFGS[FPGA_ADC_CFGS_COUNT] = {
#if CONFIG_FPGA_ADC_CFGS_COUNT >= 1
    {{CONFIG_FPGA_ADC_CFG0_THRESHOLD_LOW, CONFIG_FPGA_ADC_CFG0_THRESHOLD_HIGH,
      CONFIG_FPGA_ADC_CFG0_ACTION_DEADZONE, 0}},
#endif
#if CONFIG_FPGA_ADC_CFGS_COUNT >= 2
    {{CONFIG_FPGA_ADC_CFG1_THRESHOLD_LOW, CONFIG_FPGA_ADC_CFG1_THRESHOLD_HIGH,
      CONFIG_FPGA_ADC_CFG1_ACTION_DEADZONE, 0}},
#endif
#if CONFIG_FPGA_ADC_CFGS_COUNT >= 3
    {{CONFIG_FPGA_ADC_CFG2_THRESHOLD_LOW, CONFIG_FPGA_ADC_CFG2_THRESHOLD_HIGH,
      CONFIG_FPGA_ADC_CFG2_ACTION_DEADZONE, 0}},
#endif
#if CONFIG_FPGA_ADC_CFGS_COUNT >= 4
    {{CONFIG_FPGA_ADC_CFG3_THRESHOLD_LOW, CONFIG_FPGA_ADC_CFG3_THRESHOLD_HIGH,
      CONFIG_FPGA_ADC_CFG3_ACTION_DEADZONE, 0}},
#endif
};
static uint32_t nvs_data[FPGA_ADC_CFGS_COUNT] = {0};

std::unique_ptr<FPGAADCConfig> FPGAADCConfig::Create(uint8_t level) {
  if (level >= FPGA_ADC_CFGS_COUNT) {
    ESP_LOGE(TAG, "Invalid ADC configuration level: %d", level);
    return nullptr;  // Invalid level
  }
  return std::unique_ptr<FPGAADCConfig>(new FPGAADCConfig(level));
}

std::unique_ptr<FPGAADCConfig> FPGAADCConfig::Default() {
  return Create(FPGA_ADC_CFG_DEFAULT);
}

esp_err_t FPGAADCConfig::ApplyDefaultConfig() {
  auto config = Default();
  if (!config) {
    ESP_LOGE(TAG, "Failed to create FPGA ADC configuration");
    return ESP_ERR_INVALID_ARG;
  }
  return config->LoadAndApply();
}

esp_err_t FPGAADCConfig::EnsureNVSLoaded() {
  static std::once_flag nvs_loaded_flag;
  esp_err_t err = ESP_OK;
  ESP_RETURN_ON_ERROR(
      FPGA.GetOrDefault(nvs_keys::FPGA_POWER_CONTROL, control_enabled), TAG,
      "Failed to get FPGA power control setting from NVS");
  std::call_once(nvs_loaded_flag, [&err]() {
    err = FPGA.Get(nvs_keys::FPGA_CONFIGS, nvs_data);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
      memset(nvs_data, 0, sizeof(nvs_data));
      err = ESP_OK;
    }
  });
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to load NVS data for FPGA ADC configuration: %s",
             esp_err_to_name(err));
  }
  return err;
}

esp_err_t FPGAADCConfig::Load() {
  ESP_RETURN_ON_ERROR(EnsureNVSLoaded(), TAG,
                      "Failed to load FPGA ADC configuration from NVS");
  if (nvs_data[level] == 0) {
    // If no configuration is stored, use default values
    memcpy(&config, &DEFAULT_CFGS[level], sizeof(FPGAADCCfg_t));
  } else {
    // Load the configuration from NVS
    config.word = nvs_data[level];
  }
  return ESP_OK;
};

esp_err_t FPGAADCConfig::Save() {
  // Save the configuration to NVS
  ESP_RETURN_ON_ERROR(EnsureNVSLoaded(), TAG,
                      "Failed to load FPGA ADC configuration from NVS");
  nvs_data[level] = config.word;
  return FPGA.Set(nvs_keys::FPGA_CONFIGS, nvs_data);
};

esp_err_t FPGAADCConfig::Apply() {
  if (!control_enabled) {
    return ESP_OK;
  }
  ESP_RETURN_ON_ERROR(PauseControl(), TAG,
                      "Failed to pause FPGA power control");
  // Apply the configuration to the FPGA
  ESP_RETURN_ON_ERROR(rpc::fpga::set_adc_threshold(config.fields.thresholdLow,
                                                   config.fields.thresholdHigh),
                      TAG, "Failed to set ADC threshold");
  ESP_RETURN_ON_ERROR(
      rpc::fpga::set_action_deadzone(config.fields.actionDeadzone), TAG,
      "Failed to set action deadzone");
  ESP_LOGI(TAG, "Applied FPGA ADC configuration level %d: (%d, %d, %d)", level,
           config.fields.thresholdLow, config.fields.thresholdHigh,
           config.fields.actionDeadzone);
  return ResumeControl();
};

esp_err_t FPGAADCConfig::LoadAndApply() {
  ESP_RETURN_ON_ERROR(Load(), TAG, "Failed to load FPGA ADC configuration");
  return Apply();
}

esp_err_t FPGAADCConfig::Reset() {
  ESP_RETURN_ON_ERROR(EnsureNVSLoaded(), TAG,
                      "Failed to load FPGA ADC configuration from NVS");
  // Reset the configuration to default values
  memcpy(&config, &DEFAULT_CFGS[level], sizeof(FPGAADCCfg_t));
  nvs_data[level] = 0;
  return FPGA.Set(nvs_keys::FPGA_CONFIGS, nvs_data);
};

esp_err_t FPGAADCConfig::PauseControl() {
#ifdef CONFIG_ENABLE_FPGA_POWER_CTRL
  return rpc::fpga::toggle_power_control(false);
#else
  return ESP_OK;  // Power control is not enabled, nothing to pause
#endif
}

esp_err_t FPGAADCConfig::ResumeControl() {
#ifdef CONFIG_ENABLE_FPGA_POWER_CTRL
  return rpc::fpga::toggle_power_control(true);
#else
  return ESP_OK;  // Power control is not enabled, nothing to resume
#endif
}
