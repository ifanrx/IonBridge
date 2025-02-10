#include "power_config.h"

#include <cstdint>
#include <cstring>

#include "esp_check.h"
#include "esp_err.h"
#include "nvs_namespace.h"
#include "sdkconfig.h"

static const char *TAG = "PowerConfig";

PowerConfig default_power_config = {
    .version = 0,
    .power_budget = CONFIG_POWER_BUDGET,

    .device_high_temp_threshold = CONFIG_DEVICE_HIGH_TEMP_THRESHOLD,
    .device_high_temp_min_power = CONFIG_DEVICE_HIGH_TEMP_MIN_POWER_LIMIT,
    .device_low_temp_threshold = CONFIG_DEVICE_LOW_TEMP_THRESHOLD,
    .device_over_temp_decrement = CONFIG_DEVICE_OVER_TEMP_DECREMENT,
    .device_under_temp_increment = CONFIG_DEVICE_UNDER_TEMP_INCREMENT,
    .device_power_cooldown_time = CONFIG_DEVICE_POWER_COOLDOWN_TIME,

    .overprovisioning_threshold = CONFIG_OVERPROVISIONING_THRESHOLD,
    .cap_increase_min_threshold = CONFIG_PORT_POWER_CAP_INCREASE_MIN_REMAINING,
    .cap_decrease_min_threshold = CONFIG_PORT_POWER_CAP_DECREASE_MIN_REMAINING,
    .port_power_cap_increase_threshold =
        CONFIG_PORT_POWER_CAP_INCREASE_THRESHOLD,
    .port_power_cap_increase_step = CONFIG_PORT_POWER_CAP_INCREASE_STEP,
    .port_power_cap_decrease_threshold =
        CONFIG_PORT_POWER_CAP_DECREASE_THRESHOLD,
    .high_power_usage_threshold = CONFIG_HIGH_POWER_USAGE_THRESHOLD,
    .low_power_usage_threshold = CONFIG_LOW_POWER_USAGE_THRESHOLD,
    .port_power_adjustment_threshold = CONFIG_PORT_POWER_ADJUSTMENT_THRESHOLD,
};

PowerConfig power_config = default_power_config;

void init_power_config() {
  esp_err_t ret;
  size_t length = sizeof(PowerConfig);
  ret = PowerNVSGet(reinterpret_cast<uint8_t *>(&power_config), &length,
                    NVSKey::POWER_CONFIG);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to get power config from NVS err=0x%04X, use default",
             ret);
    power_config = default_power_config;
  }
}

esp_err_t reset_power_config() {
  ESP_RETURN_ON_ERROR(PowerNVSEraseKey(NVSKey::POWER_CONFIG), TAG,
                      "PowerNVSData::ErasePowerConfig");
  power_config = default_power_config;
  return ESP_OK;
}

esp_err_t set_power_config(PowerConfig &config) {
  if (config.version != power_config.version) {
    return ESP_ERR_INVALID_ARG;
  }
  uint8_t data[sizeof(PowerConfig)];
  std::memcpy(data, &config, sizeof(PowerConfig));
  for (uint8_t i = 1; i < sizeof(PowerConfig); i++) {
    if (data[i] == 0) {
      ESP_LOGI(TAG, "Invalid power config at %d", i);
      return ESP_ERR_INVALID_ARG;
    }
  }
  ESP_RETURN_ON_ERROR(
      PowerNVSSet(data, NVSKey::POWER_CONFIG, sizeof(PowerConfig)), TAG,
      "PowerNVSData::SavePowerConfig");
  power_config = config;
  return ESP_OK;
}

esp_err_t get_power_config(PowerConfig *config) {
  *config = power_config;
  return ESP_OK;
}

void set_power_temperature_thresholds(uint8_t high_temp_threshold,
                                      uint8_t low_temp_threshold) {
  power_config.device_high_temp_threshold = high_temp_threshold;
  power_config.device_low_temp_threshold = low_temp_threshold;
}
