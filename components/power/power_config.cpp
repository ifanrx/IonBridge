#include "power_config.h"

#include <cstdint>
#include <cstring>

#include "esp_check.h"
#include "esp_err.h"
#include "nvs_namespace.h"
#include "sdkconfig.h"

// Tag for ESP logging system
static const char *TAG = "PowerConfig";

// Default power configuration parameters initialized from Kconfig values
PowerConfig default_power_config = {
    .version = kPowerConfigVersion,
    .power_budget = CONFIG_POWER_BUDGET,

    // Temperature monitoring thresholds and power adjustment parameters
    .device_high_temp_threshold = CONFIG_DEVICE_HIGH_TEMP_THRESHOLD,
    .device_high_temp_min_power = CONFIG_DEVICE_HIGH_TEMP_MIN_POWER_LIMIT,
    .device_low_temp_threshold = CONFIG_DEVICE_LOW_TEMP_THRESHOLD,
    .device_over_temp_decrement = CONFIG_DEVICE_OVER_TEMP_DECREMENT,
    .device_under_temp_increment = CONFIG_DEVICE_UNDER_TEMP_INCREMENT,
    .device_power_cooldown_time = CONFIG_DEVICE_POWER_COOLDOWN_TIME,

    // Power allocation and adjustment thresholds
    .overprovisioning_threshold = CONFIG_OVERPROVISIONING_THRESHOLD,
    .cap_increase_min_threshold = CONFIG_PORT_POWER_CAP_INCREASE_MIN_REMAINING,
    .cap_decrease_min_threshold = CONFIG_PORT_POWER_CAP_DECREASE_MIN_REMAINING,
    .port_power_cap_increase_threshold = CONFIG_PORT_POWER_CAP_INCREASE_THRESHOLD,
    .port_power_cap_increase_step = CONFIG_PORT_POWER_CAP_INCREASE_STEP,
    .port_power_cap_decrease_threshold = CONFIG_PORT_POWER_CAP_DECREASE_THRESHOLD,
    .high_power_usage_threshold = CONFIG_HIGH_POWER_USAGE_THRESHOLD,
    .low_power_usage_threshold = CONFIG_LOW_POWER_USAGE_THRESHOLD,
    .port_power_adjustment_threshold = CONFIG_PORT_POWER_ADJUSTMENT_THRESHOLD,
};

// Global power configuration instance
PowerConfig power_config = default_power_config;

/**
 * @brief Initialize power configuration from NVS storage
 * 
 * Attempts to load configuration from NVS. If loading fails or version mismatch,
 * falls back to default configuration. Handles legacy configuration format conversion.
 */
void init_power_config() {
  esp_err_t ret;
  uint8_t data[sizeof(PowerConfig)];
  size_t length = sizeof(PowerConfig);
  ret = PowerNVSGet(data, &length, NVSKey::POWER_CONFIG);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "NVS power configuration retrieval failed (err=0x%04X), using default configuration", ret);
    power_config = default_power_config;
  } else if (data[0] == kPowerConfigVersion) {
    std::memcpy(&power_config, data, sizeof(PowerConfig));
    ESP_LOGI(TAG, "Power configuration successfully loaded from NVS");
  } else {
    if (data[0] == 0) {
      // Convert legacy configuration format to current version
      power_config.version = kPowerConfigVersion;
      power_config.power_budget = data[1];
      std::memcpy(reinterpret_cast<uint8_t *>(&power_config) + 3, data + 2,
                  sizeof(PowerConfig) - 3);
    } else {
      ESP_LOGW(TAG, "Invalid power configuration version %d detected, using default configuration", data[0]);
      power_config = default_power_config;
    }
  }
}

/**
 * @brief Reset power configuration to default values
 * 
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t reset_power_config() {
  ESP_RETURN_ON_ERROR(PowerNVSEraseKey(NVSKey::POWER_CONFIG), TAG,
                      "Failed to erase power configuration from NVS");
  power_config = default_power_config;
  return ESP_OK;
}

/**
 * @brief Update power configuration with new values
 * 
 * @param config New power configuration to apply
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG on invalid configuration
 */
esp_err_t set_power_config(PowerConfig &config) {
  if (config.version != power_config.version) {
    return ESP_ERR_INVALID_ARG;
  }
  uint8_t data[sizeof(PowerConfig)];
  std::memcpy(data, &config, sizeof(PowerConfig));
  for (uint8_t i = 1; i < sizeof(PowerConfig); i++) {
    if (data[i] == 0) {
      ESP_LOGI(TAG, "Invalid power configuration detected at offset %d", i);
      return ESP_ERR_INVALID_ARG;
    }
  }
  ESP_RETURN_ON_ERROR(
      PowerNVSSet(data, NVSKey::POWER_CONFIG, sizeof(PowerConfig)), TAG,
      "Failed to save power configuration to NVS");
  power_config = config;
  return ESP_OK;
}

/**
 * @brief Retrieve current power configuration
 * 
 * @param config Pointer to store the current configuration
 * @return esp_err_t ESP_OK on success
 */
esp_err_t get_power_config(PowerConfig *config) {
  *config = power_config;
  return ESP_OK;
}

/**
 * @brief Update temperature thresholds for power management
 * 
 * @param high_temp_threshold Upper temperature threshold for power reduction
 * @param low_temp_threshold Lower temperature threshold for power restoration
 */
void set_power_temperature_thresholds(uint8_t high_temp_threshold,
                                      uint8_t low_temp_threshold) {
  power_config.device_high_temp_threshold = high_temp_threshold;
  power_config.device_low_temp_threshold = low_temp_threshold;
}
