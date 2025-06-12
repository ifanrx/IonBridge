#ifndef POWER_CONFIG_H_
#define POWER_CONFIG_H_

#include <stdint.h>

#include "esp_err.h"

constexpr uint8_t kPowerConfigVersion = 1;

typedef struct __attribute__((packed)) {
  uint8_t version;
  uint16_t power_budget;

  uint8_t device_high_temp_threshold;
  uint8_t device_high_temp_min_power;
  uint8_t device_low_temp_threshold;
  uint8_t device_over_temp_decrement;
  uint8_t device_under_temp_increment;
  uint8_t device_power_cooldown_time;

  uint8_t overprovisioning_threshold;
  uint8_t cap_increase_min_threshold;
  uint8_t cap_decrease_min_threshold;
  uint8_t port_power_cap_increase_threshold;
  uint8_t port_power_cap_increase_step;
  uint8_t port_power_cap_decrease_threshold;
  uint8_t high_power_usage_threshold;
  uint8_t low_power_usage_threshold;
  uint8_t port_power_adjustment_threshold;
} PowerConfig;

extern PowerConfig power_config;

void init_power_config();

esp_err_t reset_power_config();

esp_err_t set_power_config(PowerConfig &config);
esp_err_t get_power_config(PowerConfig *config);

void set_power_temperature_thresholds(uint8_t high_temp_threshold,
                                      uint8_t low_temp_threshold);
#endif /* POWER_CONFIG_H_ */
