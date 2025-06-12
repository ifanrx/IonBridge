#include "acdc.h"

#include <cstring>

#include "sdkconfig.h"

#ifndef CONFIG_ACDC_MODULE
void reset_acdc() {}
void adjust_voltage(uint8_t requester, uint16_t request_mv) {}
void request_low_voltage() {}
#else

#include <cstdint>

#include "driver/gpio.h"
#include "esp_log.h"
#include "hal/gpio_types.h"
#include "ionbridge.h"
#include "sdkconfig.h"
#include "soc/gpio_num.h"

#define FEEDBACK_PIN0 (gpio_num_t)(CONFIG_ACDC_FEEDBACK_PIN0)
#if CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS > 2
#define FEEDBACK_PIN1 (gpio_num_t)(CONFIG_ACDC_FEEDBACK_PIN1)
#endif
#define FEEDBACK_SIGNAL_H (uint8_t)(CONFIG_ACDC_FEEDBACK_SIGNAL_H)
#define FEEDBACK_SIGNAL_L (uint8_t)(CONFIG_ACDC_FEEDBACK_SIGNAL_L)
#define DEFAULT_VOLTAGE_LEVEL (CONFIG_ACDC_OUTPUT_DEFAULT_VOLTAGE_LEVEL)
#define LOW_VOLTAGE_LEVEL (0)
#define LOW_VOLTAGE (uint16_t)(CONFIG_ACDC_OUTPUT_VOLTAGE_LEVEL0 * 1e3)
#define HIGH_VOLTAGE_LEVEL (1)
#define HIGH_VOLTAGE (uint16_t)(CONFIG_ACDC_OUTPUT_VOLTAGE_LEVEL1 * 1e3)

static const char *TAG = "ACDC";

static uint16_t acdc_output_voltages[CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS] = {
    LOW_VOLTAGE,
    HIGH_VOLTAGE,
#if CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS > 2
    (uint16_t)(CONFIG_ACDC_OUTPUT_VOLTAGE_LEVEL2 * 1e3),
#endif
};
static uint8_t requester_bitmaps[CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS];
static uint8_t output_voltage_level = DEFAULT_VOLTAGE_LEVEL;

void set_output_voltage(uint8_t output_voltage_level) {
  if (output_voltage_level >= CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS) {
    ESP_LOGE(TAG, "Invalid output voltage level: %d", output_voltage_level);
    return;
  }

  uint16_t voltage = acdc_output_voltages[output_voltage_level];
  ESP_LOGI(TAG, "Setting output voltage to %dmV (level %d)", voltage,
           output_voltage_level);

  // Set FEEDBACK_PIN0 based on the LSB (bit 0) of output_voltage_level
  gpio_set_level(FEEDBACK_PIN0, (output_voltage_level & 0x01)
                                    ? FEEDBACK_SIGNAL_H
                                    : FEEDBACK_SIGNAL_L);

#if CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS > 2
  // Set FEEDBACK_PIN1 based on the second LSB (bit 1) of output_voltage_level
  // This pin is only defined and used if there are more than 2 voltage levels
  gpio_set_level(FEEDBACK_PIN1, (output_voltage_level & 0x02)
                                    ? FEEDBACK_SIGNAL_H
                                    : FEEDBACK_SIGNAL_L);
#endif
}

void reset_acdc() {
  memset(requester_bitmaps, 0, sizeof(requester_bitmaps));
  output_voltage_level = DEFAULT_VOLTAGE_LEVEL;
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << FEEDBACK_PIN0),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
#ifdef FEEDBACK_PIN1
  io_conf.pin_bit_mask |= (1ULL << FEEDBACK_PIN1);
#endif
  ESP_ERROR_COMPLAIN(gpio_config(&io_conf), "Failed to configure GPIO: %d",
                     (int)FEEDBACK_PIN0);
  set_output_voltage(output_voltage_level);
}

/**
 * @brief Adjusts output voltage based on requester needs.
 *
 * @param requester   Component identifier (bit position)
 * @param request_mv  Requested minimum voltage in millivolts
 *
 * Maps the requester to the lowest suitable voltage level and
 * sets output to the highest level needed by any active requester.
 */
void adjust_voltage(uint8_t requester, uint16_t request_mv) {
  if (requester >= 8) {
    ESP_LOGE(TAG, "Invalid requester: %d", requester);
    return;
  }

  // Find lowest voltage level that satisfies request
  uint8_t request_level =
      CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS - 1;  // Default to highest
  for (uint8_t i = 0; i < CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS; i++) {
    if (request_mv <= acdc_output_voltages[i]) {
      request_level = i;
      break;
    }
  }

  // Update bitmaps - clear requester bit from all maps, then set in target map
  for (uint8_t i = 0; i < CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS; i++) {
    if (i == request_level) {
      requester_bitmaps[i] |= (1U << requester);
    } else {
      requester_bitmaps[i] &= ~(1U << requester);
    }
  }

  // Find highest level with any requesters
  for (int8_t i = CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS - 1; i >= 0; i--) {
    if (requester_bitmaps[i] == 0) {
      // No requesters at this level, skip
      continue;
    }
    // Only update if level would change
    if (i != output_voltage_level) {
      output_voltage_level = i;
      set_output_voltage(output_voltage_level);
    }
    return;
  }
}

void request_low_voltage() {
  if (output_voltage_level > LOW_VOLTAGE_LEVEL) {
    memset(requester_bitmaps, 0, sizeof(requester_bitmaps));
    output_voltage_level = LOW_VOLTAGE_LEVEL;
    set_output_voltage(LOW_VOLTAGE_LEVEL);
  }
}
#endif
