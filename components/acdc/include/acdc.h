#ifndef ACDC_H
#define ACDC_H

#include <cstdint>

#include "esp_err.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

void reset_acdc(void);
void request_highest_voltage();
void voltage_control(uint32_t total_power_uw, int16_t max_output_voltage_mv,
                     bool has_pps_avs);

// ACDC self-test status (single enum encodes both result and failed level)
typedef enum : uint8_t {
  ACDC_SELF_TEST_NOT_RUN = 0,  // Test hasn't executed yet
  ACDC_SELF_TEST_PASSED = 1,   // All voltage levels passed
  ACDC_SELF_TEST_NO_PORT = 2,  // No activated port for measurement
  // Voltage errors: 0x10 + failed_level (0-3)
  ACDC_SELF_TEST_FAIL_L0 = 0x10,
  ACDC_SELF_TEST_FAIL_L1 = 0x11,
  ACDC_SELF_TEST_FAIL_L2 = 0x12,
  ACDC_SELF_TEST_FAIL_L3 = 0x13,
} ACDCSelfTestStatus;

#define ACDC_TEST_IS_VOLT_ERR(s) ((s) >= ACDC_SELF_TEST_FAIL_L0)
#define ACDC_TEST_FAILED_LEVEL(s) ((s) - ACDC_SELF_TEST_FAIL_L0)

void acdc_self_test(void);
ACDCSelfTestStatus acdc_get_self_test_status(void);

/**
 * @brief Get valid voltage range for a specific level
 * @param level Output voltage level (0-3)
 * @param[out] min_mv Minimum acceptable voltage in mV (NULL if not needed)
 * @param[out] max_mv Maximum acceptable voltage in mV (NULL if not needed)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if level out of range
 */
esp_err_t acdc_get_voltage_range(uint8_t level, uint16_t *min_mv,
                                 uint16_t *max_mv);

/**
 * @brief Detect voltage level from measured voltage
 * @param measured_mv Measured voltage in mV
 * @param[out] detected_level Detected level (0-3), 0xFF if no match
 * @return ESP_OK if level detected, ESP_ERR_NOT_FOUND if no match
 */
esp_err_t acdc_detect_voltage_level(uint16_t measured_mv,
                                    uint8_t *detected_level);

/**
 * @brief Check if measured voltage is within valid range for level
 * @param level Output voltage level (0-3)
 * @param measured_mv Measured voltage in mV
 * @return true if within range, false otherwise
 */
bool acdc_voltage_in_range(uint8_t level, uint16_t measured_mv);

#ifdef __cplusplus
}
#endif

#endif  // ACDC_H
