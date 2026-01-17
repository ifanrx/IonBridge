#include "acdc.h"

#include "sdkconfig.h"

#ifndef CONFIG_ACDC_MODULE
void reset_acdc() {}
void request_highest_voltage() {}
void voltage_control(uint32_t, int16_t, bool) {}
void acdc_self_test() {}
ACDCSelfTestStatus acdc_get_self_test_status() {
  return ACDC_SELF_TEST_NOT_RUN;
}
esp_err_t acdc_get_voltage_range(uint8_t, uint16_t *, uint16_t *) {
  return ESP_ERR_NOT_SUPPORTED;
}
esp_err_t acdc_detect_voltage_level(uint16_t, uint8_t *) {
  return ESP_ERR_NOT_SUPPORTED;
}
bool acdc_voltage_in_range(uint8_t, uint16_t) { return false; }
#ifdef CONFIG_ENABLE_FACTORY_MODE
void factory_set_fb_pattern(uint8_t pattern) {}
#endif
#else

#include <cinttypes>
#include <cstdint>

#include "driver/gpio.h"
#include "esp_log.h"
#include "hal/gpio_types.h"
#include "ionbridge.h"
#include "port_manager.h"
#include "sdkconfig.h"
#include "soc/gpio_num.h"
#include "soc/gpio_struct.h"
#include "utils.h"

#if defined(CONFIG_INTERFACE_TYPE_UART)
#include "esp_check.h"

#if defined(CONFIG_ENABLE_FPGA_POWER_CTRL)
#include "fpga_data_types.h"

#define ENABLE_FPGA_POWER_CTRL
#endif
#endif

#define FEEDBACK_PIN0 (gpio_num_t)(CONFIG_ACDC_FEEDBACK_PIN0)
#if CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS > 2
#define FEEDBACK_PIN1 (gpio_num_t)(CONFIG_ACDC_FEEDBACK_PIN1)
#endif
#define FEEDBACK_SIGNAL_H (uint8_t)(CONFIG_ACDC_FEEDBACK_SIGNAL_H)
#define FEEDBACK_SIGNAL_L (uint8_t)(CONFIG_ACDC_FEEDBACK_SIGNAL_L)

// Voltage control thresholds
#define VOLTAGE_HEADROOM_MV 1000    // Buck converter safety margin (1V)
#define POWER_THRESHOLD_PERCENT 95  // Step down threshold (95% capacity)
#define STEP_DOWN_DELAY_CYCLES 50   // 50 × 200ms = 10 seconds

// Macro to initialize voltage config (voltage levels are in mV)
#define INIT_VOLTAGE_CFG(lv)                                    \
  {.voltage = (uint16_t)(CONFIG_ACDC_OUTPUT_VOLTAGE_LEVEL##lv), \
   .max_current_ma = (CONFIG_ACDC_CURRENT_MAX_LEVEL##lv)}

static const char *TAG = "ACDC";

// Self-test configuration
#define ACDC_SELF_TEST_STABILIZE_MS 400  // Wait time after voltage change

static ACDCSelfTestStatus self_test_status = ACDC_SELF_TEST_NOT_RUN;
static bool self_test_in_progress = false;

typedef struct {
  uint16_t voltage;         // Voltage level in mV
  uint16_t max_current_ma;  // Max current capacity in mA
} OutputVoltageConfig;

typedef struct {
  uint16_t lower_mv;  // How far below expected is acceptable
  uint16_t upper_mv;  // How far above expected is acceptable
} VoltageTolerance;

// Macro to initialize voltage tolerance (for self-test validation)
#define INIT_VOLTAGE_TOLERANCE(lv)                                    \
  {.lower_mv = (uint16_t)(CONFIG_ACDC_VOLTAGE_TOLERANCE_LOWER_L##lv), \
   .upper_mv = (uint16_t)(CONFIG_ACDC_VOLTAGE_TOLERANCE_UPPER_L##lv)}

static uint8_t current_voltage_level =
    0xFF;  // 0xFF = uninitialized, forces full pin update
static uint8_t step_down_delay_counter = 0;
static OutputVoltageConfig OUTPUT_VOLT_CFGS[CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS] =
    {
#if CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS >= 1
        INIT_VOLTAGE_CFG(0),
#endif
#if CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS >= 2
        INIT_VOLTAGE_CFG(1),
#endif
#if CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS >= 3
        INIT_VOLTAGE_CFG(2),
#endif
#if CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS >= 4
        INIT_VOLTAGE_CFG(3),
#endif
};

static const VoltageTolerance
    VOLTAGE_TOLERANCES[CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS] = {
#if CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS >= 1
        INIT_VOLTAGE_TOLERANCE(0),
#endif
#if CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS >= 2
        INIT_VOLTAGE_TOLERANCE(1),
#endif
#if CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS >= 3
        INIT_VOLTAGE_TOLERANCE(2),
#endif
#if CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS >= 4
        INIT_VOLTAGE_TOLERANCE(3),
#endif
};

// Compile-time validation: voltage levels must be non-decreasing
// (equal levels are valid - the step-down logic skips identical levels)
#if CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS >= 2
static_assert(CONFIG_ACDC_OUTPUT_VOLTAGE_LEVEL0 <=
                  CONFIG_ACDC_OUTPUT_VOLTAGE_LEVEL1,
              "Voltage levels must be non-decreasing: L0 <= L1");
#endif
#if CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS >= 3
static_assert(CONFIG_ACDC_OUTPUT_VOLTAGE_LEVEL1 <=
                  CONFIG_ACDC_OUTPUT_VOLTAGE_LEVEL2,
              "Voltage levels must be non-decreasing: L1 <= L2");
#endif
#if CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS >= 4
static_assert(CONFIG_ACDC_OUTPUT_VOLTAGE_LEVEL2 <=
                  CONFIG_ACDC_OUTPUT_VOLTAGE_LEVEL3,
              "Voltage levels must be non-decreasing: L2 <= L3");
#endif

#define GET_VOLTAGE_CONFIG_SAFE(lv) \
  ((lv) < CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS ? &OUTPUT_VOLT_CFGS[lv] : NULL)

// Helper functions
void set_output_voltage(uint8_t output_voltage_level) {
  const OutputVoltageConfig *volt_cfg =
      GET_VOLTAGE_CONFIG_SAFE(output_voltage_level);
  ESP_RETURN_VOID_ON_FALSE(volt_cfg != nullptr, ESP_ERR_INVALID_ARG, TAG,
                           "Invalid output voltage level: %d",
                           output_voltage_level);

  // Only toggle FB pins that actually change (avoid glitches)
  // Force full update when uninitialized (0xFF)
  uint8_t changed = (current_voltage_level == 0xFF)
                        ? 0xFF
                        : current_voltage_level ^ output_voltage_level;

  if (changed == 0) {
    return;
  }

  ESP_LOGI(TAG, "Setting output voltage to %dmV (level %d)", volt_cfg->voltage,
           output_voltage_level);

  // Use atomic GPIO register writes to avoid transient states
  // GPIO.out_w1ts sets pins HIGH, GPIO.out_w1tc sets pins LOW
  uint32_t set_mask = 0;
  uint32_t clear_mask = 0;

  if (changed & 0x01) {
    if ((output_voltage_level & 0x01) ? FEEDBACK_SIGNAL_H : FEEDBACK_SIGNAL_L) {
      set_mask |= (1ULL << FEEDBACK_PIN0);
    } else {
      clear_mask |= (1ULL << FEEDBACK_PIN0);
    }
  }
#if CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS > 2
  if (changed & 0x02) {
    if ((output_voltage_level & 0x02) ? FEEDBACK_SIGNAL_H : FEEDBACK_SIGNAL_L) {
      set_mask |= (1ULL << FEEDBACK_PIN1);
    } else {
      clear_mask |= (1ULL << FEEDBACK_PIN1);
    }
  }
#endif

  // Set GPIO pins FIRST before FPGA control to avoid voltage transients
  // Apply atomically - single write when all pins go same direction
  if (set_mask && !clear_mask) {
    GPIO.out_w1ts.val = set_mask;
  } else if (clear_mask && !set_mask) {
    GPIO.out_w1tc.val = clear_mask;
  } else if (set_mask || clear_mask) {
    // Mixed: clear first (lower voltage), then set
    GPIO.out_w1tc.val = clear_mask;
    GPIO.out_w1ts.val = set_mask;
  }
  current_voltage_level = output_voltage_level;

#ifdef ENABLE_FPGA_POWER_CTRL
  // Update FPGA config AFTER GPIO pins are set
  auto cfg = FPGAADCConfig::Create(output_voltage_level);
  ESP_RETURN_VOID_ON_FALSE(cfg != nullptr, ESP_ERR_NO_MEM, TAG,
                           "Failed to create FPGAADCConfig for level %d",
                           output_voltage_level);
  ESP_RETURN_VOID_ON_ERROR(cfg->PauseControl(), TAG,
                           "Failed to disable FPGA power control");

  if (output_voltage_level > 0) {
    // Re-enable FPGA power control for non-lowest voltage levels
    ESP_RETURN_VOID_ON_ERROR(cfg->LoadAndApply(), TAG,
                             "Failed to apply FPGAADCConfig for level %d",
                             output_voltage_level);
  } else {
    // At lowest voltage level, only load config but don't resume control
    ESP_RETURN_VOID_ON_ERROR(cfg->Load(), TAG,
                             "Failed to load FPGAADCConfig for level %d",
                             output_voltage_level);
  }
#endif
}

void reset_acdc() {
  uint8_t highest_level = CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS - 1;

  // Configure GPIO as output first
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << FEEDBACK_PIN0),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
#ifdef FEEDBACK_PIN1
  io_conf.pin_bit_mask |= (1ULL << FEEDBACK_PIN1);
#endif
  ESP_ERROR_COMPLAIN(gpio_config(&io_conf), "Failed to configure GPIO: %d",
                     (int)FEEDBACK_PIN0);

  // Set to highest level immediately after config
  current_voltage_level = 0xFF;  // Force full update
  set_output_voltage(highest_level);

  ESP_LOGI(TAG, "ACDC initialized at level %d (%dmV)", highest_level,
           OUTPUT_VOLT_CFGS[highest_level].voltage);
}

/**
 * @brief Request the highest available voltage level
 *
 * Sets the output voltage to the highest configured level.
 * Used for maximum power delivery scenarios or testing.
 */
void request_highest_voltage(void) {
#ifdef CONFIG_ACDC_OUTPUT_ADJUSTABLE
  uint8_t target_level = CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS - 1;

  if (current_voltage_level == target_level) {
    ESP_LOGD(TAG, "Already at highest voltage level %d (%dmV)", target_level,
             OUTPUT_VOLT_CFGS[target_level].voltage);
    return;
  }

  ESP_LOGI(TAG, "Setting to highest voltage: Level %d -> Level %d (%dmV)",
           current_voltage_level, target_level,
           OUTPUT_VOLT_CFGS[target_level].voltage);

  set_output_voltage(target_level);
#endif  // CONFIG_ACDC_OUTPUT_ADJUSTABLE
}

#ifdef CONFIG_ACDC_CURRENT_VOLTAGE_CONTROL
// Step up when device voltage requirement exceeds current ACDC level
static bool try_step_up_voltage(int16_t max_output_voltage_mv) {
  const OutputVoltageConfig *cfg =
      GET_VOLTAGE_CONFIG_SAFE(current_voltage_level);
  if (!cfg || max_output_voltage_mv <= cfg->voltage) {
    return false;
  }

  // Find level that can provide required voltage
  uint8_t target_level = current_voltage_level;
  for (uint8_t i = current_voltage_level + 1;
       i < CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS; i++) {
    if (OUTPUT_VOLT_CFGS[i].voltage >= max_output_voltage_mv) {
      target_level = i;
      break;
    }
  }

  if (target_level == current_voltage_level) {
    return false;  // No higher level available
  }

  // Execute voltage-based step-up
  step_down_delay_counter = 0;  // Reset delay on step-up
  ESP_LOGI(TAG,
           "Stepping up: %d->%d (%" PRIu16 "mV->%" PRIu16
           "mV), device requires %" PRId16 "mV",
           current_voltage_level, target_level,
           OUTPUT_VOLT_CFGS[current_voltage_level].voltage,
           OUTPUT_VOLT_CFGS[target_level].voltage, max_output_voltage_mv);
  set_output_voltage(target_level);
  return true;
}

// Step up when power consumption exceeds current level capacity
static bool try_step_up_power(uint32_t total_power_uw) {
  const OutputVoltageConfig *cfg =
      GET_VOLTAGE_CONFIG_SAFE(current_voltage_level);
  if (!cfg) return false;

  uint32_t level_power = (uint32_t)cfg->voltage * cfg->max_current_ma;
  uint32_t port_power = total_power_uw;

  if (port_power < level_power) {
    return false;
  }

  ESP_LOGI(TAG,
           "Power exceeds capacity: level=%d (%" PRIu16
           "mV), port_power=%" PRIu32 "mW >= level_capacity=%" PRIu32 "mW",
           current_voltage_level, cfg->voltage, port_power / 1000,
           level_power / 1000);

  // Find level with higher capacity
  uint8_t target_level = current_voltage_level;
  for (uint8_t i = current_voltage_level + 1;
       i < CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS; i++) {
    const OutputVoltageConfig *next_cfg = &OUTPUT_VOLT_CFGS[i];
    uint32_t next_power =
        (uint32_t)next_cfg->voltage * next_cfg->max_current_ma;

    if (next_power > level_power) {
      target_level = i;
      break;
    }
  }

  if (target_level == current_voltage_level) {
    ESP_LOGI(TAG, "Step-up check: no higher level available");
    return false;
  }

  // Execute power-based step-up
  step_down_delay_counter = 0;  // Reset delay on step-up
  ESP_LOGI(TAG,
           "Stepping up: %d->%d (%" PRIu16 "mV->%" PRIu16
           "mV), port_power=%" PRIu32 "mW > level_capacity=%" PRIu32 "mW",
           current_voltage_level, target_level,
           OUTPUT_VOLT_CFGS[current_voltage_level].voltage,
           OUTPUT_VOLT_CFGS[target_level].voltage, port_power / 1000,
           level_power / 1000);
  set_output_voltage(target_level);
  return true;
}

// Step down when both voltage and power headroom exist
static void try_step_down(uint32_t total_power_uw,
                          int16_t max_output_voltage_mv, bool has_pps_avs) {
  const OutputVoltageConfig *cfg =
      GET_VOLTAGE_CONFIG_SAFE(current_voltage_level);
  if (!cfg) return;

  // max_output_voltage_mv semantics:
  //   -1: no ports attached (allow step down to save heat)
  //    0: ports attached but negotiating (block step down)
  //   >0: actual voltage requirement (normal logic)
  if (max_output_voltage_mv == 0) {
    step_down_delay_counter = 0;  // Reset delay when blocked
    return;
  }

  // Delay step-down to let voltage stabilize after changes
  if (step_down_delay_counter < STEP_DOWN_DELAY_CYCLES) {
    step_down_delay_counter++;
    return;
  }

  // Find next level with actually lower voltage (skip identical levels)
  uint8_t target_level = current_voltage_level;
  for (int8_t i = current_voltage_level - 1; i >= 0; i--) {
    if (OUTPUT_VOLT_CFGS[i].voltage < cfg->voltage) {
      target_level = i;
      break;
    }
  }

  if (target_level == current_voltage_level) return;  // No lower level found

  if (target_level == 0 && has_pps_avs)
    return;  // Don't step to level 0 with PPS/AVS

  const OutputVoltageConfig *lower_cfg = &OUTPUT_VOLT_CFGS[target_level];

  // Voltage headroom: lower level must support port output + safety margin
  if (lower_cfg->voltage < max_output_voltage_mv + VOLTAGE_HEADROOM_MV) {
    return;
  }

  // Power headroom: port power must be below threshold of lower tier's capacity
  uint32_t lower_power =
      (uint32_t)lower_cfg->voltage * lower_cfg->max_current_ma;
  uint32_t port_power = total_power_uw;
  uint32_t threshold = lower_power * POWER_THRESHOLD_PERCENT / 100;

  if (port_power < threshold) {
    step_down_delay_counter = 0;  // Reset delay after step-down
    ESP_LOGI(TAG, "Stepping down: %d->%d (%" PRIu16 "mV->%" PRIu16 "mV)",
             current_voltage_level, target_level,
             OUTPUT_VOLT_CFGS[current_voltage_level].voltage,
             OUTPUT_VOLT_CFGS[target_level].voltage);
    set_output_voltage(target_level);
  }
}
#endif  // CONFIG_ACDC_CURRENT_VOLTAGE_CONTROL

void voltage_control(uint32_t total_power_uw, int16_t max_output_voltage_mv,
                     bool has_pps_avs) {
#ifdef CONFIG_ACDC_CURRENT_VOLTAGE_CONTROL
  if (self_test_in_progress) return;

  // Try voltage-based step-up first (device needs higher voltage)
  if (try_step_up_voltage(max_output_voltage_mv)) return;

  // Try power-based step-up (consumption exceeds capacity)
  if (try_step_up_power(total_power_uw)) return;

  // Skip step-down until self-test has run
  if (self_test_status == ACDC_SELF_TEST_NOT_RUN) return;

  try_step_down(total_power_uw, max_output_voltage_mv, has_pps_avs);
#else
  (void)total_power_uw;
  (void)max_output_voltage_mv;
  (void)has_pps_avs;
#endif
}

#ifdef CONFIG_ENABLE_FACTORY_MODE
void factory_set_fb_pattern(uint8_t pattern) {
  if (pattern > 0x03) {
    ESP_LOGE(TAG, "Invalid FB pattern: 0x%02X (must be 0-3)", pattern);
    return;
  }

  gpio_set_level(FEEDBACK_PIN0,
                 (pattern & 0x01) ? FEEDBACK_SIGNAL_H : FEEDBACK_SIGNAL_L);
#if CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS > 2
  gpio_set_level(FEEDBACK_PIN1,
                 (pattern & 0x02) ? FEEDBACK_SIGNAL_H : FEEDBACK_SIGNAL_L);
#endif

  ESP_LOGI(TAG, "Factory FB pattern set: 0x%02X (FB0=%d, FB1=%d)", pattern,
           (pattern & 0x01), (pattern >> 1) & 0x01);
}
#endif

ACDCSelfTestStatus acdc_get_self_test_status(void) { return self_test_status; }

esp_err_t acdc_get_voltage_range(uint8_t level, uint16_t *min_mv,
                                 uint16_t *max_mv) {
  if (level >= CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS) {
    return ESP_ERR_INVALID_ARG;
  }
  uint16_t expected = OUTPUT_VOLT_CFGS[level].voltage;
  const VoltageTolerance &tol = VOLTAGE_TOLERANCES[level];
  if (min_mv) *min_mv = expected - tol.lower_mv;
  if (max_mv) *max_mv = expected + tol.upper_mv;
  return ESP_OK;
}

esp_err_t acdc_detect_voltage_level(uint16_t measured_mv,
                                    uint8_t *detected_level) {
  if (!detected_level) {
    return ESP_ERR_INVALID_ARG;
  }
  for (uint8_t lv = 0; lv < CONFIG_ACDC_OUTPUT_VOLTAGE_LEVELS; lv++) {
    if (acdc_voltage_in_range(lv, measured_mv)) {
      *detected_level = lv;
      return ESP_OK;
    }
  }
  *detected_level = 0xFF;
  return ESP_ERR_NOT_FOUND;
}

bool acdc_voltage_in_range(uint8_t level, uint16_t measured_mv) {
  uint16_t min_mv, max_mv;
  if (acdc_get_voltage_range(level, &min_mv, &max_mv) != ESP_OK) {
    return false;
  }
  return measured_mv >= min_mv && measured_mv <= max_mv;
}

void acdc_self_test(void) {
#ifndef CONFIG_ACDC_OUTPUT_ADJUSTABLE
  return;
#endif

  ESP_LOGI(TAG, "Starting ACDC self-test");
  self_test_in_progress = true;

  PortManager &pm = PortManager::GetInstance();
  Port *port = nullptr;
  for (Port &p : pm.GetAlivePorts()) {
    if (p.Activated()) {
      port = &p;
      break;
    }
  }

  if (!port) {
    ESP_LOGE(TAG, "No activated port for self-test");
    self_test_status = ACDC_SELF_TEST_NO_PORT;
    return;
  }

  ESP_LOGI(TAG, "Using port %d for voltage measurement", port->Id());

  // Test sequence: 11 -> 10 -> 00 (single-bit changes to avoid glitches)
  const uint8_t test_levels[] = {3, 2, 0};
  for (uint8_t i = 0; i < sizeof(test_levels); i++) {
    uint8_t lv = test_levels[i];
    set_output_voltage(lv);
    DELAY_MS(ACDC_SELF_TEST_STABILIZE_MS);

    PortDetails details;
    if (port->FetchDetails(&details) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to fetch port details");
      self_test_status = ACDC_SELF_TEST_NO_PORT;
      goto cleanup;
    }

    uint16_t expected = OUTPUT_VOLT_CFGS[lv].voltage;
    uint16_t actual = details.vin_value;

    if (!acdc_voltage_in_range(lv, actual)) {
      uint16_t min_mv, max_mv;
      acdc_get_voltage_range(lv, &min_mv, &max_mv);
      ESP_LOGE(TAG,
               "Level %d (FB=%d%d): expected %dmV, got %dmV (valid: %d-%dmV)",
               lv, (lv >> 1) & 1, lv & 1, expected, actual, min_mv, max_mv);
      self_test_status =
          static_cast<ACDCSelfTestStatus>(ACDC_SELF_TEST_FAIL_L0 + lv);
      goto cleanup;
    }
    ESP_LOGI(TAG, "Level %d (FB=%d%d): OK (%dmV)", lv, (lv >> 1) & 1, lv & 1,
             actual);
  }

  self_test_status = ACDC_SELF_TEST_PASSED;
  ESP_LOGI(TAG, "ACDC self-test passed");

cleanup:
  self_test_in_progress = false;
  request_highest_voltage();
}
#endif
