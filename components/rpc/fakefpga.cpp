#include "display_types.h"
#include "esp_check.h"
#include "esp_log.h"
#include "fpga_data_types.h"
#include "rpc.h"

#if CONFIG_MCU_MODEL_FAKE_SW3566
#define FPGA_ADDR 0xFE
#define MCU_TO_PORT(mcu) (uint8_t)(mcu + 1)

static const char *TAG = "FPGARpc";

esp_err_t read_fpga_reg(FPGARegAddr addr, uint8_t *value) {
  if (value != nullptr) {
    *value = 0;
  }
  return ESP_OK;
}

esp_err_t write_fpga_reg(FPGARegAddr addr, uint8_t value) { return ESP_OK; }

esp_err_t rpc::fpga::set_display_percentage(uint8_t percentage) {
  return write_fpga_reg(FPGARegAddr::DISPLAY_PERCENTAGE, percentage);
}

esp_err_t rpc::fpga::set_display_mode(uint8_t mode) {
  switch (mode) {
    case DisplayMode::OFF:
    case DisplayMode::MANUAL:
    case DisplayMode::POWER_METER:
      return write_fpga_reg(FPGARegAddr::DISPLAY_MODE, mode);
    default:
      ESP_LOGE(TAG, "Invalid display mode %d", mode);
  }
  return ESP_ERR_INVALID_ARG;
}

esp_err_t rpc::fpga::set_display_intensity(uint8_t intensity) {
  return write_fpga_reg(FPGARegAddr::DISPLAY_INTENSITY, intensity);
}

esp_err_t rpc::fpga::toggle_power_control(bool enable) {
  return write_fpga_reg(FPGARegAddr::ENABLE_POWER_CONTROL, enable ? 1 : 0);
}

esp_err_t rpc::fpga::set_mcu_priority(uint8_t mcu, uint8_t priority) {
  FPGARegAddr addr;
  switch (priority) {
    case 1:
      addr = FPGARegAddr::P1_PRIORITY;
      break;
    case 2:
      addr = FPGARegAddr::P2_PRIORITY;
      break;
    case 3:
      addr = FPGARegAddr::P3_PRIORITY;
      break;
    case 4:
      addr = FPGARegAddr::P4_PRIORITY;
      break;
    case 5:
      addr = FPGARegAddr::P5_PRIORITY;
      break;
    default:
      return ESP_ERR_INVALID_ARG;
  }

  return write_fpga_reg(addr, MCU_TO_PORT(mcu));
}

esp_err_t rpc::fpga::read_adc_value(uint8_t *value) {
  return read_fpga_reg(FPGARegAddr::ADC_VALUE, value);
}

esp_err_t rpc::fpga::toggle_mcu_gpio(uint8_t mcu) {
  return write_fpga_reg(FPGARegAddr::TOGGLE_SW3566_IO2, MCU_TO_PORT(mcu));
}

esp_err_t rpc::fpga::set_adc_threshold(uint8_t low, uint8_t high) {
  if (low > high) {
    return ESP_ERR_INVALID_ARG;
  }
  ESP_RETURN_ON_ERROR(write_fpga_reg(FPGARegAddr::ADC_THRESHOLD_LOW, low), TAG,
                      "write_fpga_reg: ADC_THRESHOLD_LOW=%d", low);
  ESP_RETURN_ON_ERROR(write_fpga_reg(FPGARegAddr::ADC_THRESHOLD_HIGH, high),
                      TAG, "write_fpga_reg: ADC_THRESHOLD_HIGH=%d", high);
  return ESP_OK;
}

esp_err_t rpc::fpga::set_display_flip_mode(uint8_t mode) {
  switch (mode) {
    case DisplayFlipMode::NORMAL:
    case DisplayFlipMode::FLIP:
      return write_fpga_reg(FPGARegAddr::DISPLAY_FLIP, mode);
    default:
      ESP_LOGE(TAG, "Invalid display filp mode: %d", mode);
  }
  return ESP_ERR_INVALID_ARG;
}

esp_err_t rpc::fpga::set_action_deadzone(uint8_t deadzone) {
  return write_fpga_reg(FPGARegAddr::ACTION_DEADZONE, deadzone);
}

#endif
