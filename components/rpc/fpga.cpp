#include <cstdint>
#include <cstring>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "fpga_data_types.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/projdefs.h"
#include "freertos/semphr.h"
#include "ionbridge.h"
#include "rpc.h"
#include "sdkconfig.h"
#include "uart.h"

#define MCU_TO_PORT(mcu) (uint8_t)(mcu + 1)
#define LOCK_TIMEOUT_MS (CONFIG_UART_MESSAGE_RECV_TIMEOUT_MS + 50)

static const char *TAG = "FPGARpc";
struct {
  SemaphoreHandle_t lock;
  FPGARequest req;
} rpc_lock = {
    .lock = xSemaphoreCreateMutex(),
    .req = {},
};

esp_err_t send_to_fpga(const FPGAOperation &op, const FPGARequest &req,
                       FPGAResponse *res, size_t *res_size) {
  if (res == nullptr) {
    ESP_LOGE(TAG, "Response pointer is null");
    return ESP_ERR_INVALID_ARG;
  }

  ESP_RETURN_ON_FALSE(
      xSemaphoreTake(rpc_lock.lock, pdMS_TO_TICKS(LOCK_TIMEOUT_MS)),
      ESP_ERR_TIMEOUT, TAG, "Failed to take lock, token by (0x%02X, 0x%02X)",
      rpc_lock.req.addr, rpc_lock.req.value);
  rpc_lock.req = req;
  esp_err_t ret;
  ret = uart_send(FPGA_ADDR, op, (const uint8_t *)&req, sizeof(req));
  if (ret == ESP_OK) {
    ret = uart_read(FPGA_ADDR, op, (uint8_t *)res, (uint8_t *)res_size);
  }
  memset(&rpc_lock.req, 0, sizeof(rpc_lock.req));  // Clear request after use
  xSemaphoreGive(rpc_lock.lock);
  ESP_ERROR_COMPLAIN(ret, "failed to send 0x%04X op to FPGA", op);
  return ret;
}

esp_err_t read_fpga_reg(FPGARegAddr addr, uint8_t *value) {
  FPGARequest req{.addr = addr, .value = 0};  // value is always 0
  FPGAResponse res{};
  size_t res_size = sizeof(res);
  ESP_RETURN_ON_ERROR(
      send_to_fpga(FPGAOperation::READ_REG, req, &res, &res_size), TAG,
      "Failed to read FPGA register 0x%04X", addr);
  if (res.addr != addr) {
    ESP_LOGE(TAG, "Invalid response addr: 0x%04X, expected: 0x%04X", res.addr,
             addr);
    return ESP_ERR_INVALID_MAC;
  }
  if (value != nullptr) {
    *value = res.data;
  }
  return ESP_OK;
}

esp_err_t write_fpga_reg(FPGARegAddr addr, uint8_t value) {
  FPGARequest req{.addr = addr, .value = value};
  FPGAResponse res{};
  size_t res_size = sizeof(res);
  ESP_RETURN_ON_ERROR(
      send_to_fpga(FPGAOperation::WRITE_REG, req, &res, &res_size), TAG,
      "Failed to write FPGA register 0x%04X", addr);
  if (res.addr != addr) {
    return ESP_ERR_INVALID_MAC;
  }
  if (res.data != value) {
    return ESP_ERR_INVALID_RESPONSE;
  }
  return ESP_OK;
}

esp_err_t rpc::fpga::set_display_percentage(uint8_t percentage) {
  return write_fpga_reg(FPGARegAddr::DISPLAY_PERCENTAGE, percentage);
}

esp_err_t rpc::fpga::set_display_mode(uint8_t mode) {
  ESP_LOGD(TAG, "Setting display mode to %d", mode);
  return write_fpga_reg(FPGARegAddr::DISPLAY_MODE, mode);
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
  return write_fpga_reg(FPGARegAddr::DISPLAY_FLIP, mode);
}

esp_err_t rpc::fpga::set_action_deadzone(uint8_t deadzone) {
  return write_fpga_reg(FPGARegAddr::ACTION_DEADZONE, deadzone);
}
