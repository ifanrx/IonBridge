#include "fpga_display_driver.h"

#include "esp_err.h"
#include "rpc.h"

esp_err_t FPGADisplayDriver::Init() { return ESP_OK; }

esp_err_t FPGADisplayDriver::Deinit() { return ESP_OK; }

esp_err_t FPGADisplayDriver::SetMode(uint8_t mode) {
  if (mode == current_mode_)
    return ESP_OK;  // Idempotent - skip RPC if no change
  esp_err_t err = rpc::fpga::set_display_mode(mode);
  if (err == ESP_OK) {
    current_mode_ = mode;
  }
  return err;
}

esp_err_t FPGADisplayDriver::SetIntensity(uint8_t intensity) {
  // Convert 0-100 percentage to 0-128 hardware range
  return rpc::fpga::set_display_intensity(intensity * 128 / 100);
}

esp_err_t FPGADisplayDriver::SetPercentage(uint8_t percentage) {
  SetMode(DisplayMode::MANUAL);  // Percentage display requires MANUAL mode
  return rpc::fpga::set_display_percentage(percentage);
}

esp_err_t FPGADisplayDriver::SetRotation(uint8_t rotation) {
  // FPGA only supports 0° (normal) and 180° (flipped)
  // Translate rotation to flip mode for hardware
  if (rotation == 0) {
    return rpc::fpga::set_display_flip_mode(0);  // Normal
  } else if (rotation == 180) {
    return rpc::fpga::set_display_flip_mode(1);  // Flipped
  } else {
    return ESP_ERR_NOT_SUPPORTED;  // FPGA can't do 90/270
  }
}

esp_err_t FPGADisplayDriver::EnableAutoPowerDisplay() {
  esp_err_t err = rpc::fpga::set_display_mode(DisplayMode::POWER_METER);
  if (err == ESP_OK) {
    current_mode_ = DisplayMode::POWER_METER;
  }
  return err;
}

void FPGADisplayDriver::Clear() {
  // Only clear in MANUAL mode; POWER_METER is hardware-controlled
  if (current_mode_ == DisplayMode::MANUAL) {
    rpc::fpga::set_display_percentage(0);
  }
}
