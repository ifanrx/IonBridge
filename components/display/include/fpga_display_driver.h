#ifndef H_FPGA_DISPLAY_DRIVER_
#define H_FPGA_DISPLAY_DRIVER_

#include "display_driver.h"
#include "display_types.h"
#include "esp_err.h"

/**
 * @brief FPGA-based display driver
 *
 * This driver controls the display via FPGA RPC commands over UART.
 * Used in CP02 hardware where the FPGA manages the display controller.
 *
 * Supported features:
 * - Display mode control
 * - Intensity/brightness adjustment (0-100% mapped to 0-128)
 * - Percentage display (progress bars, etc.)
 * - Rotation (0° and 180° only, mapped to FPGA flip register)
 *
 * Not supported:
 * - 90° and 270° rotation (FPGA hardware limitation)
 */
class FPGADisplayDriver : public IDisplayDriver {
 public:
  esp_err_t Init() override;
  esp_err_t Deinit() override;
  esp_err_t SetMode(uint8_t mode) override;
  esp_err_t SetIntensity(uint8_t intensity) override;
  esp_err_t SetPercentage(uint8_t percentage) override;
  esp_err_t SetRotation(uint8_t rotation) override;
  esp_err_t EnableAutoPowerDisplay() override;
  void Clear() override;

 private:
  uint8_t current_mode_ = DisplayMode::OFF;
};

#endif /* H_FPGA_DISPLAY_DRIVER_ */
