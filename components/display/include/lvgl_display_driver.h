#ifndef H_LVGL_DISPLAY_DRIVER_
#define H_LVGL_DISPLAY_DRIVER_

#include "display_driver.h"
#include "esp_err.h"

/**
 * @brief LVGL-based display driver
 *
 * This driver controls LCD panels directly via the LVGL graphics library
 * and ESP BSP (Board Support Package). Used in I2C-based hardware like
 * 68W/PA768S boards.
 *
 * Supported features:
 * - Display mode (on/off via backlight control)
 * - Brightness/intensity adjustment (0-100%)
 * - Display rotation (0°, 90°, 180°, 270°)
 *
 * Not supported:
 * - Percentage display (handled by animation layer)
 *
 * Note: This driver only handles hardware-level operations. Application-level
 * features like fonts, styles, and animations are managed by DisplayManager.
 */
class LVGLDisplayDriver : public IDisplayDriver {
 public:
  esp_err_t Init() override;
  esp_err_t Deinit() override;
  esp_err_t SetMode(uint8_t mode) override;
  esp_err_t SetIntensity(uint8_t intensity) override;
  esp_err_t SetRotation(uint8_t rotation) override;
  bool IsMemoryIntensive() const override { return true; }
};

#endif /* H_LVGL_DISPLAY_DRIVER_ */
