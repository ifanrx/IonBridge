#include "lvgl_display_driver.h"

#include "bsp/bsp_generic.h"
#include "bsp/display.h"
#include "display_types.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"

static const char *TAG = "LVGLDisplayDriver";

#define display_backlight_on bsp_display_backlight_on
#define display_backlight_off bsp_display_backlight_off

esp_err_t LVGLDisplayDriver::Init() {
  ESP_RETURN_ON_FALSE(bsp_display_start() != NULL, ESP_FAIL, TAG,
                      "bsp_display_start");
  display_backlight_off();
  return ESP_OK;
}

esp_err_t LVGLDisplayDriver::Deinit() { return lvgl_port_deinit(); }

esp_err_t LVGLDisplayDriver::SetMode(uint8_t mode) {
  switch (mode) {
    case DisplayMode::OFF:
      return display_backlight_off();
    default:
      return display_backlight_on();
  }
}

esp_err_t LVGLDisplayDriver::SetIntensity(uint8_t intensity) {
  return bsp_display_brightness_set(intensity);
}

esp_err_t LVGLDisplayDriver::SetRotation(uint8_t rotation) {
#if CONFIG_BSP_DISPLAY_GAP_SET
  lvgl_port_display_cfg_t *disp_cfg = bsp_display_get_config();
  int x_gap = 0;
  int y_gap = 0;

  switch (rotation) {
    case LV_DISPLAY_ROTATION_0:
      x_gap = CONFIG_BSP_DISPLAY_GAP_X;
      y_gap = CONFIG_BSP_DISPLAY_GAP_Y;
      break;
    case LV_DISPLAY_ROTATION_90:
    case LV_DISPLAY_ROTATION_180:
      x_gap = 0;
      y_gap = 0;
      break;
    case LV_DISPLAY_ROTATION_270:
      x_gap = CONFIG_BSP_DISPLAY_GAP_Y;
      y_gap = CONFIG_BSP_DISPLAY_GAP_X;
      break;
    default:
      ESP_LOGW(TAG, "Invalid rotation: %d", rotation);
      return ESP_ERR_INVALID_ARG;
  }

  ESP_RETURN_ON_ERROR(
      esp_lcd_panel_set_gap(disp_cfg->panel_handle, x_gap, y_gap), TAG,
      "esp_lcd_panel_set_gap");
#else
  // Validate rotation value
  switch (rotation) {
    case LV_DISPLAY_ROTATION_0:
    case LV_DISPLAY_ROTATION_90:
    case LV_DISPLAY_ROTATION_180:
    case LV_DISPLAY_ROTATION_270:
      break;
    default:
      ESP_LOGW(TAG, "Invalid rotation: %d", rotation);
      return ESP_ERR_INVALID_ARG;
  }
#endif

  bsp_display_rotate(NULL, static_cast<lv_display_rotation_t>(rotation));
  return ESP_OK;
}
