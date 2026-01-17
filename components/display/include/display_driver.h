#ifndef H_DISPLAY_DRIVER_
#define H_DISPLAY_DRIVER_

#include <cstdint>

#include "esp_err.h"

/**
 * @brief Abstract interface for display drivers
 *
 * This interface provides a common abstraction for different display hardware:
 * - FPGA: Display controlled via FPGA RPC commands (UART)
 * - GPU: Display with dedicated GPU chip via RPC (UART)
 * - LVGL: Direct LCD panel control via LVGL graphics library (I2C)
 * - Stub: No-op implementation for testing/headless operation
 *
 * Each driver is compiled conditionally based on CONFIG_DISPLAY_DRIVER_*
 * Kconfig options, ensuring zero runtime overhead.
 */
class IDisplayDriver {
 public:
  virtual ~IDisplayDriver() = default;

  /**
   * @brief Initialize the display driver
   * @return ESP_OK on success, error code otherwise
   */
  virtual esp_err_t Init() = 0;

  /**
   * @brief Deinitialize the display driver
   * @return ESP_OK on success, error code otherwise
   */
  virtual esp_err_t Deinit() = 0;

  /**
   * @brief Set display mode
   * @param mode Display mode value (driver-specific interpretation)
   * @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED if not implemented
   */
  virtual esp_err_t SetMode(uint8_t mode) = 0;

  /**
   * @brief Set display intensity/brightness
   * @param intensity Intensity value 0-100 (percentage)
   * @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED if not implemented
   */
  virtual esp_err_t SetIntensity(uint8_t intensity) = 0;

  /**
   * @brief Set display percentage (progress bar, etc.)
   * @param percentage Percentage value 0-100
   * @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED if not implemented
   */
  virtual esp_err_t SetPercentage(uint8_t percentage) {
    return ESP_ERR_NOT_SUPPORTED;
  }

  /**
   * @brief Set display rotation
   * @param rotation Rotation angle (0, 90, 180, 270 degrees)
   * @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED if not implemented
   */
  virtual esp_err_t SetRotation(uint8_t rotation) {
    return ESP_ERR_NOT_SUPPORTED;
  }

  /**
   * @brief Enable auto-updating power display mode
   * @details For hardware with auto-power-meter capabilities (e.g., FPGA
   * POWER_METER mode), enables automatic display of real-time power
   * consumption.
   * @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED if not implemented
   */
  virtual esp_err_t EnableAutoPowerDisplay() { return ESP_ERR_NOT_SUPPORTED; }

  /**
   * @brief Clear the display
   * @details Mode-aware clear operation. In autonomous modes (e.g.,
   * POWER_METER), this is a no-op to avoid disrupting hardware-controlled
   * display.
   */
  virtual void Clear() {}

  /**
   * @brief Check if driver uses significant memory
   * @return true if driver uses large memory buffers (e.g., LVGL frame
   * buffers)
   * @return false for lightweight drivers (e.g., FPGA, GPU via RPC)
   */
  virtual bool IsMemoryIntensive() const { return false; }
};

#endif /* H_DISPLAY_DRIVER_ */
