#ifndef H_STUB_DISPLAY_DRIVER_
#define H_STUB_DISPLAY_DRIVER_

#include "display_driver.h"
#include "esp_err.h"

/**
 * @brief Stub display driver for testing/headless operation
 *
 * This driver provides a no-op implementation that accepts all commands
 * but performs no actual display operations. Useful for:
 * - Headless operation without physical display hardware
 * - Testing firmware logic without display dependencies
 * - Simulation environments
 */
class StubDisplayDriver : public IDisplayDriver {
 public:
  esp_err_t Init() override { return ESP_OK; }
  esp_err_t Deinit() override { return ESP_OK; }
  esp_err_t SetMode(uint8_t mode) override { return ESP_OK; }
  esp_err_t SetIntensity(uint8_t intensity) override { return ESP_OK; }
  esp_err_t SetPercentage(uint8_t percentage) override { return ESP_OK; }
  esp_err_t SetRotation(uint8_t rotation) override { return ESP_OK; }
};

#endif /* H_STUB_DISPLAY_DRIVER_ */
