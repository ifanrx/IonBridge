#include "stub_display_renderer.h"

#ifdef CONFIG_DISPLAY_DRIVER_STUB

#include "esp_log.h"

static const char* TAG = "StubRenderer";

esp_err_t StubRenderer::Init() {
  ESP_LOGI(TAG, "Stub renderer initialized (no-op)");
  return ESP_OK;
}

esp_err_t StubRenderer::Deinit() {
  ESP_LOGI(TAG, "Stub renderer deinitialized (no-op)");
  return ESP_OK;
}

#endif  // CONFIG_DISPLAY_DRIVER_STUB
