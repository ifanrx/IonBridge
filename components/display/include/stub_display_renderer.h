#ifndef H_STUB_DISPLAY_RENDERER_
#define H_STUB_DISPLAY_RENDERER_

#include "display_renderer.h"
#include "stub_display_driver.h"

#ifdef CONFIG_DISPLAY_DRIVER_STUB

/**
 * @brief Stub display renderer (no-op implementation)
 *
 * Inherits all default no-op behavior from IDisplayRenderer.
 * Only overrides Init/Deinit for logging, and GetDriver() (required).
 */
class StubRenderer : public IDisplayRenderer {
 public:
  explicit StubRenderer(StubDisplayDriver* driver) : driver_(driver) {}

  esp_err_t Init() override;
  esp_err_t Deinit() override;
  IDisplayDriver* GetDriver() override { return driver_; }

 private:
  StubDisplayDriver* driver_;
};

#endif  // CONFIG_DISPLAY_DRIVER_STUB

#endif  // H_STUB_DISPLAY_RENDERER_
