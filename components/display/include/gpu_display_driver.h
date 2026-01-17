#ifndef H_GPU_DISPLAY_DRIVER_
#define H_GPU_DISPLAY_DRIVER_

#include "display_driver.h"
#include "esp_err.h"
#include "gpu_data_types.h"

/**
 * @brief GPU-based display driver
 *
 * This driver controls a 96x32 monochrome OLED display via a dedicated GPU
 * chip over UART RPC. The GPU provides hardware-accelerated graphics
 * primitives, built-in animations, and framebuffer compositing.
 *
 * Supported features:
 * - Backlight intensity control (0-100%)
 * - Display mode (on/off via backlight)
 *
 * Advanced features (not exposed via IDisplayDriver):
 * - Built-in animations (VU meters, Game of Life, test patterns)
 * - Custom framebuffer upload (chunked protocol for large frames)
 * - Text rendering (6x12 and 8x16 fonts)
 * - BitBLT compositing modes (pre/post composite)
 *
 * Not supported via basic interface:
 * - Percentage display (could be implemented as custom animation)
 * - Flip mode
 * - Rotation
 */
class GPUDisplayDriver : public IDisplayDriver {
 public:
  esp_err_t Init() override;
  esp_err_t Deinit() override;
  esp_err_t SetMode(uint8_t mode) override;
  esp_err_t SetIntensity(uint8_t intensity) override;
  esp_err_t SetRotation(uint8_t rotation) override;

  // GPU-specific methods (not part of IDisplayDriver interface)
  esp_err_t SetAnimationIndex(GPUAnimationIndex animation_index,
                              PreCompositeMode pre_composite_mode,
                              PostCompositeMode post_composite_mode);
  esp_err_t SetAnimationData(const SetAnimationDataRequest& data);
  esp_err_t DrawText(const DrawTextRequest& request);
  esp_err_t PrepareFramebuffer(const PrepareFramebufferRequest& request);
  esp_err_t SetFramebufferChunk(const SetFramebufferRequest& request);

 private:
  uint8_t current_intensity_ = 50;  // Default 50%
};

#endif /* H_GPU_DISPLAY_DRIVER_ */
