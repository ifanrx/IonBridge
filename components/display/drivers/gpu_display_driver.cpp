#include "gpu_display_driver.h"

#include "display_types.h"
#include "esp_err.h"
#include "rpc.h"

esp_err_t GPUDisplayDriver::Init() {
  // Initialize with default intensity
  return rpc::gpu::set_backlight_intensity(current_intensity_);
}

esp_err_t GPUDisplayDriver::Deinit() {
  // Turn off display on deinit
  return rpc::gpu::set_display_status(false);
}

esp_err_t GPUDisplayDriver::SetMode(uint8_t mode) {
  if (mode == DisplayMode::OFF) {
    // Turn off display
    return rpc::gpu::set_display_status(false);
  }

  // Turn on display and restore intensity
  esp_err_t err = rpc::gpu::set_display_status(true);
  if (err != ESP_OK) {
    return err;
  }
  return SetIntensity(current_intensity_);
}

esp_err_t GPUDisplayDriver::SetIntensity(uint8_t intensity) {
  // Clamp input to 0-100% range
  if (intensity > 100) {
    intensity = 100;
  }

  /**
   * Fast conversion from 0-100 to 0-255 using fixed-point arithmetic.
   * The goal is to multiply by 255/100 (2.55).
   * * Optimization: (intensity * 653) >> 8
   * 653/256 is approximately 2.55078.
   * * This avoids expensive integer division and floating-point math,
   * executing in just a few clock cycles on most MCUs.
   */
  uint8_t scaled_val =
      static_cast<uint8_t>((static_cast<uint32_t>(intensity) * 653) >> 8);

  current_intensity_ = intensity;

  // Pass the 8-bit value to the hardware abstraction layer
  return rpc::gpu::set_backlight_intensity(scaled_val);
}

esp_err_t GPUDisplayDriver::SetRotation(uint8_t rotation) {
  (void)rotation;
  return ESP_OK;
}

esp_err_t GPUDisplayDriver::SetAnimationIndex(
    GPUAnimationIndex animation_index, PreCompositeMode pre_composite_mode,
    PostCompositeMode post_composite_mode) {
  return rpc::gpu::set_animation_index(animation_index, pre_composite_mode,
                                       post_composite_mode);
}

esp_err_t GPUDisplayDriver::SetAnimationData(
    const SetAnimationDataRequest& data) {
  return rpc::gpu::set_animation_data(data);
}

esp_err_t GPUDisplayDriver::DrawText(const DrawTextRequest& request) {
  return rpc::gpu::draw_text(request);
}

esp_err_t GPUDisplayDriver::PrepareFramebuffer(
    const PrepareFramebufferRequest& request) {
  return rpc::gpu::prepare_framebuffer(request);
}

esp_err_t GPUDisplayDriver::SetFramebufferChunk(
    const SetFramebufferRequest& request) {
  return rpc::gpu::set_framebuffer_chunk(request);
}
