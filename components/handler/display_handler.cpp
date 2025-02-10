#include "display_handler.h"

#include <cstdint>
#include <vector>

#include "animation.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_default.h"
#include "nvs_namespace.h"
#include "rpc.h"

static const char *TAG = "DisplayHandler";

static uint8_t display_state = 0;

esp_err_t DisplayHandler::SetDisplayIntensity(
    AppContext &ctx, const std::vector<uint8_t> &request,
    std::vector<uint8_t> &response) {
  if (request.size() != 1) {
    ESP_LOGW(TAG, "Invalid request size");
    return ESP_FAIL;
  }
  uint8_t intensity = request[0];
  if (intensity == 0) {
    // set display mode to off if intensity is 0
    ctx.controller.set_display_intensity(intensity);
    ESP_RETURN_ON_ERROR(ctx.controller.set_display_mode(DisplayMode::OFF), TAG,
                        "rpc::display::set_display_mode");
    return ESP_OK;
  }
  if (ctx.controller.get_display_mode() == DisplayMode::OFF) {
    ESP_RETURN_ON_ERROR(
        ctx.controller.set_display_mode(DisplayMode::POWER_METER), TAG,
        "rpc::display::set_display_mode");
  }
  return ctx.controller.set_display_intensity(intensity);
}

esp_err_t DisplayHandler::SetDisplayMode(AppContext &ctx,
                                         const std::vector<uint8_t> &request,
                                         std::vector<uint8_t> &response) {
#if CONFIG_MCU_MODEL_FAKE_SW3566
  return ESP_OK;
#endif
  if (request.size() != 1) {
    ESP_LOGW(TAG, "Invalid request size");
    return ESP_FAIL;
  }
  uint8_t mode = request[0];
  return ctx.controller.set_display_mode(mode);
}

esp_err_t DisplayHandler::GetDisplayIntensity(
    AppContext &ctx, const std::vector<uint8_t> &request,
    std::vector<uint8_t> &response) {
  uint8_t display_intensity = ctx.controller.get_display_intensity();
  response.emplace_back(display_intensity);
  return ESP_OK;
}

esp_err_t DisplayHandler::GetDisplayMode(AppContext &ctx,
                                         const std::vector<uint8_t> &request,
                                         std::vector<uint8_t> &response) {
  uint8_t display_mode = ctx.controller.get_display_mode();
  response.emplace_back(display_mode);
  return ESP_OK;
}

esp_err_t DisplayHandler::SetDisplayFlip(AppContext &ctx,
                                         const std::vector<uint8_t> &request,
                                         std::vector<uint8_t> &response) {
#if CONFIG_MCU_MODEL_FAKE_SW3566
  return ESP_OK;
#endif
  if (request.size() != 1) {
    ESP_LOGW(TAG, "Invalid request size");
    return ESP_FAIL;
  }

  uint8_t display_flip = request[0];
  ESP_RETURN_ON_ERROR(
      DisplayNVSSetIfDifferent(display_flip, NVSKey::DISPLAY_FLIP), TAG,
      "SaveDisplayFlip");
  ESP_RETURN_ON_ERROR(rpc::display::set_display_flip_mode(display_flip), TAG,
                      "rpc::display::set_display_flip_mode");
  return ESP_OK;
}

esp_err_t DisplayHandler::GetDisplayFlip(AppContext &ctx,
                                         const std::vector<uint8_t> &request,
                                         std::vector<uint8_t> &response) {
  uint8_t display_flip = 0;
  DisplayNVSGetOrDefault(&display_flip, NVSKey::DISPLAY_FLIP);
  response.emplace_back(display_flip);
  return ESP_OK;
}

esp_err_t DisplayHandler::SetDisplayConfig(AppContext &ctx,
                                           const std::vector<uint8_t> &request,
                                           std::vector<uint8_t> &response) {
  if (request.size() != 5) {
    ESP_LOGE(TAG, "Invalid SetDisplayConfig request size %d", request.size());
    return ESP_FAIL;
  }
  uint8_t intensity = request[0];
  uint8_t mode = request[2];
  if (intensity == 0) {
    mode = DisplayMode::OFF;
  }
  ESP_RETURN_ON_ERROR(ctx.controller.set_display_intensity(intensity), TAG,
                      "set_display_intensity");

  uint8_t flip = request[1];
  ESP_RETURN_ON_ERROR(DisplayNVSSetIfDifferent(flip, NVSKey::DISPLAY_FLIP), TAG,
                      "SaveDisplayFlip");
  ESP_RETURN_ON_ERROR(rpc::display::set_display_flip_mode(flip), TAG,
                      "rpc::display::set_display_flip_mode");

  ESP_RETURN_ON_ERROR(ctx.controller.set_display_mode(mode), TAG,
                      "set_display_mode");

  return ESP_OK;
}

enum DisplayState {
  DISPLAY_OFF = 0,
  DISPLAY_ON_FULL = 1,
  DISPLAY_RESET = 2,
};

esp_err_t DisplayHandler::SetDisplayState(AppContext &ctx,
                                          const std::vector<uint8_t> &request,
                                          std::vector<uint8_t> &response) {
  if (request.size() != 1) {
    ESP_LOGE(TAG, "Invalid request size");
    return ESP_FAIL;
  }
  uint8_t state = request[0];
  switch (state) {
    case DISPLAY_OFF: {
      ESP_RETURN_ON_ERROR(rpc::display::set_display_mode(DisplayMode::OFF), TAG,
                          "rpc::display::set_display_mode OFF");
      break;
    }
    case DISPLAY_ON_FULL: {
      AnimationController::GetInstance().StopAnimation(
          AnimationId::BLE_ADVERTISING);
      ESP_RETURN_ON_ERROR(rpc::display::set_display_mode(DisplayMode::MANUAL),
                          TAG, "rpc::display::set_display_mode MANUAL");
      ESP_RETURN_ON_ERROR(rpc::display::set_display_intensity(0xff), TAG,
                          "rpc::display::set_display_intensity");
      ESP_RETURN_ON_ERROR(rpc::display::set_display_percentage(100), TAG,
                          "rpc::display::set_display_percentage");
      break;
    }
    case DISPLAY_RESET: {
      uint8_t display_mode = ctx.controller.get_display_mode();
      uint8_t display_intensity = ctx.controller.get_display_intensity();
      ESP_RETURN_ON_ERROR(rpc::display::set_display_mode(display_mode), TAG,
                          "rpc::display::set_display_mode");
      ESP_RETURN_ON_ERROR(
          rpc::display::set_display_intensity(display_intensity), TAG,
          "rpc::display::set_display_intensity");
      break;
    }
    default:
      ESP_LOGE(TAG, "Invalid state %d", state);
      return ESP_FAIL;
  }
  display_state = state;
  return ESP_OK;
}

esp_err_t DisplayHandler::GetDisplayState(AppContext &ctx,
                                          const std::vector<uint8_t> &request,
                                          std::vector<uint8_t> &response) {
  response.emplace_back(display_state);
  return ESP_OK;
}
