#include "display_handler.h"

#include <cstdint>
#include <vector>

#include "display_manager.h"
#include "display_types.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_default.h"
#include "nvs_namespace.h"

static const char *TAG = "DisplayHandler";

static uint8_t display_state = 0;

esp_err_t DisplayHandler::SetDisplayIntensity(
    const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  if (request.size() != 1) {
    ESP_LOGW(TAG, "Invalid request size");
    return ESP_FAIL;
  }
  uint8_t intensity = request[0], mode;
  DisplayManager &display_manager = DisplayManager::GetInstance();
  mode = display_manager.GetDisplayMode();
  if (intensity == 0) {
    // set display mode to off if intensity is 0
    mode = DisplayMode::OFF;
  } else if (mode == DisplayMode::OFF) {
    mode = DisplayMode::POWER_METER;
  }
  ESP_RETURN_ON_ERROR(display_manager.SetDisplayMode(mode), TAG,
                      "SetDisplayMode");
  return display_manager.SetDisplayIntensity(intensity);
}

esp_err_t DisplayHandler::SetDisplayMode(const std::vector<uint8_t> &request,
                                         std::vector<uint8_t> &response) {
#if CONFIG_MCU_MODEL_FAKE_SW3566
  return ESP_OK;
#endif
  if (request.size() != 1) {
    ESP_LOGW(TAG, "Invalid request size");
    return ESP_FAIL;
  }
  uint8_t mode = request[0];
  return DisplayManager::GetInstance().SetDisplayMode(mode);
}

esp_err_t DisplayHandler::GetDisplayIntensity(
    const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  uint8_t display_intensity =
      DisplayManager::GetInstance().GetDisplayIntensity();
  response.emplace_back(display_intensity);
  return ESP_OK;
}

esp_err_t DisplayHandler::GetDisplayMode(const std::vector<uint8_t> &request,
                                         std::vector<uint8_t> &response) {
  uint8_t display_mode = DisplayManager::GetInstance().GetDisplayMode();
  response.emplace_back(display_mode);
  return ESP_OK;
}

esp_err_t DisplayHandler::SetDisplayFlip(const std::vector<uint8_t> &request,
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
  ESP_RETURN_ON_ERROR(
      DisplayManager::GetInstance().SetDisplayFlipMode(display_flip), TAG,
      "set_display_flip_mode");
  return ESP_OK;
}

esp_err_t DisplayHandler::GetDisplayFlip(const std::vector<uint8_t> &request,
                                         std::vector<uint8_t> &response) {
  uint8_t display_flip = 0;
  DisplayNVSGetOrDefault(&display_flip, NVSKey::DISPLAY_FLIP);
  response.emplace_back(display_flip);
  return ESP_OK;
}

esp_err_t DisplayHandler::SetDisplayConfig(const std::vector<uint8_t> &request,
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
  DisplayManager &display_manager = DisplayManager::GetInstance();
  ESP_RETURN_ON_ERROR(display_manager.SetDisplayIntensity(intensity), TAG,
                      "SetDisplayIntensity");

  uint8_t flip = request[1];
  ESP_RETURN_ON_ERROR(DisplayNVSSetIfDifferent(flip, NVSKey::DISPLAY_FLIP), TAG,
                      "SaveDisplayFlip");
  ESP_RETURN_ON_ERROR(DisplayManager::GetInstance().SetDisplayFlipMode(flip),
                      TAG, "set_display_flip_mode");

  ESP_RETURN_ON_ERROR(display_manager.SetDisplayMode(mode), TAG,
                      "SetDisplayMode");

  return ESP_OK;
}

enum DisplayState {
  DISPLAY_OFF = 0,
  DISPLAY_ON_FULL = 1,
  DISPLAY_RESET = 2,
};

esp_err_t DisplayHandler::SetDisplayState(const std::vector<uint8_t> &request,
                                          std::vector<uint8_t> &response) {
  if (request.size() != 1) {
    ESP_LOGE(TAG, "Invalid request size");
    return ESP_FAIL;
  }
  uint8_t state = request[0];
  DisplayManager &display_manager = DisplayManager::GetInstance();
  switch (state) {
    case DISPLAY_OFF: {
      ESP_RETURN_ON_ERROR(display_manager.SetDisplayMode(DisplayMode::OFF), TAG,
                          "DisplayManager::SetDisplayMode OFF");
      break;
    }
    case DISPLAY_ON_FULL: {
      display_manager.SetAnimation(AnimationType::IDLE_ANIMATION, true);
      ESP_RETURN_ON_ERROR(display_manager.SetDisplayMode(DisplayMode::MANUAL),
                          TAG, "DisplayManager::SetDisplayMode MANUAL");
      ESP_RETURN_ON_ERROR(display_manager.SetDisplayIntensity(0xff), TAG,
                          "DisplayManager::SetDisplayIntensity");
      ESP_RETURN_ON_ERROR(display_manager.SetDisplayPercentage(100), TAG,
                          "DisplayManager::SetDisplayPercentage");
      break;
    }
    case DISPLAY_RESET: {
      ESP_RETURN_ON_ERROR(display_manager.SetDisplayMode(), TAG,
                          "DisplayManager::SetDisplayMode");
      ESP_RETURN_ON_ERROR(display_manager.SetDisplayIntensity(), TAG,
                          "DisplayManager::SetDisplayIntensity");
      break;
    }
    default:
      ESP_LOGE(TAG, "Invalid state %d", state);
      return ESP_FAIL;
  }
  display_state = state;
  return ESP_OK;
}

esp_err_t DisplayHandler::GetDisplayState(const std::vector<uint8_t> &request,
                                          std::vector<uint8_t> &response) {
  response.emplace_back(display_state);
  return ESP_OK;
}
