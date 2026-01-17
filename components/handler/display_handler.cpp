#include "display_handler.h"

#include <cstdint>
#include <vector>

#include "animation_data_source.h"
#include "command.pb.h"
#include "config_sync_manager.h"
#include "display_manager.h"
#include "display_types.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_key.h"
#include "nvs_namespace.h"

static const char *TAG = "DisplayHandler";

using nvs_namespace::user_data::DISPLAY_DATA;
using namespace DisplayHandler;

namespace {
static esp_err_t SetIntensity(uint8_t intensity) {
  DisplayManager &display_manager = DisplayManager::GetInstance();
  uint8_t mode = display_manager.GetMode();
  if (intensity == 0) {
    // set display mode to off if intensity is 0
    mode = DisplayMode::OFF;
  } else if (mode == DisplayMode::OFF) {
    mode = DisplayMode::POWER_METER;
  }
  ESP_RETURN_ON_ERROR(display_manager.SetMode(mode), TAG, "SetMode");
  return display_manager.SetIntensity(intensity);
}
};  // namespace

static uint8_t display_state = 0;

esp_err_t FromMQTT::SetDisplayIntensity(const pb_CommandRequest &request,
                                        pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_set_display_intensity_tag;
  uint8_t intensity = request.payload.set_display_intensity.intensity;
  esp_err_t result = SetIntensity(intensity);
  if (result == ESP_OK) {
    ConfigSyncManager::GetInstance().NotifyConfigChange();
  }
  return result;
}

esp_err_t FromMQTT::SetDisplayMode(const pb_CommandRequest &request,
                                   pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_set_display_mode_tag;
#if CONFIG_MCU_MODEL_FAKE_SW3566
  return ESP_OK;
#endif
  uint8_t mode = request.payload.set_display_mode.mode;
  esp_err_t result = DisplayManager::GetInstance().SetMode(mode);
  if (result == ESP_OK) {
    ConfigSyncManager::GetInstance().NotifyConfigChange();
  }
  return result;
}

esp_err_t FromMQTT::GetDisplayIntensity(const pb_CommandRequest &request,
                                        pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_get_display_intensity_tag;
  uint8_t display_intensity = DisplayManager::GetInstance().GetIntensity();
  response.payload.get_display_intensity.intensity = display_intensity;
  return ESP_OK;
}

esp_err_t FromMQTT::GetDisplayMode(const pb_CommandRequest &request,
                                   pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_get_display_mode_tag;
  uint8_t display_mode = DisplayManager::GetInstance().GetMode();
  response.payload.get_display_mode.mode =
      static_cast<pb_DisplayMode>(display_mode);
  return ESP_OK;
}

esp_err_t FromMQTT::SetDisplayFlip(const pb_CommandRequest &request,
                                   pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_set_display_flip_tag;
#if CONFIG_MCU_MODEL_FAKE_SW3566
  return ESP_OK;
#endif
  // Translate flip command to rotation: 0 (normal) -> 0°, 1 (flipped) -> 180°
  uint8_t display_flip = request.payload.set_display_flip.flip;
  uint8_t rotation = (display_flip == 1) ? 180 : 0;
  ESP_RETURN_ON_ERROR(DisplayManager::GetInstance().SetRotation(rotation), TAG,
                      "set_display_rotation");
  ConfigSyncManager::GetInstance().NotifyConfigChange();
  return ESP_OK;
}

esp_err_t FromMQTT::GetDisplayFlip(const pb_CommandRequest &request,
                                   pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_get_display_flip_tag;
  // Read rotation and convert to flip: 180° -> flipped, otherwise -> normal
  uint8_t rotation = 0;
  DISPLAY_DATA.Get(nvs_keys::DISPLAY_ROTATION, rotation);
  uint8_t display_flip = (rotation == 180) ? 1 : 0;
  response.payload.get_display_flip.flip =
      static_cast<pb_DisplayFlip>(display_flip);
  return ESP_OK;
}

esp_err_t FromMQTT::SetDisplayConfig(const pb_CommandRequest &request,
                                     pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_set_display_config_tag;

  uint8_t intensity = request.payload.set_display_config.config.intensity;
  uint8_t flip = request.payload.set_display_config.config.flip;
  uint8_t mode = request.payload.set_display_config.config.mode;

  DisplayManager &display_manager = DisplayManager::GetInstance();

  // If intensity is 0, just turn off display and skip other settings
  if (intensity == 0) {
    mode = DisplayMode::OFF;
  }

  ESP_RETURN_ON_ERROR(display_manager.SetIntensity(intensity), TAG,
                      "SetIntensity");

  // Translate flip to rotation: 0 (normal) -> 0°, 1 (flipped) -> 180°
  uint8_t rotation = (flip == 1) ? 180 : 0;
  ESP_RETURN_ON_ERROR(display_manager.SetRotation(rotation), TAG,
                      "set_display_rotation");

  ESP_RETURN_ON_ERROR(display_manager.SetMode(mode), TAG, "SetMode");

  ConfigSyncManager::GetInstance().NotifyConfigChange();
  return ESP_OK;
}

esp_err_t FromMQTT::GetDisplayRotation(const pb_CommandRequest &request,
                                       pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_get_display_rotation_tag;
  uint8_t rotation = DisplayManager::GetInstance().GetRotation();
  response.payload.get_display_rotation.rotation =
      static_cast<pb_DisplayRotation>(rotation);
  return ESP_OK;
}

esp_err_t FromMQTT::SetDisplayRotation(const pb_CommandRequest &request,
                                       pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_set_display_rotation_tag;
  uint8_t rotation = request.payload.set_display_rotation.rotation;
  ESP_RETURN_ON_ERROR(DisplayManager::GetInstance().SetRotation(rotation), TAG,
                      "SetRotation");
  ConfigSyncManager::GetInstance().NotifyConfigChange();
  return ESP_OK;
}

enum DisplayState {
  DISPLAY_OFF = 0,
  DISPLAY_ON_FULL = 1,
  DISPLAY_RESET = 2,
};

esp_err_t FromBLE::SetDisplayState(const std::vector<uint8_t> &request,
                                   std::vector<uint8_t> &response) {
  if (request.size() != 1) {
    ESP_LOGE(TAG, "Invalid request size");
    return ESP_FAIL;
  }
  uint8_t state = request[0];
  DisplayManager &display_manager = DisplayManager::GetInstance();
  switch (state) {
    case DISPLAY_OFF: {
      display_manager.SetNextAnimation(AnimationType::SCREEN_OFF, true);
      break;
    }
    case DISPLAY_ON_FULL: {
      display_manager.SetNextAnimation(AnimationType::IDLE, true);
      ESP_RETURN_ON_ERROR(display_manager.SetMode(DisplayMode::MANUAL), TAG,
                          "DisplayManager::SetMode MANUAL");
      ESP_RETURN_ON_ERROR(display_manager.SetIntensity(100), TAG,
                          "DisplayManager::SetIntensity");
      ESP_RETURN_ON_ERROR(display_manager.SetPercentage(100), TAG,
                          "DisplayManager::SetPercentage");
      break;
    }
    case DISPLAY_RESET: {
      ESP_RETURN_ON_ERROR(display_manager.SetMode(), TAG,
                          "DisplayManager::SetMode");
      ESP_RETURN_ON_ERROR(display_manager.SetIntensity(), TAG,
                          "DisplayManager::SetIntensity");
      break;
    }
    default:
      ESP_LOGE(TAG, "Invalid state %d", state);
      return ESP_FAIL;
  }
  display_state = state;
  ConfigSyncManager::GetInstance().NotifyConfigChange();
  return ESP_OK;
}

esp_err_t FromBLE::GetDisplayState(const std::vector<uint8_t> &request,
                                   std::vector<uint8_t> &response) {
  response.emplace_back(display_state);
  return ESP_OK;
}

esp_err_t FromBLE::GetDisplayRotation(const std::vector<uint8_t> &request,
                                      std::vector<uint8_t> &response) {
  uint8_t rotation = DisplayManager::GetInstance().GetRotation();
  response.emplace_back(rotation);
  return ESP_OK;
}

esp_err_t FromBLE::SetDisplayRotation(const std::vector<uint8_t> &request,
                                      std::vector<uint8_t> &response) {
  if (request.size() != 1) {
    ESP_LOGE(TAG, "Invalid request size");
    return ESP_FAIL;
  }
  uint8_t rotation = request[0];
  ESP_RETURN_ON_ERROR(DisplayManager::GetInstance().SetRotation(rotation), TAG,
                      "SetRotation");
  ConfigSyncManager::GetInstance().NotifyConfigChange();
  return ESP_OK;
}

esp_err_t FromBLE::SetDisplayIntensity(const std::vector<uint8_t> &request,
                                       std::vector<uint8_t> &response) {
  if (request.size() != 1) {
    ESP_LOGE(TAG, "Invalid request size");
    return ESP_FAIL;
  }
  uint8_t intensity = request[0];
  esp_err_t result = SetIntensity(intensity);
  if (result == ESP_OK) {
    ConfigSyncManager::GetInstance().NotifyConfigChange();
  }
  return result;
}

esp_err_t FromBLE::GetDisplayIntensity(const std::vector<uint8_t> &request,
                                       std::vector<uint8_t> &response) {
  uint8_t display_intensity = DisplayManager::GetInstance().GetIntensity();
  response.emplace_back(display_intensity);
  return ESP_OK;
}

esp_err_t FromBLE::ClearFactoryDMFlag(const std::vector<uint8_t> &request,
                                      std::vector<uint8_t> &response) {
  (void)request;   // Unused parameter
  (void)response;  // Unused parameter
  // Erase factory flag - one-way factory→production transition
  nvs_namespace::protected_data::DEVICE_DATA.EraseKey(
      nvs_keys::DEVICE_DATA_MATRIX_FLAG);
  return ESP_OK;
}

esp_err_t FromMQTT::ClearFactoryDMFlag(const pb_CommandRequest &request,
                                       pb_CommandResponse &response) {
  (void)request;  // Unused parameter
  response.which_payload = pb_CommandResponse_clear_factory_dm_flag_tag;
  // Erase factory flag - one-way factory→production transition
  nvs_namespace::protected_data::DEVICE_DATA.EraseKey(
      nvs_keys::DEVICE_DATA_MATRIX_FLAG);
  return ESP_OK;
}

esp_err_t FromMQTT::SetDisplaySetup(const pb_CommandRequest &request,
                                    pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_set_display_setup_tag;

  const auto &setup = request.payload.set_display_setup.setup;
  DisplayManager &display_manager = DisplayManager::GetInstance();

  // Set intensity
  ESP_RETURN_ON_ERROR(SetIntensity(setup.intensity), TAG, "SetIntensity");

  // Set rotation (ignore if not supported by driver)
  esp_err_t rot_err = display_manager.SetRotation(setup.rotation);
  if (rot_err != ESP_OK && rot_err != ESP_ERR_NOT_SUPPORTED) {
    ESP_LOGE(TAG, "SetRotation failed: %s", esp_err_to_name(rot_err));
    return rot_err;
  }

  // Set idle animation type
  AnimationDataSource::SetIdleAnimation(setup.idle_animation);

  ConfigSyncManager::GetInstance().NotifyConfigChange();
  return ESP_OK;
}

esp_err_t FromMQTT::GetDisplaySetup(const pb_CommandRequest &request,
                                    pb_CommandResponse &response) {
  (void)request;  // Unused parameter
  response.which_payload = pb_CommandResponse_get_display_setup_tag;

  DisplayManager &display_manager = DisplayManager::GetInstance();
  AnimationDataSource data_source;

  auto &setup = response.payload.get_display_setup.setup;
  setup.intensity = display_manager.GetIntensity();
  setup.rotation =
      static_cast<pb_DisplayRotation>(display_manager.GetRotation());
  setup.idle_animation =
      static_cast<pb_IdleAnimationType>(data_source.GetIdleAnimationType());

  return ESP_OK;
}
