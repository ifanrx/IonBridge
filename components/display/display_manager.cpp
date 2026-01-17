#include "display_manager.h"

#include <cstdint>

#include "animation_manager.h"
#include "display_animation.h"
#include "display_driver.h"
#include "display_renderer.h"
#include "display_test.h"
#include "display_types.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_key.h"
#include "nvs_namespace.h"
#include "sdkconfig.h"

// Workaround for Linux target: ULONG_MAX not properly defined in some build
// configurations. This ensures consistent behavior across all platforms.
#if CONFIG_IDF_TARGET_LINUX
#undef ULONG_MAX
#define ULONG_MAX 0xFFFFFFFF
#else
#include <climits>
#endif

static const char *TAG = "DisplayManager";

using nvs_namespace::user_data::DISPLAY_DATA;

DisplayManager &DisplayManager::GetInstance() {
  static DisplayManager instance;
  return instance;
}

DisplayManager::DisplayManager()
    : driver_(nullptr),
      renderer_(nullptr),
      display_mode_(CONFIG_DISPLAY_MODE),
      previous_display_mode_(CONFIG_DISPLAY_MODE),
      display_intensity_(CONFIG_DISPLAY_INTENSITY),
      display_rotation_(0) {}

esp_err_t DisplayManager::Init(IDisplayDriver *driver,
                               IDisplayRenderer *renderer,
                               bool skip_animations) {
  ESP_RETURN_ON_FALSE(driver != nullptr, ESP_ERR_INVALID_ARG, TAG,
                      "driver is null");
  ESP_RETURN_ON_FALSE(renderer != nullptr, ESP_ERR_INVALID_ARG, TAG,
                      "renderer is null");
  driver_ = driver;
  renderer_ = renderer;

  // Load display state from NVS
  DISPLAY_DATA.GetOrDefault(nvs_keys::DISPLAY_MODE, display_mode_);
  DISPLAY_DATA.GetOrDefault(nvs_keys::DISPLAY_INTENSITY, display_intensity_);
  if (display_intensity_ > 100) {
    ESP_LOGW(TAG, "Correcting invalid NVS display intensity %d to 100",
             display_intensity_);
    display_intensity_ = 100;
    DISPLAY_DATA.Set(nvs_keys::DISPLAY_INTENSITY, display_intensity_);
  }
  DISPLAY_DATA.GetOrDefault(nvs_keys::DISPLAY_ROTATION, display_rotation_);

  // Migrate legacy DISPLAY_FLIP to DISPLAY_ROTATION (backward compatibility)
  uint8_t legacy_flip;
  if (DISPLAY_DATA.Get(nvs_keys::DISPLAY_FLIP, legacy_flip) == ESP_OK) {
    ESP_LOGI(TAG, "Migrating legacy DISPLAY_FLIP=%d to DISPLAY_ROTATION",
             legacy_flip);
    display_rotation_ = (legacy_flip == 1) ? 180 : 0;
    DISPLAY_DATA.Set(nvs_keys::DISPLAY_ROTATION, display_rotation_);
    DISPLAY_DATA.EraseKey(nvs_keys::DISPLAY_FLIP);
  }

  // Initialize driver
  esp_err_t err = driver_->Init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "driver init failed: %s", esp_err_to_name(err));
    return err;
  }

  // Initialize renderer (cleanup driver on failure)
  err = renderer_->Init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "renderer init failed: %s", esp_err_to_name(err));
    driver_->Deinit();  // Clean up driver
    return err;
  }

  // Skip animation task if requested (e.g., missing animation resources)
  if (skip_animations) {
    ESP_LOGW(TAG, "Animation task skipped");
    return ESP_OK;
  }

  // Start animation task (cleanup driver and renderer on failure)
  err = AnimationManager::GetInstance().StartTask();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start AnimationManager task: %s",
             esp_err_to_name(err));
    renderer_->Deinit();  // Clean up renderer
    driver_->Deinit();    // Clean up driver
    return err;
  }

  return ESP_OK;
}

esp_err_t DisplayManager::Deinit() {
  Stop();
  renderer_->Deinit();
  return driver_->Deinit();
}

void DisplayManager::Stop() { AnimationManager::GetInstance().StopTask(); }

esp_err_t DisplayManager::PrepareForOTA() {
  if (driver_ && driver_->IsMemoryIntensive()) {
    return Deinit();  // Stops animations + frees driver memory
  }
  return ESP_OK;  // Lightweight driver, keep animations running
}

esp_err_t DisplayManager::Reset() {
  ESP_RETURN_ON_ERROR(SetMode((uint8_t)CONFIG_DISPLAY_MODE), TAG,
                      "Reset display mode");
  ESP_RETURN_ON_ERROR(SetIntensity((uint8_t)CONFIG_DISPLAY_INTENSITY), TAG,
                      "Reset display intensity");

  // Restore rotation from NVS - driver decides if rotation is supported
  uint8_t rotation = 0;
  DISPLAY_DATA.GetOrDefault(nvs_keys::DISPLAY_ROTATION, rotation);
  esp_err_t rot_err = SetRotation(rotation);
  if (rot_err != ESP_OK && rot_err != ESP_ERR_NOT_SUPPORTED) {
    ESP_LOGW(TAG, "Failed to reset rotation: %s", esp_err_to_name(rot_err));
  }

  return ESP_OK;
}

esp_err_t DisplayManager::SetPercentage(uint8_t percentage) {
  return driver_->SetPercentage(percentage);
}

esp_err_t DisplayManager::SetMode(uint8_t mode) {
  if (display_mode_ == mode) {
    return ESP_OK;
  }
  esp_err_t err = driver_->SetMode(mode);
  if (err == ESP_OK) {
    previous_display_mode_ = display_mode_;
    display_mode_ = mode;
  }
  return err;
}

esp_err_t DisplayManager::SetIntensity(uint8_t intensity) {
  if (intensity > 100) {
    ESP_LOGW(TAG, "Clamping display intensity %d to 100", intensity);
    intensity = 100;
  }
  esp_err_t err = driver_->SetIntensity(intensity);
  if (err == ESP_OK) {
    display_intensity_ = intensity;
  }
  return err;
}

esp_err_t DisplayManager::SetRotation(uint8_t rotation) {
  esp_err_t err = driver_->SetRotation(rotation);
  if (err == ESP_OK) {
    display_rotation_ = rotation;
  }
  return err;
}

esp_err_t DisplayManager::DisplayOn() {
  if (display_mode_ == DisplayMode::OFF) {
    return SetMode(previous_display_mode_);
  }
  return ESP_OK;
}

esp_err_t DisplayManager::DisplayOff() { return SetMode(DisplayMode::OFF); }

esp_err_t DisplayManager::TestDisplay(const DisplayTestEvent &event) {
  if (!(event.kind == DisplayTestKind::TEST_ENTER || display_testing_)) {
    ESP_LOGW(TAG, "Display test doesn't start, ignoring event: %d",
             static_cast<int>(event.kind));
    return ESP_ERR_INVALID_STATE;
  }

  auto *test_caps = renderer_->GetTestCapabilities();
  if (!test_caps) {
    ESP_LOGW(TAG, "Test screen not supported by current renderer");
    return ESP_ERR_NOT_SUPPORTED;
  }

  switch (event.kind) {
    case DisplayTestKind::TEST_ENTER: {
      if (!display_testing_) {
        display_testing_ = true;
        test_original_intensity_ = display_intensity_;
        // Save current animation to restore when exiting test mode
        test_previous_animation_ =
            AnimationManager::GetInstance().GetCurrentAnimationType();
        ESP_LOGI(TAG, "Display test started (previous animation: %s)",
                 test_previous_animation_.toString());
        SetNextAnimation(AnimationType::DISPLAY_TEST, true);
      }
    } break;
    case DisplayTestKind::TEST_EXIT: {
      display_testing_ = false;
      ESP_LOGI(TAG, "Display test ended (restoring animation: %s)",
               test_previous_animation_.toString());
      SetIntensity(test_original_intensity_);
      // Restore previous animation instead of going to IDLE
      SetNextAnimation(test_previous_animation_, false);
    } break;
    case DisplayTestKind::TEST_LIGHTNESS: {
      ESP_LOGI(TAG, "Setting display lightness: %d", event.param->lightness);
      return SetIntensity(event.param->lightness);
    } break;
    case DisplayTestKind::TEST_SCREEN: {
      ESP_LOGI(TAG, "Setting display screen: %d", event.param->screen);
      return test_caps->SetScreen(event.param->screen);
    } break;
    case DisplayTestKind::TEST_AUTO:
    case DisplayTestKind::TEST_AUTO_ONCE: {
      bool loop_once = (event.kind == DisplayTestKind::TEST_AUTO_ONCE);
      ESP_LOGI(TAG, "Setting display to auto mode (loop_once: %s)",
               loop_once ? "true" : "false");
      uint8_t duration = event.param ? event.param->duration_secs : 0;
      return test_caps->SetAutoMode(duration, loop_once);
    } break;
    case DisplayTestKind::TEST_IGNORE: {
      ESP_LOGW(TAG,
               "Received a TEST_IGNORE event, please check the code logic");
      return ESP_ERR_INVALID_STATE;
    } break;
    default: {
      ESP_LOGW(TAG, "Unknown display test kind: %d",
               static_cast<int>(event.kind));
      return ESP_ERR_INVALID_ARG;
    } break;
  }
  return ESP_OK;
}

void DisplayManager::SetNextAnimation(AnimationType animation_type, bool loop) {
  AnimationManager::GetInstance().SetNextAnimation(animation_type, loop);
}

void DisplayManager::StopAnimation(AnimationType animation_type) {
  AnimationManager::GetInstance().StopAnimation(animation_type);
}

void DisplayManager::SetOTAStage(OTAStage stage) {
  AnimationManager::GetInstance().SetOTAStage(stage);
}
