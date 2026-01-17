#include <sys/stat.h>

#include "display_animation.h"
#include "display_renderer.h"
#include "display_types.h"
#include "esp_log.h"
#include "nvs_key.h"
#include "nvs_namespace.h"
#include "sdkconfig.h"

static const char* TAG = "FactoryAnimation";

// ============================================================================
// Data Matrix Animation
// ============================================================================

void DataMatrixAnimation::Entering() {
  const char* dm_path = CONFIG_FILE_SYSTEM_BASE_PATH "/dm.bin";

  struct stat st;
  if (stat(dm_path, &st) != 0) {
    ESP_LOGE(TAG, "FATAL: Factory dm.bin missing at %s - manufacturing defect",
             dm_path);
    dm_missing_ = true;
    return;
  }

  dm_missing_ = false;
  auto* r = GetRenderer();
  r->ShowImage(dm_path, Alignment::CENTER);
}

AnimationAction DataMatrixAnimation::PlayNextFrame(int64_t dt_us) {
  if (dm_missing_) {
    return AnimationAction::Complete();
  }

  std::string show_dm_flag;
  esp_err_t err = nvs_namespace::protected_data::DEVICE_DATA.Get(
      nvs_keys::DEVICE_DATA_MATRIX_FLAG, show_dm_flag);

  if ((err == ESP_OK && show_dm_flag.empty()) || err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGI(TAG, "Factory DM flag cleared, exiting animation");
    return AnimationAction::Complete();
  }

  return AnimationAction::Continue(animation_timing::kCheckIntervalUs);
}

void DataMatrixAnimation::Exiting() {
  auto* r = GetRenderer();
  r->Clear();
}

bool DataMatrixAnimation::CanBeInterrupted(AnimationType incoming) const {
  return incoming == AnimationType::DISPLAY_TEST;
}

// ============================================================================
// Display Test Animation
// ============================================================================

void DisplayTestAnimation::Entering() {
  if (auto_mode_) {
    return;
  }

  elapsed_us_ = 0;

  auto* test_caps = GetRenderer()->GetTestCapabilities();
  if (test_caps) {
    test_caps->ResetTestSequence();
  }

  ShowTestPattern(DisplayTestScreen::TEST_SCREEN_DEFAULT);
}

AnimationAction DisplayTestAnimation::PlayNextFrame(int64_t dt_us) {
  if (!auto_mode_ || (loop_once_ && completed_)) {
    return AnimationAction::Continue(animation_timing::kNeverWake);
  }

  elapsed_us_ += dt_us;
  int64_t duration_us =
      (auto_duration_ms != 0 ? auto_duration_ms : 1000) * 1000;

  if (elapsed_us_ >= duration_us) {
    elapsed_us_ = 0;

    auto* test_caps = GetRenderer()->GetTestCapabilities();
    if (!test_caps) {
      return AnimationAction::Continue(duration_us);
    }

    DisplayTestScreen screen = test_caps->GetNextTestScreen();

    if (loop_once_ && screen == DisplayTestScreen::TEST_SCREEN_OFF) {
      ShowTestPattern(DisplayTestScreen::TEST_SCREEN_DEFAULT);
      completed_ = true;
      return AnimationAction::Continue(animation_timing::kNeverWake);
    }

    ShowTestPattern(screen);
  }

  return AnimationAction::Continue(duration_us - elapsed_us_);
}

void DisplayTestAnimation::Exiting() {
  ShowTestPattern(DisplayTestScreen::TEST_SCREEN_BLACK);
  auto_mode_ = false;
  loop_once_ = false;
  completed_ = false;
  elapsed_us_ = 0;
}

void DisplayTestAnimation::ShowTestPattern(DisplayTestScreen screen) {
  auto* r = GetRenderer();
  r->ShowTestPattern(screen);

  auto* driver = r->GetDriver();
  if (screen == DisplayTestScreen::TEST_SCREEN_OFF) {
    driver->SetMode(DisplayMode::OFF);
  } else {
    driver->SetMode(DisplayMode::MANUAL);

    if (screen == DisplayTestScreen::TEST_SCREEN_LOW_LIGHT) {
      driver->SetIntensity(10);
    } else if (screen == DisplayTestScreen::TEST_SCREEN_HIGH_LIGHT) {
      driver->SetIntensity(100);
    } else {
      driver->SetIntensity(100);
    }
  }
}
