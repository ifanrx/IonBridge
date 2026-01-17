#include <cmath>

#include "display_animation.h"
#include "display_renderer.h"
#include "display_types.h"

// ============================================================================
// BLE Advertising Animation
// ============================================================================

void BleAdvertisingAnimation::Entering() {
  auto* r = GetRenderer();
  r->ShowAdvertisingState();
}

AnimationAction BleAdvertisingAnimation::PlayNextFrame(int64_t dt_us) {
  return AnimationAction::Continue(WakeInterval::Never());
}

bool BleAdvertisingAnimation::CanBePlayed() const {
  const auto* data_source = GetDataSource();
  if (data_source == nullptr) {
    return true;
  }
  const auto family = data_source->GetProductFamily();
  return family != "CP02s";
}

// ============================================================================
// BLE Connected Animation
// ============================================================================

int64_t BleConnectedAnimation::GetStageDuration(size_t index) const {
  // Stage 0: 10 seconds ON, Stage 1: 60 seconds OFF
  return index == 0 ? 10000 * 1000 : 60000 * 1000;
}

void BleConnectedAnimation::RenderStage(size_t index) {
  auto* r = GetRenderer();
  if (index == 0) {
    r->ShowConnectedState();
    r->GetDriver()->SetMode(DisplayMode::MANUAL);
  } else {
    r->GetDriver()->SetMode(DisplayMode::OFF);
  }
}

// ============================================================================
// WiFi Connecting Animation
// ============================================================================

void WifiConnectingAnimation::Entering() {
  current_stage_ = Stage::GREETING;
  stage_elapsed_us_ = 0;
  GetRenderer()->ShowGreetingProgress(0.0f);
}

bool WifiConnectingAnimation::CanBePlayed() const {
  const auto* data_source = GetDataSource();
  if (data_source == nullptr) {
    return true;
  }
  const auto family = data_source->GetProductFamily();
  return family != "CP02" && family != "CP02s";
}

AnimationAction WifiConnectingAnimation::PlayNextFrame(int64_t dt_us) {
  stage_elapsed_us_ += dt_us;
  auto* r = GetRenderer();

  switch (current_stage_) {
    case Stage::GREETING: {
      float progress = static_cast<float>(stage_elapsed_us_) /
                       static_cast<float>(kGreetingDurationUs);
      if (progress >= 1.0f) {
        current_stage_ = Stage::WIFI_CONNECTING;
        stage_elapsed_us_ = 0;
        r->ShowWifiConnecting(0.0f);
      } else {
        r->ShowGreetingProgress(progress);
      }
      break;
    }

    case Stage::WIFI_CONNECTING: {
      // Infinite loop, waiting for external MQTT event to interrupt
      float progress = fmodf(static_cast<float>(stage_elapsed_us_) /
                                 static_cast<float>(kConnectingCycleDurationUs),
                             1.0f);
      r->ShowWifiConnecting(progress);
      break;
    }
  }

  return AnimationAction::Continue(ANIMATION_FRAME_INTERVAL_US);
}

void WifiConnectingAnimation::Exiting() { GetRenderer()->Clear(); }
