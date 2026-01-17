#include "display_animation.h"
#include "display_manager.h"
#include "display_renderer.h"

// Animation type names lookup table
static constexpr const char* kAnimationNames[] = {
    "BOOT_APP",       "BLE_ADVERTISING", "BLE_CONNECTED", "START_OTA",
    "OTA_ANIMATION",  "WIFI_CONNECTING", "IDLE",          "DEVICE_INSERTED",
    "DEVICE_REMOVED", "SCREEN_OFF",      "DEVICE_INFO",   "CHARGING",
    "POWER_CHART",    "DISPLAY_TEST",    "DATA_MATRIX"};

const char* AnimationType::toString() const {
  if (value < NO_ANIMATIONS) {
    return kAnimationNames[value];
  }
  return "UNKNOWN";
}

// ============================================================================
// BaseAnimation Implementation
// ============================================================================

void BaseAnimation::Init() {}

IDisplayRenderer* BaseAnimation::GetRenderer() const {
  return DisplayManager::GetInstance().GetRenderer();
}

AnimationAction BaseAnimation::TransitionToChargingOrIdle() const {
  if (data_source_->GetAttachedPortCount() > 0) {
    return AnimationAction::Transition(AnimationType::CHARGING, false);
  }
  return AnimationAction::Transition(AnimationType::IDLE, false);
}

// ============================================================================
// StagedAnimation Implementation
// ============================================================================

void StagedAnimation::Entering() {
  current_stage_ = 0;
  stage_elapsed_us_ = 0;

  if (GetStageCount() > 0) {
    RenderStage(0);
  }
}

AnimationAction StagedAnimation::PlayNextFrame(int64_t dt_us) {
  size_t stage_count = GetStageCount();
  if (stage_count == 0) {
    return AnimationAction::Complete();
  }

  stage_elapsed_us_ += dt_us;

  if (stage_elapsed_us_ >= GetStageDuration(current_stage_)) {
    current_stage_ = (current_stage_ + 1) % stage_count;
    stage_elapsed_us_ = 0;
    RenderStage(current_stage_);
  }

  int64_t remaining_us = GetStageDuration(current_stage_) - stage_elapsed_us_;
  return AnimationAction::Continue(remaining_us);
}

// ============================================================================
// ProgressAnimation Implementation
// ============================================================================

void ProgressAnimation::Entering() {
  elapsed_us_ = 0;
  RenderProgress(0.0f);
}

AnimationAction ProgressAnimation::PlayNextFrame(int64_t dt_us) {
  elapsed_us_ += dt_us;
  float progress =
      static_cast<float>(elapsed_us_) / static_cast<float>(GetDurationUs());
  if (progress >= 1.0f) {
    RenderProgress(1.0f);
    return OnComplete();
  }
  RenderProgress(progress);
  return AnimationAction::Continue(ANIMATION_FRAME_INTERVAL_US);
}

void ProgressAnimation::Exiting() { GetRenderer()->Clear(); }
