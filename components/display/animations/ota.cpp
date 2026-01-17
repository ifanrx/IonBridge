#include "display_animation.h"
#include "display_renderer.h"
#include "display_types.h"
#include "esp_timer.h"

// ============================================================================
// Start OTA Animation
// ============================================================================

void StartOTAAnimation::Entering() {
  auto* r = GetRenderer();
  r->Clear();

  const auto* data_source = GetDataSource();
  const auto family =
      data_source != nullptr ? data_source->GetProductFamily() : "";
  const bool show_image = family != "CP02";
  if (show_image) {
    r->ShowImage(IMAGE_PATH("ota/download"), Alignment::CENTER);
    expired_at_us_ = esp_timer_get_time() + (kDisplayDurationMs * 1000);
  } else {
    expired_at_us_ = esp_timer_get_time();
  }
}

AnimationAction StartOTAAnimation::PlayNextFrame(int64_t dt_us) {
  if (esp_timer_get_time() >= expired_at_us_) {
    return AnimationAction::Transition(AnimationType::OTA_ANIMATION, true);
  }

  int64_t remaining_us = expired_at_us_ - esp_timer_get_time();
  return AnimationAction::Continue(remaining_us > 0 ? remaining_us : 0);
}

void StartOTAAnimation::Exiting() {
  auto* r = GetRenderer();
  r->Clear();
}

// ============================================================================
// OTA Animation
// ============================================================================

uint8_t OTAAnimation::GetProgressPercentage(OTAStage stage) {
  static constexpr uint8_t kProgressMap[] = {
      33,   // DOWNLOAD_SW3566
      66,   // DOWNLOAD_FPGA
      100,  // DOWNLOAD_ESP32
      20,   // UPDATE_ERASE
      40,   // UPDATE_STAGE1
      60,   // UPDATE_STAGE2
      80,   // UPDATE_STAGE3
      100,  // UPDATE_STAGE4
      100,  // COMPLETE
  };

  uint8_t stage_index = static_cast<uint8_t>(stage);
  if (stage_index >= sizeof(kProgressMap) / sizeof(kProgressMap[0])) {
    return 100;
  }

  return kProgressMap[stage_index];
}

bool OTAAnimation::IsDownloadStage(OTAStage stage) {
  return stage == OTAStage::DOWNLOAD_SW3566 ||
         stage == OTAStage::DOWNLOAD_FPGA || stage == OTAStage::DOWNLOAD_ESP32;
}

bool OTAAnimation::IsUpdateStage(OTAStage stage) {
  return stage == OTAStage::UPDATE_ERASE || stage == OTAStage::UPDATE_STAGE1 ||
         stage == OTAStage::UPDATE_STAGE2 || stage == OTAStage::UPDATE_STAGE3 ||
         stage == OTAStage::UPDATE_STAGE4;
}

void OTAAnimation::SetStage(OTAStage stage) {
  OTAStage prev_stage = stage_;
  stage_ = stage;
  transition_.SetTarget(GetProgressPercentage(stage));

  auto* r = GetRenderer();

  if (IsDownloadStage(prev_stage) && IsUpdateStage(stage)) {
    r->ShowOTAProgress(0.0f, false);
  }

  auto* driver = r->GetDriver();
  driver->SetMode(DisplayMode::MANUAL);
  driver->SetIntensity(100);
}

void OTAAnimation::Entering() {
  auto* r = GetRenderer();
  stage_ = OTAStage::DOWNLOAD_SW3566;
  transition_.SetTarget(GetProgressPercentage(stage_));

  r->ShowOTAProgress(0.0f, true);
  auto* driver = r->GetDriver();
  driver->SetMode(DisplayMode::MANUAL);
  driver->SetIntensity(100);
}

AnimationAction OTAAnimation::PlayNextFrame(int64_t dt_us) {
  auto* r = GetRenderer();
  uint8_t current_percent = (uint8_t)transition_.Update(dt_us);

  float progress = static_cast<float>(current_percent) / 100.0f;
  r->ShowOTAProgress(progress, IsDownloadStage(stage_));

  if (!transition_.IsAnimating()) {
    transition_.SetCurrent(0);
    transition_.SetTarget(GetProgressPercentage(stage_));
  }

  return AnimationAction::Continue(animation_timing::kOtaUpdateIntervalUs);
}

void OTAAnimation::Exiting() {
  auto* r = GetRenderer();
  r->Clear();
}
