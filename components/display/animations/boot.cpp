#include "display_animation.h"
#include "display_renderer.h"
#include "sdkconfig.h"

// ============================================================================
// BootAppAnimation Implementation
// ============================================================================

void BootAppAnimation::RenderProgress(float progress) {
  GetRenderer()->ShowBootProgress(progress);
}

AnimationAction BootAppAnimation::OnComplete() {
  // Transition to DEVICE_INFO with IDLE fallback
  // CP02/CP02s: DEVICE_INFO rejected by CanBePlayed() -> fallback to IDLE
  // PA768s: DEVICE_INFO succeeds -> shows PSN/MAC
  return AnimationAction::Transition(AnimationType::DEVICE_INFO, false,
                                     AnimationType::IDLE, true);
}

// ============================================================================
// DeviceInfoAnimation Implementation
// ============================================================================

bool DeviceInfoAnimation::CanBePlayed() const {
  const auto* data_source = GetDataSource();
  if (data_source == nullptr) {
    return true;
  }
  const auto family = data_source->GetProductFamily();
  // Only play on PA768s (LVGL) - skip on CP02/CP02s (FPGA/GPU)
  return family == "PA768s";
}

int64_t DeviceInfoAnimation::GetStageDuration(size_t index) const {
  (void)index;  // Both stages have same duration
  return CONFIG_DEVICE_INFO_ANIMATION_PLAY_DURATION * 1000;
}

void DeviceInfoAnimation::RenderStage(size_t index) {
  if (index == 0) {
    ShowPSN();
  } else {
    ShowMac();
  }
}

void DeviceInfoAnimation::Exiting() {
  auto* r = GetRenderer();
  r->Clear();
}

void DeviceInfoAnimation::ShowPSN() { ShowInfo(true); }

void DeviceInfoAnimation::ShowMac() { ShowInfo(false); }

void DeviceInfoAnimation::ShowInfo(bool show_psn) {
  char info_text[64];
  const char* title = show_psn ? "PSN:" : "MAC Address";

  if (show_psn) {
    const std::string psn = data_source_->GetPSN();
    snprintf(info_text, sizeof(info_text), "%s %s\n%s %s",
             psn.substr(0, 4).c_str(), psn.substr(4, 4).c_str(),
             psn.substr(8, 4).c_str(), psn.substr(12, 4).c_str());
  } else {
    const std::array<uint8_t, 6> mac = data_source_->GetBleMac();
    snprintf(info_text, sizeof(info_text), "%02X:%02X:%02X:%02X:\n%02X:%02X",
             (mac)[0], (mac)[1], (mac)[2], (mac)[3], (mac)[4], (mac)[5]);
  }

  auto* r = GetRenderer();
  r->Clear();
  r->ShowInfoScreen(title, info_text);
}
