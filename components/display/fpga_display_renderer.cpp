#include "fpga_display_renderer.h"

#ifdef CONFIG_DISPLAY_DRIVER_FPGA

#include "display_animation.h"
#include "display_types.h"

FPGARenderer::FPGARenderer(FPGADisplayDriver* driver) : driver_(driver) {}

void FPGARenderer::Clear() { driver_->Clear(); }

void FPGARenderer::ShowPercentage(uint8_t percentage, ProgressStyle style) {
  driver_->SetPercentage(percentage);
  driver_->SetIntensity(display_renderer_utils::GetIntensityForStyle(style));
}

void FPGARenderer::ShowAdvertisingState() {
  driver_->SetPercentage(100);
  driver_->SetIntensity(100);
}

void FPGARenderer::ShowConnectedState() {
  driver_->SetPercentage(100);
  driver_->SetIntensity(100);
}

void FPGARenderer::ShowBootState() {
  driver_->SetIntensity(100);
  ShowBootProgress(0.0f);
}

void FPGARenderer::ShowBootProgress(float progress) {
  uint8_t percent = static_cast<uint8_t>(progress * 100.0f);
  if (percent > 100) percent = 100;
  driver_->SetPercentage(percent);
}

void FPGARenderer::ShowWifiConnecting(float progress) {
  // Pulsing effect: oscillate between 20-80% based on progress
  float pulse = progress < 0.5f ? progress * 2.0f : (1.0f - progress) * 2.0f;
  uint8_t percent = 20 + static_cast<uint8_t>(pulse * 60.0f);
  driver_->SetPercentage(percent);
}

void FPGARenderer::ShowOTAProgress(float progress, bool is_download_phase) {
  (void)is_download_phase;  // FPGA only shows progress bar
  uint8_t percent = static_cast<uint8_t>(progress * 100.0f);
  if (percent > 100) percent = 100;
  driver_->SetPercentage(percent);
}

WakeInterval FPGARenderer::ShowChargingStatus(IAnimationDataSource* data_source,
                                              int64_t dt_us) {
  (void)data_source;  // FPGA doesn't need port data
  (void)dt_us;
  driver_->SetMode(DisplayMode::POWER_METER);
  return WakeInterval::Never();  // Autonomous mode
}

WakeInterval FPGARenderer::ShowIdleState(IAnimationDataSource* data_source,
                                         int64_t dt_us) {
  (void)dt_us;
  if (!data_source) {
    return WakeInterval::CheckInterval();
  }
  uint8_t port_count = data_source->GetAttachedPortCount();

  if (port_count > 0) {
    driver_->SetMode(DisplayMode::POWER_METER);
    return WakeInterval::Never();
  }
  driver_->SetMode(DisplayMode::MANUAL);
  ShowPercentage(5, ProgressStyle::NORMAL);
  return WakeInterval::Polling(100000);  // 100ms idle poll
}

IDisplayDriver* FPGARenderer::GetDriver() { return driver_; }

#endif  // CONFIG_DISPLAY_DRIVER_FPGA
