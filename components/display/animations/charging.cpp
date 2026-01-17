#include <algorithm>
#include <cstdio>

#include "display_animation.h"
#include "display_renderer.h"
#include "display_types.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char* TAG = "ChargingAnimation";

#define POWER_CHART_PORT_UPDATE_MS 500
#define POWER_CHART_POWER_UPDATE_MS 1000

// ============================================================================
// Device Inserted Animation
// ============================================================================

void DeviceInsertedAnimation::Entering() {
  stored_port_id_ = data_source_->GetLastInsertedPortId();
  ProgressAnimation::Entering();
}

bool DeviceInsertedAnimation::CanBePlayed() const {
  const auto* data_source = GetDataSource();
  if (data_source == nullptr) {
    return true;
  }
  const auto family = data_source->GetProductFamily();
  return family == "CP02s";
}

AnimationAction DeviceInsertedAnimation::PlayNextFrame(int64_t dt_us) {
  uint8_t current_port = data_source_->GetLastInsertedPortId();
  if (current_port != stored_port_id_) {
    return AnimationAction::Transition(AnimationType::DEVICE_INSERTED, false);
  }

  return ProgressAnimation::PlayNextFrame(dt_us);
}

int64_t DeviceInsertedAnimation::GetDurationUs() const {
  return 2000 * 1000;  // 2 seconds
}

void DeviceInsertedAnimation::RenderProgress(float progress) {
  GetRenderer()->ShowGreetingProgress(progress);
}

AnimationAction DeviceInsertedAnimation::OnComplete() {
  return AnimationAction::Transition(AnimationType::CHARGING, false);
}

// ============================================================================
// Device Removed Animation
// ============================================================================

void DeviceRemovedAnimation::Entering() {
  elapsed_us_ = 0;
  auto* r = GetRenderer();
  r->Clear();
  r->ShowImage(IMAGE_PATH("unattached"), Alignment::CENTER);
}

AnimationAction DeviceRemovedAnimation::PlayNextFrame(int64_t dt_us) {
  elapsed_us_ += dt_us;
  int64_t duration_us = CONFIG_DEVICE_REMOVED_ANIMATION_PLAY_DURATION * 1000;
  if (elapsed_us_ >= duration_us) {
    return TransitionToChargingOrIdle();
  }
  return AnimationAction::Continue(duration_us - elapsed_us_);
}

void DeviceRemovedAnimation::Exiting() {
  auto* r = GetRenderer();
  r->Clear();
}

bool DeviceRemovedAnimation::CanBePlayed() const {
  const auto* data_source = GetDataSource();
  if (data_source == nullptr) {
    return true;
  }
  const auto family = data_source->GetProductFamily();
  // Only play on CP02 and PA768s (allowlist pattern)
  return family == "CP02" || family == "PA768s";
}

// ============================================================================
// Charging Animation
// ============================================================================

void ChargingAnimation::Entering() {
  size_t count = data_source_->GetPortsData().size();
  if (count == 0) {
    ESP_LOGW(TAG, "No ports attached, skipping charging animation");
    elapsed_us_ = 0;
    last_port_count_ = 0;
    return;
  }

  last_port_count_ = static_cast<uint8_t>(count);
  elapsed_us_ = 0;
  RenderChargingStatus();
}

AnimationAction ChargingAnimation::PlayNextFrame(int64_t dt_us) {
  elapsed_us_ += dt_us;

  uint8_t current_count =
      static_cast<uint8_t>(data_source_->GetPortsData().size());
  if (current_count == 0) {
    GetRenderer()->Clear();
    return AnimationAction::Transition(AnimationType::IDLE, false);
  }

  // Renderer returns wake interval (FPGA: never wake, GPU: periodic)
  WakeInterval wake = RenderChargingStatus(dt_us);

  // Never wake = autonomous mode (e.g., FPGA POWER_METER)
  if (wake == WakeInterval::Never()) {
    return AnimationAction::Continue(WakeInterval::Never());
  }

  // GPU: check display duration timeout
  constexpr int64_t kDisplayDurationUs = 10 * 1000 * 1000;
  if (elapsed_us_ >= kDisplayDurationUs) {
    return AnimationAction::Transition(AnimationType::IDLE, false);
  }

  // Wake at next interval boundary
  return AnimationAction::Continue(wake.us - (elapsed_us_ % wake.us));
}

void ChargingAnimation::Exiting() {
  auto* r = GetRenderer();
  r->Clear();
}

WakeInterval ChargingAnimation::RenderChargingStatus(int64_t dt_us) {
  auto* r = GetRenderer();
  return r->ShowChargingStatus(data_source_, dt_us);
}

// ============================================================================
// Power Chart Animation
// ============================================================================

void PowerChartAnimation::Entering() {
  port_id_ = 0;
  elapsed_us_ = 0;
  port_update_elapsed_us_ = 0;
  power_update_elapsed_us_ = 0;

  UpdatePowerData();
  RenderChart();
}

AnimationAction PowerChartAnimation::PlayNextFrame(int64_t dt_us) {
  elapsed_us_ += dt_us;
  port_update_elapsed_us_ += dt_us;
  power_update_elapsed_us_ += dt_us;

  int64_t duration_us = CONFIG_POWER_CHART_ANIMATION_PLAY_DURATION * 1000;
  if (elapsed_us_ >= duration_us) {
    return TransitionToChargingOrIdle();
  }

  bool should_render = false;

  if (port_update_elapsed_us_ >= POWER_CHART_PORT_UPDATE_MS * 1000) {
    port_update_elapsed_us_ = 0;
    should_render = true;
  }

  if (power_update_elapsed_us_ >= POWER_CHART_POWER_UPDATE_MS * 1000) {
    power_update_elapsed_us_ = 0;
    UpdatePowerData();
    should_render = true;
  }

  if (should_render) {
    RenderChart();
  }

  int64_t remaining_duration = duration_us - elapsed_us_;
  int64_t remaining_port =
      POWER_CHART_PORT_UPDATE_MS * 1000 - port_update_elapsed_us_;
  int64_t remaining_power =
      POWER_CHART_POWER_UPDATE_MS * 1000 - power_update_elapsed_us_;

  int64_t next_wake =
      std::min({remaining_duration, remaining_port, remaining_power});
  return AnimationAction::Continue(next_wake);
}

void PowerChartAnimation::Exiting() {
  auto* r = GetRenderer();
  r->Clear();
}

void PowerChartAnimation::RenderChart() {
  auto* r = GetRenderer();

  const auto& ports_data = data_source_->GetPortsData();

  if (ports_data.empty()) {
    r->Clear();
    return;
  }

  uint16_t total_power = data_source_->GetTotalWatts();

  port_id_ = port_id_ % ports_data.size();
  uint8_t port_power = ports_data[port_id_].power_w;
  uint8_t display_port_id = ports_data[port_id_].port_id;

  r->Clear();

  TextStyle style_total;
  style_total.size = TextStyle::Size::SMALL;
  style_total.align = Alignment::LEFT;
  char total_buf[16];
  snprintf(total_buf, sizeof(total_buf), "%dW", total_power);
  r->ShowText(total_buf, style_total);

  TextStyle style_port;
  style_port.size = TextStyle::Size::MEDIUM;
  style_port.align = Alignment::RIGHT;
  char port_buf[16];
  snprintf(port_buf, sizeof(port_buf), "%s:%dW",
           display_renderer_utils::GetPortDisplayName(display_port_id).c_str(),
           port_power);
  r->ShowText(port_buf, style_port);

  r->ShowPowerChart(power_data_, power_data_count_);

  port_id_ = (port_id_ + 1) % ports_data.size();
}

void PowerChartAnimation::UpdatePowerData() {
  std::vector<PowerDataPoint> historical_data =
      data_source_->GetHistoricalPowerData(POWER_CHART_POINT_COUNT);

  power_data_count_ = historical_data.size();
  if (power_data_count_ > POWER_CHART_POINT_COUNT) {
    power_data_count_ = POWER_CHART_POINT_COUNT;
  }

  for (size_t i = 0; i < power_data_count_; i++) {
    power_data_[i] = historical_data[i];
  }
}

bool PowerChartAnimation::CanBePlayed() const {
  const auto* data_source = GetDataSource();
  if (data_source == nullptr) {
    return true;
  }
  const auto family = data_source->GetProductFamily();
  // Only play on PA768s (LVGL display)
  return family == "PA768s";
}

AnimationType PowerChartAnimation::GetFallbackAnimation() const {
  if (data_source_ && data_source_->GetAttachedPortCount() > 0) {
    return AnimationType::CHARGING;
  }
  return AnimationType::IDLE;
}
