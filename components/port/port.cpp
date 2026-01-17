#include "port.h"

#include <sys/param.h>
#include <sys/types.h>

#include <algorithm>
#include <cinttypes>
#include <cstdint>
#include <cstring>
#include <string>

#include "data_types.h"
#include "display_manager.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_log_buffer.h"
#include "esp_log_level.h"
#include "esp_timer.h"
#include "ionbridge.h"
#include "machine_info.h"
#include "port_data.h"
#include "port_state.h"
#include "rpc.h"
#include "sdkconfig.h"
#include "utils.h"

#ifdef CONFIG_ENABLE_PROGRAMMER
#include <cstddef>

#include "firmware_programmer.h"
#include "sw3566_data_types.h"
#endif

#define FC_HANDSHAKE_USAGE_TIME 5e6
#define LIMITED_PORT_POWER_BUDGET CONFIG_LIMITED_PORT_POWER_BUDGET
#define PORT_HISTORICAL_DATA_INTERVAL \
  (CONFIG_PORT_HISTORICAL_DATA_SAMPLE_RATE) * 1e6
#define PORT_CHECKING_TIMER_MS CONFIG_PORT_CHECKING_TIMER_MS

static const char *TAG = "Port";

Port::Port(uint8_t port_id)
    : id_(port_id),
      historical_stats_(PORT_HISTORICAL_DATA_SIZE),
      historical_details_(PORT_HISTORICAL_DETAILS_SIZE),
      checking_timer_name_("port_watchdog_" + std::to_string(port_id)),
      detaching_timer_name_("port_detach_" + std::to_string(port_id)) {
  data_ = PortPowerData();
  details_ = PortDetails();
  average_data_ = PortAverageData();
  stats_last_updated_at_ = esp_timer_get_time();
  state_ = &PortBootingState::GetInstance();
#ifdef CONFIG_MCU_MODEL_SW3566
  system_flags_ = {
      .commit = 0,
      .enable_flash_cache = true,
      .port_type = static_cast<uint8_t>(PortType::PORT_TYPE_C),
      .enable_cable_compensation = true,
      .cable_compensation = 1,
      .fixed_voltage_offset = 0,
      .unused = 0,
  };
#endif
  this->Reset();
}

bool Port::Initialize(bool active, uint8_t initial_power_budget) {
  if (!active) {
    SetState(PortStateType::DEAD);
    return true;
  }
  esp_err_t ret __attribute__((unused));
  PowerFeatures mcu_features = {};  // Declare before any goto
#ifdef CONFIG_MCU_MODEL_SW3566
  ESP_GOTO_ON_ERROR(rpc::mcu::set_system_flags(id_, &system_flags_), RECOVER,
                    TAG, "rpc::mcu::set_system_flags(%d)", id_);
#if defined(CONFIG_DEVICE_HUMMING_BOARD) && CONFIG_SW3566_TYPE_A_PORT == -1
  if (id_ == 0) {
    system_flags_.port_type = static_cast<uint8_t>(PortType::PORT_TYPE_C);
    system_flags_.commit = 0x55;
    ESP_GOTO_ON_ERROR(rpc::mcu::set_system_flags(id_, &system_flags_), RECOVER,
                      TAG, "rpc::mcu::set_system_flags(%d)", id_);
    system_flags_.commit = 0;
  }
#endif
#elif defined(CONFIG_MCU_MODEL_SW3516) || defined(CONFIG_MCU_MODEL_SW3536)
  ESP_GOTO_ON_ERROR(rpc::mcu::initialize_port(id_), RECOVER, TAG,
                    "Unable to initialize port %d", id_);
#endif
  ESP_GOTO_ON_ERROR(rpc::mcu::get_power_features(id_, &mcu_features), RECOVER,
                    TAG, "rpc::mcu::get_power_features(%d)", id_);
  ESP_LOGD(TAG, "Port features: ");
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, &mcu_features, sizeof(mcu_features),
                           ESP_LOG_DEBUG);

  if (id_ == 1 || id_ == 3) {
    // Check hardware version
    const std::string &hw_version = MachineInfo::GetInstance().GetHwRev();
    // pcb layout issue, enable limited current mode on port 1 and 3
    if (hw_version.compare("B0P0") == 0 || hw_version.compare("PVT1") == 0 ||
        hw_version.compare("PVT2") == 0) {
      ESP_GOTO_ON_ERROR(EnableLimitedCurrentMode(), RECOVER, TAG,
                        "EnableLimitedCurrentMode(%d)", id_);
      ESP_LOGW(TAG, "Limited current mode enabled for port %d", id_);
    }
  }

#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
  ESP_GOTO_ON_ERROR(rpc::mcu::set_subscription(id_, subscriptions_), RECOVER,
                    TAG, "Unable to subscribe to port %d", id_);
#endif
  initial_power_budget_ = initial_power_budget;
  initialized_ = true;
  // Just verify MCU is responsive, don't store features
  this->ResetConfig();
  this->Reset();
  this->SetState(PortStateType::ACTIVE);
  return ApplyMaxPowerBudget(initial_power_budget_);
RECOVER:
  this->SetState(PortStateType::CHECKING);
  return false;
}

void Port::SetState(PortState *state) {
#ifdef CONFIG_ENABLE_PROGRAMMER
  FirmwareProgrammer &prog_mgr = FirmwareProgrammer::GetInstance();
  if (prog_mgr.IsActivated()) {
    ESP_LOGW(TAG, "Port %d is in programming mode, state change to %s ignored",
             id_, state_->ToString().c_str());
    return;
  }
#endif

  if (state_->Type() == state->Type()) {
    return;
  }
  if (state_->Type() == PortStateType::DEAD) {
    return;
  }
  ESP_LOGI(TAG, "Port %d state change: %s -> %s", id_,
           state_->ToString().c_str(), state->ToString().c_str());
  last_state_ = state_->Type();
  state_->Exiting(*this);
  state_ = state;
  state_->Entering(*this);

  PortStateType state_type = state->Type();
  if (state_type >= 0 && state_type < PortStateType::COUNT &&
      state_callbacks_[state_type]) {
    state_callbacks_[state_type](*this);
  }
}

void Port::SetState(PortStateType type) {
#ifdef CONFIG_ENABLE_PROGRAMMER
  FirmwareProgrammer &prog_mgr = FirmwareProgrammer::GetInstance();
  if (prog_mgr.IsActivated()) {
    ESP_LOGW(TAG, "Port %d is in programming mode, state change to %s ignored",
             id_, state_->ToString().c_str());
    return;
  }
#endif

  switch (type) {
    case PortStateType::ACTIVE:
      SetState(&PortActiveState::GetInstance());
      break;
    case PortStateType::INACTIVE:
      SetState(&PortInactiveState::GetInstance());
      break;
    case PortStateType::ATTACHED:
      SetState(&PortAttachedState::GetInstance());
      break;
    case PortStateType::DETACHING:
      SetState(&PortDetachingState::GetInstance());
      break;
    case PortStateType::OPENING:
      SetState(&PortOpeningState::GetInstance());
      break;
    case PortStateType::CLOSING:
      SetState(&PortClosingState::GetInstance());
      break;
    case PortStateType::OVER_TEMP_WARNING:
      SetState(&PortOverTempWarningState::GetInstance());
      break;
    case PortStateType::OVER_TEMP_ALERT:
      SetState(&PortOverTempAlertState::GetInstance());
      break;
    case PortStateType::COOLING:
      SetState(&PortCoolingState::GetInstance());
      break;
    case PortStateType::CHECKING:
      SetState(&PortCheckingState::GetInstance());
      break;
    case PortStateType::RECOVERING:
      SetState(&PortRecoveringState::GetInstance());
      break;
    case PortStateType::DEAD:
      SetState(&PortDeadState::GetInstance());
      break;
    case PortStateType::POWER_LIMITING:
      SetState(&PortPowerLimitingState::GetInstance());
      break;
    default:
      ESP_LOGE(TAG, "Invalid port state type: %d", type);
      break;
  }
}

bool Port::Close() {
  // Special handling for DEAD ports - they're already effectively closed
  if (state_->Type() == PortStateType::DEAD) {
    ESP_LOGD(TAG, "Port %d is DEAD, treating as already closed", id_);
    return true;
  }

  if (IsOpen()) {
    SetState(PortStateType::CLOSING);
    return true;
  }

  // Port is already closed (INACTIVE) or in process of closing
  if (state_->Type() == PortStateType::CLOSING ||
      state_->Type() == PortStateType::INACTIVE) {
    return true;  // Already in desired state
  }

  ESP_LOGW(TAG, "Cannot close port %d, current state: %s", id_,
           state_->ToString().c_str());
  return false;
}

bool Port::Open() {
  // Special handling for DEAD ports - cannot be opened
  if (state_->Type() == PortStateType::DEAD) {
    ESP_LOGW(TAG, "Cannot open port %d - port is DEAD", id_);
    return false;  // DEAD ports cannot be opened
  }

  if (IsClose()) {
    SetState(PortStateType::OPENING);
    return true;
  }

  // Port is already open (ACTIVE/ATTACHED) or in process of opening
  if (state_->Type() == PortStateType::OPENING ||
      state_->Type() == PortStateType::ACTIVE ||
      state_->Type() == PortStateType::ATTACHED) {
    return true;  // Already in desired state
  }

  ESP_LOGW(TAG, "Cannot open port %d, current state: %s", id_,
           state_->ToString().c_str());
  return false;
}

bool Port::Toggle() {
  if (IsClose()) {
    SetState(PortStateType::OPENING);
    return true;
  } else if (IsOpen()) {
    SetState(PortStateType::CLOSING);
    return true;
  }
  return false;
}

esp_err_t Port::FetchDetails(PortDetails *details) {
  if (!Activated()) {
    return ESP_ERR_INVALID_STATE;
  }

  PortDetails fetched;
  esp_err_t err = rpc::mcu::get_port_details(id_, &fetched);
  if (err != ESP_OK) {
    return err;
  }

  UpdateData(fetched);
  if (details) {
    memcpy(details, &details_, sizeof(PortDetails));
  }
  return ESP_OK;
}

void Port::UpdateData(const PortDetails &details) {
  if (!Activated()) {
    return;
  }

  updated_at_ = esp_timer_get_time();

  ESP_LOGD(TAG, "#PORTSTATS[%d] T=%dC, I=%dmA, V=%dmV, connected=%d", id_,
           details.die_temperature, details.iout_value, details.vout_value,
           details.connected);

  // Only trigger attach when there's actual charging:
  // 1. Fast charging protocol negotiated (FC_QC2 through FC_PD_AVS), OR
  // 2. Basic 5V charging with actual current draw (FC_None but iout > 0)
  // This prevents animation when only a cable is plugged in without a device
  bool has_fast_charging =
      details.fc_protocol > FC_None && details.fc_protocol < FC_NOT_CHARGING;
  bool has_basic_charging =
      details.fc_protocol == FC_None && details.iout_value > 0;
  bool is_charging = has_fast_charging || has_basic_charging;
  if (!attached_ && details.connected && is_charging) {
    this->Attach();
  } else if (attached_ && !details.connected) {
    // Only trigger Detach() if not already in DETACHING state
    // (DETACHING state will handle reconnection detection and timer expiry)
    if (!IsDetaching()) {
      this->Detach();
    }
  }

  details_ = ValidateAndSanitizeDetails(details);
  if (details.connected) {
    data_.SetFCProtocol(static_cast<uint8_t>(details.fc_protocol));
    data_.SetCurrent(details.iout_value);
    data_.SetVoltage(details.vout_value);
  }
  data_.SetTemperature(details.die_temperature);
  data_.SetVinValue(details.vin_value);

  // Update average data
  average_data_.IncrementalAverageData(data_);

  // Push to historical data
  int64_t now = esp_timer_get_time();
  if (now - stats_last_updated_at_ > PORT_HISTORICAL_DATA_INTERVAL) {
    stats_last_updated_at_ = now;
    historical_stats_.Push(PortStatsData(average_data_));
    historical_details_.Push(details);
    average_data_.Reset();
  }
}

void Port::SetInactiveHistoricalData() {
  // append zero data to historical data while port is inactive
  int64_t now = esp_timer_get_time();
  if (now - stats_last_updated_at_ <= PORT_HISTORICAL_DATA_INTERVAL) {
    return;
  }

  stats_last_updated_at_ = now;
  average_data_.Reset();
  historical_stats_.Push(PortStatsData(average_data_));
  historical_details_.Push(details_);
}

void Port::Reset() {
  attached_ = false;
  attached_at_ = -1;
  updated_at_ = -1;
  memset(&pd_status_, 0, sizeof(pd_status_));
  memset(&details_, 0, sizeof(details_));
  details_.fc_protocol = FC_NOT_CHARGING;
  data_.Reset();
}

esp_err_t Port::ResetState() {
  SetState(PortStateType::INACTIVE);
  esp_err_t err = rpc::mcu::port_disconnect(id_);
  if (err != ESP_OK) {
    SetState(PortStateType::CHECKING);
    return err;
  }
  err = rpc::mcu::port_connect(id_);
  if (err != ESP_OK) {
    SetState(PortStateType::CHECKING);
    return err;
  }
  Reset();
  SetState(PortStateType::ACTIVE);
  ESP_LOGI(TAG, "Port %d state reset completed", id_);
  return ESP_OK;
}

void Port::EnsureBudgetInRange(uint8_t *power) const {
  uint8_t rounded_power = *power;
  *power = std::min(std::max(rounded_power, MinPowerCap()), MaxPowerCap());
}

esp_err_t Port::SetMaxPowerBudget(uint8_t maxPowerBudget, uint8_t watermark,
                                  bool forceRebroadcast,
                                  bool enterCheckingOnFailed) {
  esp_err_t ret = ESP_OK;
  if (Abnormal()) {
    ESP_LOGE(TAG, "Cannot set max power budget for abnormal port %d", id_);
    return ESP_ERR_INVALID_STATE;
  }

  int64_t now = esp_timer_get_time();

  constexpr int64_t kMinUpdateIntervalUs = 1e6;  // 1 second in microseconds
  // Skip update if not forced, and called too soon
  bool skip_update =
      !forceRebroadcast && (now - cap_updated_at_ < kMinUpdateIntervalUs);
  if (skip_update) {
    ESP_LOGD(TAG, "Port %d: SetMaxPowerBudget %dW called too soon, ignoring",
             id_, maxPowerBudget);
    return ESP_OK;
  }
  cap_updated_at_ = now;

  EnsureBudgetInRange(&maxPowerBudget);
  if (watermark == 0 || watermark > maxPowerBudget) {
    watermark = maxPowerBudget;
  }
  if (!forceRebroadcast && maxPowerBudget == power_budget_ &&
      watermark == power_budget_watermark_) {
    ESP_LOGD(TAG, "Port %d: Power budget already set to %dW", id_,
             power_budget_);
    return ESP_OK;
  }

  ESP_LOGD(TAG, "Port %d power budget: %dW => %dW, wmk: %dW => %dW", id_,
           power_budget_, maxPowerBudget, power_budget_watermark_, watermark);
  ESP_GOTO_ON_ERROR(
      rpc::mcu::set_max_power_budget(id_, maxPowerBudget, watermark,
                                     forceRebroadcast, &power_budget_),
      RESET_STATE, TAG, "rpc::mcu::set_max_power_budget(%d, %d)", id_,
      maxPowerBudget);
#ifndef CONFIG_MCU_MODEL_SW3566
  ESP_GOTO_ON_ERROR(rpc::mcu::get_max_power_budget(id_, &power_budget_),
                    RESET_STATE, TAG, "rpc::mcu::get_max_power_budget(%d)",
                    id_);
#endif

  power_budget_watermark_ = watermark;
  ESP_LOGD(TAG, "Port %d power budget changed: %dW", id_, power_budget_);
  return ESP_OK;

RESET_STATE:
  if (ret != ESP_OK && enterCheckingOnFailed) {
    this->SetState(PortStateType::CHECKING);
  }
  return ret;
}

bool Port::DecreasePowerBudget(uint8_t power) {
  // Ensure we don't go below the minimum power budget
  power = (power + 4) / 5 * 5;
  uint8_t new_budget =
      (power_budget_ > power) ? power_budget_ - power : this->MinPowerCap();
  return SetMaxPowerBudget(new_budget) == ESP_OK;
}

bool Port::IncreasePowerBudget(uint8_t power) {
  // Calculate new budget and ensure it does not exceed the max
  power = (power + 4) / 5 * 5;
  uint8_t new_budget = power_budget_ + power;
  if (new_budget <= MaxPowerCap()) {
    return SetMaxPowerBudget(new_budget) == ESP_OK;
  }
  return false;
}

bool Port::ApplyMaxPowerBudget(uint8_t max_power_budget) {
  esp_err_t __attribute__((unused)) ret;

  if (this->Abnormal()) {
    ESP_LOGE(TAG, "Cannot apply max power budget for abnormal port %d", id_);
    return false;
  }

  ESP_GOTO_ON_ERROR(this->SetMaxPowerBudget(max_power_budget, 0, true, false),
                    RESET_STATE, TAG,
                    "Failed to set max power budget for port %d", id_);

  // TODO: Add comments to explain the logic here
  if (this->IsOpen() || this->Closing() || this->IsChecking()) {
    // Trigger a reconnect to ensure the power budget is correctly applied
    ESP_GOTO_ON_ERROR(rpc::mcu::port_disconnect(id_), RESET_STATE, TAG,
                      "rpc::mcu::port_disconnect(%d)", id_);
    ESP_GOTO_ON_ERROR(rpc::mcu::port_connect(id_), RESET_STATE, TAG,
                      "rpc::mcu::port_connect(%d)", id_);
  }

  initial_power_budget_ = power_budget_;
  ESP_LOGI(TAG, "Port %d: Initial power budget: %dW", id_,
           initial_power_budget_);
  return true;

RESET_STATE:
  if (initial_power_budget_ == 0xFF) {
    // Failed to apply first power budget, mark port as dead
    this->SetState(PortStateType::DEAD);
  } else {
    this->SetState(PortStateType::CHECKING);
  }
  return false;
}

uint32_t Port::GetChargingDurationSeconds() const {
  if (attached_at_ == -1) {
    return 0;
  }
  uint32_t now = esp_timer_get_time() / 1e3;
  if (now < attached_at_) {
    return 0;
  }
  return (now - attached_at_) / 1e3;
}

bool Port::RestoreInitialPowerBudget() {
  if (initial_power_budget_ == 0xFF) {
    return false;
  }
  ESP_LOGD(TAG, "Port %d: Restoring to initial power budget %dW", id_,
           initial_power_budget_);
  return SetMaxPowerBudget(initial_power_budget_) == ESP_OK;
}

esp_err_t Port::Reconnect() {
  esp_err_t ret;
  ESP_GOTO_ON_ERROR(rpc::mcu::port_disconnect(id_), RESET_STATE, TAG,
                    "rpc::mcu::port_disconnect(%d)", id_);
  ESP_GOTO_ON_ERROR(rpc::mcu::port_connect(id_), RESET_STATE, TAG,
                    "rpc::mcu::port_connect(%d)", id_);
  return ESP_OK;
RESET_STATE:
  if (initial_power_budget_ == 0xFF) {
    // Failed to apply first power budget, mark port as dead
    this->SetState(PortStateType::DEAD);
  } else {
    this->SetState(PortStateType::CHECKING);
  }
  return ret;
}

esp_err_t Port::UpdatePowerFeatures(const PowerFeatures &features) {
  esp_err_t err = rpc::mcu::set_power_features(id_, features);
  if (err != ESP_OK) {
    ESP_ERROR_COMPLAIN(err, "rpc::mcu::set_power_features(%d)", id_);
    this->SetState(PortStateType::CHECKING);
    return err;
  }

  config_.features = features;
  return ESP_OK;
}

esp_err_t Port::Shutdown() {
#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
  ESP_RETURN_ON_ERROR(rpc::fpga::toggle_mcu_gpio(id_), TAG,
                      "toggle_mcu_gpio: %d", id_);
#endif
  this->SetState(PortStateType::DEAD);
  return ESP_OK;
}

esp_err_t Port::EnableLimitedCurrentMode() {
  config_.features.LimitedCurrentMode = true;
  return rpc::mcu::set_power_features(id_, config_.features);
}

esp_err_t Port::DisableLimitedCurrentMode() {
  config_.features.LimitedCurrentMode = false;
  return rpc::mcu::set_power_features(id_, config_.features);
}

void Port::EnterPowerLimiting() {
  ESP_LOGW(TAG, "Port %d enter power limiting", id_);
  esp_err_t ret = ESP_OK;
  uint8_t max_power_budget = MinPowerCap();
  ESP_GOTO_ON_ERROR(EnableLimitedCurrentMode(), RESET_STATE, TAG,
                    "EnableLimitedCurrentMode(%d)", id_);
  ESP_GOTO_ON_ERROR(SetMaxPowerBudget(max_power_budget, 0, true), RESET_STATE,
                    TAG, "SetMaxPowerBudget(%d, %d) failed", id_,
                    max_power_budget);
  SetState(PortStateType::POWER_LIMITING);
  return;

RESET_STATE:
  ESP_LOGE(TAG, "Port %d enter power limiting failed: %d", id_, ret);
#ifdef CONFIG_MCU_MODEL_SW3566
  ESP_ERROR_COMPLAIN(rpc::fpga::toggle_mcu_gpio(id_), "toggle_mcu_gpio: %d",
                     id_);
#endif
  SetState(PortStateType::DEAD);
  return;
}

void Port::ExitPowerLimiting() {
  ESP_LOGW(TAG, "Port %d exit power limiting", id_);
  esp_err_t ret __attribute__((unused)) = ESP_OK;
  if (Dead()) {
    Reboot();
    return;
  }
  if (!IsPowerLimiting()) {
    return;
  }
  ESP_GOTO_ON_ERROR(DisableLimitedCurrentMode(), DEAD_PORT, TAG,
                    "DisableLimitedCurrentMode(%d)", id_);
  SetState(PortStateType::ACTIVE);
  return;

DEAD_PORT:
  SetState(PortStateType::DEAD);
  return;
}

void Port::Reboot() {
  ESP_LOGW(TAG, "Port %d reboot", id_);
#ifdef CONFIG_MCU_MODEL_SW3566
  esp_err_t ret = ESP_OK;
  KeepAliveStatus status;
  ESP_GOTO_ON_ERROR(rpc::mcu::boot(id_), DEAD_PORT, TAG,
                    "Unable to boot port %d", id_);
  // Wait for user application which will be booted
  DELAY_MS(100);
  ESP_GOTO_ON_ERROR(rpc::mcu::keep_alive(id_, &status), DEAD_PORT, TAG,
                    "Port %d keep-alive failed after bootup", id_);
  ESP_GOTO_ON_FALSE(status == KeepAliveStatus::USER_APPLICATION_ALIVE,
                    ESP_ERR_INVALID_STATE, DEAD_PORT, TAG,
                    "Port %d isn't in user application after bootup", id_);
#endif

  Initialize(true, PORT_MIN_POWER);  // After reboot, reinitialize the port
  return;

#ifdef CONFIG_MCU_MODEL_SW3566
DEAD_PORT:
  ESP_LOGE(TAG, "Port %d boot failed: %d", id_, ret);
  // Port is at unknown state, mark it as dead
  SetState(PortStateType::DEAD);
  return;
#endif
}

bool Port::IsAdjustable() const {
  bool adjustable_ = false;
  if (!attached_) {
    adjustable_ = true;
  } else {
    uint8_t fc_protocal = data_.GetFCProtocol();
    if (fc_protocal == FC_UFCS ||
        (FC_NOT_CHARGING > fc_protocal && fc_protocal >= FC_PD_Fix5V)) {
      adjustable_ = true;
    }
  }
  return adjustable_;
}

esp_err_t Port::SubscribePDPcapStreaming(bool subscribe) {
#ifdef CONFIG_MCU_MODEL_SW3566
  if (subscriptions_.EnablePDPcapStreaming == subscribe) {
    ESP_LOGD(TAG, "Port %d: PDCap streaming is already %s", id_,
             subscribe ? "enabled" : "disabled");
    return ESP_OK;
  }
  subscriptions_.EnablePDPcapStreaming = subscribe;
  ESP_RETURN_ON_ERROR(rpc::mcu::set_subscription(id_, subscriptions_), TAG,
                      "Unable to %s to port %d",
                      subscribe ? "subscribe" : "unsubscribe", id_);
  return ESP_OK;
#else
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

void Port::ResetConfig() {
  memset(&config_, 0, sizeof(PortConfig));
  config_.version = 2;
  // Set default feature values
  config_.features = {
      .EnableTfcp = false,
      .EnablePe = true,
      .EnableQc2p0 = true,
      .EnableQc3p0 = true,
#ifdef CONFIG_MCU_MODEL_SW3516
      .EnableQc3plus = false,
#else
      .EnableQc3plus = !this->IsTypeA(),
#endif
      .EnableAfc = true,
      .EnableFcp = true,
      .EnableHvScp = true,
      .EnableLvScp = true,
      .EnableSfcp = true,
#ifdef CONFIG_MCU_MODEL_SW3516
      .EnableApple = false,
      .EnableSamsung = false,
#else
      .EnableApple = true,
      .EnableSamsung = true,
#endif
      .EnableUfcs = false,
      .EnablePd = !this->IsTypeA(),
      .EnablePdCompatMode = false,
      .LimitedCurrentMode = false,
      .EnablePdLVPPS = !this->IsTypeA(),
#ifdef CONFIG_MCU_MODEL_SW3516
      .EnablePdEPR = false,
#else
      .EnablePdEPR = !this->IsTypeA(),
#endif
      .EnablePd5V5A = false,
      .EnablePdHVPPS = !this->IsTypeA(),
      .EnablePdHardResetOnRefusal = false,
      .EnablePdSPRAVS = false,
      .reserved0 = 0,
      .reserved1 = 0,
  };
}

void Port::ResetToInitialConfig() {
  if (initial_config_saved_) {
    ESP_LOGI(TAG, "Port %d: Resetting to initial config", id_);
    config_ = initial_config_;
  } else {
    ESP_LOGW(TAG, "Port %d: No initial config saved, using defaults", id_);
    ResetConfig();
  }
}

PowerFeatures Port::MigrateFeatures(const PortConfig &config) const {
  PowerFeatures features = {};
  memcpy(&features, &config.features, sizeof(PowerFeatures));
  // CL mode shouldn't change by port config
  features.LimitedCurrentMode = config_.features.LimitedCurrentMode;
  features.reserved0 = 0;
  features.reserved1 = 0;

  if (config.version == 0) {
    features.EnablePdLVPPS = !this->IsTypeA();
    features.EnablePdHVPPS = !this->IsTypeA();
    features.EnablePdEPR = !this->IsTypeA();
    features.EnablePd5V5A = false;
  }
  if (config.version < 2) {
    features.EnablePdHardResetOnRefusal = false;
    features.EnablePdSPRAVS = false;
  }

  features.EnablePd &= !this->IsTypeA();
  features.EnablePdCompatMode &= features.EnablePd;
  features.EnablePdLVPPS &= features.EnablePd;
  features.EnablePdHVPPS &= features.EnablePd;
  features.EnablePdEPR &= features.EnablePd;
  features.EnablePd5V5A &= features.EnablePd;
  features.EnablePdHardResetOnRefusal &= features.EnablePd;
  features.EnablePdSPRAVS &= features.EnablePd;

  // Mutual exclusivity: PD compat mode disables PPS/AVS
  if (features.EnablePdCompatMode &&
      (features.EnablePdLVPPS || features.EnablePdHVPPS ||
       features.EnablePdSPRAVS)) {
    ESP_LOGI(TAG, "Port %d: PD compat mode enabled, disabling PPS/AVS", id_);
    features.EnablePdLVPPS = false;
    features.EnablePdHVPPS = false;
    features.EnablePdSPRAVS = false;
  }

  if (features.EnablePdSPRAVS) {
    ESP_LOGI(TAG, "Port %d: SPR AVS enabled", id_);
  }
  return features;
}

esp_err_t Port::SetConfig(const PortConfig &config) {
  ESP_RETURN_ON_ERROR(this->UpdatePowerFeatures(this->MigrateFeatures(config)),
                      TAG, "Unable to set power features for port %d", id_);
  ESP_RETURN_ON_ERROR(this->Reconnect(), TAG, "Unable to reconnect port %d",
                      id_);
  this->Open();

  return ESP_OK;
}

esp_err_t Port::ApplyConfig(const PortConfig *config) {
  if (config != nullptr) {
    config_ = *config;
  }

  // Save the first applied config for future resets
  if (!initial_config_saved_) {
    initial_config_ = config_;
    initial_config_saved_ = true;
  }

  return SetConfig(config_);
}

void Port::GetData(uint8_t *fc_protocol, uint8_t *temperature,
                   uint16_t *current, uint16_t *voltage) const {
  bool attached = this->Attached();
  if (fc_protocol != nullptr) {
    *fc_protocol = attached ? data_.GetFCProtocol()
                            : static_cast<uint8_t>(FC_NOT_CHARGING);
  }
  if (temperature != nullptr) {
    *temperature = attached ? data_.GetTemperature() : 0;
  }
  if (current != nullptr) {
    *current = attached ? data_.GetCurrent() : 0;
  }
  if (voltage != nullptr) {
    *voltage = attached ? data_.GetVoltage() : 0;
  }
}

esp_err_t Port::SetPortType(PortType type) {
#ifdef CONFIG_MCU_MODEL_SW3566
  SystemFlags new_flags = system_flags_;
  new_flags.port_type = static_cast<uint8_t>(type);
  new_flags.commit = 0x55;
  esp_err_t err = rpc::mcu::set_system_flags(id_, &new_flags);
  if (err != ESP_OK) {
    ESP_ERROR_COMPLAIN(err, "rpc::mcu::set_system_flags(%d)", id_);
    this->SetState(PortStateType::CHECKING);
    return err;
  }
  system_flags_.port_type = static_cast<uint8_t>(type);
#endif
  return ESP_OK;
}

esp_err_t Port::SetCableCompensation(bool enable, CableCompensation resistance,
                                     FixedVoltageOffset voltage_offset,
                                     bool safety) {
#ifdef CONFIG_MCU_MODEL_SW3566
  if (Dead()) {
    ESP_LOGW(TAG, "Port %d is dead, cannot set cable compensation", id_);
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t ret = ESP_OK;
  SystemFlags new_flags;

  if (safety) {
    ESP_GOTO_ON_ERROR(rpc::mcu::port_disconnect(id_), RESET_STATE, TAG,
                      "rpc::mcu::port_disconnect(%d)", id_);
  }

  new_flags = system_flags_;

  new_flags.enable_cable_compensation = enable;
  new_flags.cable_compensation = static_cast<uint8_t>(resistance);
  new_flags.fixed_voltage_offset = static_cast<uint8_t>(voltage_offset);

  new_flags.commit = 0x55;
  ESP_GOTO_ON_ERROR(rpc::mcu::set_system_flags(id_, &new_flags), RESET_STATE,
                    TAG, "rpc::mcu::set_system_flags(%d)", id_);
  system_flags_.enable_cable_compensation = new_flags.enable_cable_compensation;
  system_flags_.cable_compensation = new_flags.cable_compensation;
  system_flags_.fixed_voltage_offset = new_flags.fixed_voltage_offset;

  if (safety) {
    ESP_GOTO_ON_ERROR(rpc::mcu::port_connect(id_), RESET_STATE, TAG,
                      "rpc::mcu::port_connect(%d)", id_);
  }

  return ESP_OK;
RESET_STATE:
  this->SetState(PortStateType::CHECKING);
  return ret;
#else
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

void Port::GetCableCompensation(bool &enable, uint8_t &resistance_mohm,
                                uint8_t &voltage_offset_level) const {
#ifdef CONFIG_MCU_MODEL_SW3566
  enable = system_flags_.enable_cable_compensation;
  resistance_mohm = system_flags_.cable_compensation;
  voltage_offset_level = system_flags_.fixed_voltage_offset;
#endif
}

esp_err_t Port::SetCustomPdoVoltage(uint16_t voltage_mv) {
#ifdef CONFIG_MCU_MODEL_SW3566
  // Validate voltage range [5000, 20000]
  if (voltage_mv < 5000 || voltage_mv > 20000) {
    ESP_LOGE(TAG, "Invalid custom PDO voltage %dmV for port %d", voltage_mv,
             id_);
    return ESP_ERR_INVALID_ARG;
  }

  // Validate not standard PDO voltages
  const uint16_t standard_voltages[] = {5000, 9000, 15000, 20000};
  for (size_t i = 0;
       i < sizeof(standard_voltages) / sizeof(standard_voltages[0]); i++) {
    if (voltage_mv == standard_voltages[i]) {
      ESP_LOGE(TAG, "Voltage %dmV is a standard PDO voltage for port %d",
               voltage_mv, id_);
      return ESP_ERR_INVALID_ARG;
    }
  }

  esp_err_t err = rpc::mcu::set_custom_pdo_voltage(id_, voltage_mv);
  if (err != ESP_OK) {
    ESP_ERROR_COMPLAIN(err, "rpc::mcu::set_custom_pdo_voltage(%d, %d)", id_,
                       voltage_mv);
    this->SetState(PortStateType::CHECKING);
    return err;
  }

  // Store the custom PDO voltage after successful RPC call
  custom_pdo_voltage_mv_ = voltage_mv;
  ESP_LOGI(TAG, "Port %d: Set custom PDO voltage to %dmV", id_, voltage_mv);

  return ESP_OK;
#else
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

bool Port::Attach() {
  if (!IsOpen()) {
    return false;
  }
  if (Attached()) {
    return true;
  }
  attached_ = true;
  attached_at_ = esp_timer_get_time() / 1e3;
  SetState(PortStateType::ATTACHED);
  return true;
}

bool Port::Detach() {
  if (!IsOpen()) {
    return false;
  }
  if (!Attached()) {
    return true;
  }
  if (IsDetaching()) {
    return true;  // Already in debounce window, don't reset timer
  }
  // Enter DETACHING state instead of immediate ACTIVE transition.
  // This provides debounce for quick reattachments (e.g., PD protocol events).
  // attached_ stays true during debounce - Reset() only called in
  // CompleteDetach().
  SetState(PortStateType::DETACHING);
  return true;
}

void Port::StartCheckingWatchdog() {
  if (checking_timer_ != nullptr) {
    ESP_LOGW(TAG, "Port watchdog has already started for port %d.", id_);
    return;
  }

  // Set the checking start time here
  checking_at_ = esp_timer_get_time();

  const esp_timer_create_args_t timer_args = {
      .callback = CheckingWatchdogCallback,
      .arg = (void *)this,
      .dispatch_method = ESP_TIMER_TASK,
      .name = checking_timer_name_.c_str(),
      .skip_unhandled_events = true,
  };

  esp_err_t err = esp_timer_create(&timer_args, &checking_timer_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create port watchdog timer: %s",
             esp_err_to_name(err));
    checking_at_ = -1;  // Reset on failure
    return;
  }

  err = esp_timer_start_once(checking_timer_, PORT_CHECKING_TIMER_MS * 1000);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start port watchdog timer: %s",
             esp_err_to_name(err));
    esp_timer_delete(checking_timer_);
    checking_timer_ = nullptr;
    checking_at_ = -1;  // Reset on failure
    return;
  }
  ESP_LOGD(TAG, "Port watchdog has started for port %d.", id_);
}

void Port::StopCheckingWatchdog() {
  if (checking_timer_ != nullptr) {
    esp_timer_stop(checking_timer_);
    esp_timer_delete(checking_timer_);
    checking_timer_ = nullptr;
    checking_at_ = -1;  // Reset when stopping
    ESP_LOGD(TAG, "Port watchdog stopped for port %d", Id());
  }
}

void Port::CheckingWatchdogCallback(void *arg) {
#ifdef CONFIG_MCU_MODEL_SW3566
  Port *port = (Port *)arg;

  // Send the keep-alive message to the port
  KeepAliveStatus status;
  uint32_t uptime_ms;
  uint32_t reboot_reason;
  esp_err_t ret = ESP_OK;

  ESP_LOGD(TAG, "Port %d watchdog callback started.", port->Id());

  for (int i = 0; i < 3; i++) {
    ret = rpc::mcu::keep_alive(port->Id(), &status, &uptime_ms, &reboot_reason);
    if (ret == ESP_OK) {
      break;
    }
    DELAY_MS(20);
  }

  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Port %d keep-alive failed: %d, recovering", port->Id(), ret);
    port->SetState(PortStateType::RECOVERING);
    return;
  }

  if (status == KeepAliveStatus::USER_APPLICATION_ALIVE) {
    // Port is still alive, revert to previous state
    port->RevertToPreviousState();
    return;
  }

  if (status == KeepAliveStatus::BOOTLOADER_ALIVE) {
    ESP_LOGW(TAG,
             "Port %d is in bootloader, uptime: %" PRIu32
             "ms, reboot_reason: 0x%" PRIx32,
             port->Id(), uptime_ms, reboot_reason);
    port->Reboot();
    return;
  }
#endif
}

void Port::StartDetachingTimer() {
  if (detaching_timer_ != nullptr) {
    ESP_LOGW(TAG, "Port %d detaching timer already running.", id_);
    return;
  }

  const esp_timer_create_args_t timer_args = {
      .callback = DetachingTimerCallback,
      .arg = this,
      .dispatch_method = ESP_TIMER_TASK,
      .name = detaching_timer_name_.c_str(),
      .skip_unhandled_events = true,
  };

  esp_err_t err = esp_timer_create(&timer_args, &detaching_timer_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create detaching timer for port %d: %s", id_,
             esp_err_to_name(err));
    // Fallback: immediate detach
    CompleteDetach();
    return;
  }

  err = esp_timer_start_once(detaching_timer_, DETACH_DEBOUNCE_MS * 1000);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start detaching timer for port %d: %s", id_,
             esp_err_to_name(err));
    esp_timer_delete(detaching_timer_);
    detaching_timer_ = nullptr;
    // Fallback: immediate detach
    CompleteDetach();
    return;
  }

  ESP_LOGI(TAG, "Port %d: Starting %" PRIu32 "ms detach debounce", id_,
           DETACH_DEBOUNCE_MS);
}

void Port::StopDetachingTimer() {
  if (detaching_timer_ != nullptr) {
    esp_timer_stop(detaching_timer_);
    esp_timer_delete(detaching_timer_);
    detaching_timer_ = nullptr;
    ESP_LOGD(TAG, "Port %d detaching timer stopped", id_);
  }
}

void Port::DetachingTimerCallback(void *arg) {
  Port *port = static_cast<Port *>(arg);
  ESP_LOGI(TAG, "Port %d: Detach debounce expired, completing detach",
           port->Id());
  port->CompleteDetach();
}

void Port::CompleteDetach() {
  // Reset port first so GetAttachedPortCount() reflects the new state
  this->Reset();
  SetState(PortStateType::ACTIVE);

  // Trigger DEVICE_REMOVED animation after reset
  // Fallback will correctly see remaining port count
  DisplayManager::GetInstance().SetNextAnimation(AnimationType::DEVICE_REMOVED,
                                                 false);
}

void Port::Checking() {
  if (!(Booting() || Dead())) {
    SetState(PortStateType::CHECKING);
  }
}

PortDetails Port::ValidateAndSanitizeDetails(const PortDetails &details) {
  uint64_t new_charge = details.session_charge;
  uint64_t prev_charge = details_.session_charge;
  bool new_session = (details.session_id != details_.session_id);

  // Accept any value for new sessions
  if (new_session) return details;

  if (details.fc_protocol != details_.fc_protocol) {
    ESP_LOGI(
        TAG,
        "[Port %d] fc_protocol: %d -> %d, pwr: %.2fW, curr: %dmA, volt: %dmV",
        id_, details_.fc_protocol, details.fc_protocol,
        details.iout_value * details.vout_value / 1000000.0, details.iout_value,
        details.vout_value);
  }

  // Reject overflow values
  if (new_charge > NEAR_OVERFLOW_THRESHOLD) {
    ESP_LOGW(TAG,
             "[Port %d] Rejected session_charge overflow: 0x%" PRIx32
             "%08" PRIx32,
             id_, (uint32_t)(new_charge >> 32), (uint32_t)new_charge);
    goto SANITIZE;
  }

  // Reject decreases within same session
  if (attached_ && new_charge < prev_charge && prev_charge > 0) {
    ESP_LOGW(TAG,
             "[Port %d] Rejected session_charge decrease: 0x%" PRIx32
             "%08" PRIx32 " -> 0x%" PRIx32 "%08" PRIx32,
             id_, (uint32_t)(prev_charge >> 32), (uint32_t)prev_charge,
             (uint32_t)(new_charge >> 32), (uint32_t)new_charge);
    goto SANITIZE;
  }

  // Reject huge jumps within same session
  if (new_charge > prev_charge &&
      (new_charge - prev_charge) > MAX_CHARGE_JUMP_PER_UPDATE) {
    ESP_LOGW(TAG,
             "[Port %d] Rejected session_charge jump: +0x%" PRIx32 "%08" PRIx32
             " µWs",
             id_, (uint32_t)((new_charge - prev_charge) >> 32),
             (uint32_t)(new_charge - prev_charge));
    goto SANITIZE;
  }

  return details;

SANITIZE:
  PortDetails sanitized = details;
  sanitized.session_charge = prev_charge;
  return sanitized;
}
