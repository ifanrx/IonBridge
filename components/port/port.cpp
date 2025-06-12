#include "port.h"

#include <sys/param.h>
#include <sys/types.h>

#include <algorithm>
#include <cinttypes>
#include <cstdint>
#include <cstring>
#include <string>

#include "data_types.h"
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

#ifdef CONFIG_MCU_MODEL_SW3566
#include <cstddef>

#include "sw3566_data_types.h"
#include "utils.h"
#endif

#define SET_BITS(original, bits, value) \
  ((original) =                         \
       ((original) & ~((1 << bits) - 1)) | ((value) & ((1 << bits) - 1)))

#define FC_HANDSHAKE_USAGE_TIME 5e6
#define LIMITED_PORT_POWER_BUDGET CONFIG_LIMITED_PORT_POWER_BUDGET
#define PORT_HISTORICAL_DATA_INTERVAL \
  (CONFIG_PORT_HISTORICAL_DATA_SAMPLE_RATE) * 1e6

static const char *TAG = "Port";

PortConfig default_port_config = {
    .version = 1,
    .features =
        {
            .EnableTfcp = false,
            .EnablePe = true,
            .EnableQc2p0 = true,
            .EnableQc3p0 = true,
            .EnableQc3plus = true,
            .EnableAfc = true,
            .EnableFcp = true,
            .EnableHvScp = true,
            .EnableLvScp = true,
            .EnableSfcp = true,
            .EnableApple = true,
            .EnableSamsung = true,
            .EnableUfcs = false,
            .EnablePd = true,
            .EnablePdCompatMode = false,
            .LimitedCurrentMode = false,
            .EnablePdLVPPS = true,
            .EnablePdEPR = true,
            .EnablePd5V5A = false,
            .EnablePdHVPPS = true,
            .reserved = 0,
        },
};

PortConfig default_port_a_config = {
    .version = 1,
    .features =
        {
            .EnableTfcp = false,
            .EnablePe = true,
            .EnableQc2p0 = true,
            .EnableQc3p0 = true,
            .EnableQc3plus = false,
            .EnableAfc = true,
            .EnableFcp = true,
            .EnableHvScp = true,
            .EnableLvScp = true,
            .EnableSfcp = true,
            .EnableApple = true,
            .EnableSamsung = true,
            .EnableUfcs = false,
            .EnablePd = false,
            .EnablePdCompatMode = false,
            .LimitedCurrentMode = false,
            .EnablePdLVPPS = false,
            .EnablePdEPR = false,
            .EnablePd5V5A = false,
            .EnablePdHVPPS = false,
            .reserved = 0,
        },
};

Port::Port(uint8_t port_id)
    : id_(port_id), historical_stats_(PORT_HISTORICAL_DATA_SIZE) {
  data_ = PortPowerData();
  details_ = PortDetails();
  average_data_ = PortAverageData();
  stats_last_updated_at_ = esp_timer_get_time();
  state_ = &PortActiveState::GetInstance();
#ifdef CONFIG_MCU_MODEL_SW3566
  system_flags_ = {
      .commit = 0,
      .unused0 = true,
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
#endif
  ESP_GOTO_ON_ERROR(rpc::mcu::get_power_features(id_, &features_), RECOVER, TAG,
                    "rpc::mcu::get_power_features(%d)", id_);
  ESP_LOGD(TAG, "Port features: ");
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, &features_, sizeof(features_), ESP_LOG_DEBUG);

  if (IsTypeA()) {
    config_ = default_port_a_config;
  } else {
    config_ = default_port_config;
  }
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
  this->Reset();
  return ApplyMaxPowerBudget(initial_power_budget_);
RECOVER:
  this->SetState(PortStateType::CHECKING);
  return false;
}

bool Port::Reinitialize() {
  return Initialize(state_->Type() != PortStateType::DEAD,
                    initial_power_budget_);
}

void Port::SetState(PortState *state) {
  if (state_->Type() == state->Type()) {
    return;
  }
  ESP_LOGI(TAG, "Port %d state change: %s -> %s", id_,
           state_->ToString().c_str(), state->ToString().c_str());
  last_state_ = state_->Type();
  state_->Exiting(*this);
  state_ = state;
  state_->Entering(*this);
}

void Port::SetState(PortStateType type) {
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
  if (IsOpen()) {
    SetState(PortStateType::CLOSING);
    return true;
  }
  return false;
}

bool Port::Open() {
  if (IsClose()) {
    SetState(PortStateType::OPENING);
    return true;
  }
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

void Port::UpdateData(const PortDetails &details) {
  if (!Actived()) {
    return;
  }

  updated_at_ = esp_timer_get_time();

  ESP_LOGD(TAG, "#PORTSTATS[%d] T=%dC, I=%dmA, V=%dmV, connected=%d", id_,
           details.die_temperature, details.iout_value, details.vout_value,
           details.connected);
  details_ = details;
  attached_ = details.connected;
  if (attached_) {
    if (attached_at_ == -1) {
      attached_at_ = esp_timer_get_time() / 1e3;
      ESP_LOGD(TAG, "#PORTSTATS[%d] attached_at: %" PRIi32, id_,
               (int32_t)attached_at_);
    }
    data_.SetFCProtocol(static_cast<uint8_t>(details.fc_protocol));
    data_.SetCurrent(details.iout_value);
    data_.SetVoltage(details.vout_value);
    data_.SetTemperature(details.die_temperature);
    data_.SetVinValue(details.vin_value);
  } else {
    data_.Reset();
    attached_at_ = -1;
  }

  // Update average data
  average_data_.IncrementalAverageData(data_);

  // Push to historical data
  int64_t now = esp_timer_get_time();
  if (now - stats_last_updated_at_ > PORT_HISTORICAL_DATA_INTERVAL) {
    stats_last_updated_at_ = now;
    historical_stats_.Push(PortStatsData(average_data_));
    average_data_.Reset();
  }
}

void Port::SetInactiveHistoricalData() {
  // append zero data to historical data while port is inactive
  int64_t now = esp_timer_get_time();
  if (now - stats_last_updated_at_ > PORT_HISTORICAL_DATA_INTERVAL) {
    stats_last_updated_at_ = now;
    average_data_.Reset();
    historical_stats_.Push(PortStatsData(average_data_));
  }
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

void Port::EnsureBudgetInRange(uint8_t *power) const {
  uint8_t rounded_power = *power;
  *power = std::min(std::max(rounded_power, MinPowerCap()), MaxPowerCap());
}

esp_err_t Port::SetMaxPowerBudget(uint8_t maxPowerBudget, uint8_t watermark,
                                  bool forceRebroadcast,
                                  bool enterCheckingOnFailed) {
  esp_err_t ret = ESP_OK;
  int64_t now = esp_timer_get_time();

  constexpr int64_t kMinUpdateIntervalUs = 1e6;  // 1 second in microseconds
  // Skip update if not forced, budget not increased, and called too soon
  bool skip_update = !forceRebroadcast && maxPowerBudget <= power_budget_ &&
                     (now - cap_updated_at_ < kMinUpdateIntervalUs);
  if (skip_update) {
    ESP_LOGD(TAG, "Port %d: SetMaxPowerBudget called too soon, ignoring", id_);
    return ESP_OK;
  }
  cap_updated_at_ = now;

  EnsureBudgetInRange(&maxPowerBudget);
  if (watermark == 0 || watermark > maxPowerBudget) {
    watermark = maxPowerBudget;
  }
  if (maxPowerBudget == power_budget_ && watermark == power_budget_watermark_) {
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

  features_ = features;
  return ESP_OK;
}

void Port::Shutdown() {
#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
  rpc::fpga::toggle_mcu_gpio(id_);
#endif
  this->SetState(PortStateType::DEAD);
}

esp_err_t Port::EnableLimitedCurrentMode() {
  features_.LimitedCurrentMode = true;
  return rpc::mcu::set_power_features(id_, features_);
}

esp_err_t Port::DisableLimitedCurrentMode() {
  features_.LimitedCurrentMode = false;
  return rpc::mcu::set_power_features(id_, features_);
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

  SetState(PortStateType::ACTIVE);
  Reinitialize();
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
  subscriptions_.EnablePDPcapStreaming = subscribe;
  ESP_RETURN_ON_ERROR(rpc::mcu::set_subscription(id_, subscriptions_), TAG,
                      "Unable to subscribe to port %d", id_);
  return ESP_OK;
#else
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t Port::UnsubscribePDPcapStreaming() {
#ifdef CONFIG_MCU_MODEL_SW3566
  if (subscriptions_.EnablePDPcapStreaming) {
    ESP_LOGI(TAG, "Unsubscribing PD Pcap stream from port %d", id_);
    subscriptions_.EnablePDPcapStreaming = false;
    ESP_RETURN_ON_ERROR(rpc::mcu::set_subscription(id_, subscriptions_), TAG,
                        "Unable to unsubscribe to port %d", id_);
  }
#endif
  return ESP_OK;
}

PowerFeatures Port::MigrateFeatures(const PortConfig &config) const {
  PowerFeatures features = {};

  // Directly assign the first byte
  ((uint8_t *)&features)[0] = ((uint8_t *)&config.features)[0];
  // Assign the first 7 bits of the second byte
  SET_BITS(((uint8_t *)&features)[1], 7, ((uint8_t *)&config.features)[1]);
  // Assign the first 4 bits of the third byte
  SET_BITS(((uint8_t *)&features)[2], 4, ((uint8_t *)&config.features)[2]);

  if (config.version == 0) {
    features.EnablePdLVPPS = !this->IsTypeA();
    features.EnablePdHVPPS = !this->IsTypeA();
    features.EnablePdEPR = !this->IsTypeA();
    features.reserved = 0;
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

esp_err_t Port::ResetConfig() {
  PortConfig config;
  if (IsTypeA()) {
    config = default_port_a_config;
  } else {
    config = default_port_config;
  }
  return ApplyConfig(&config);
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
  system_flags_ = new_flags;
  system_flags_.commit = 0;
#endif
  return ESP_OK;
}

bool Port::Attach() {
  if (!IsOpen()) {
    return false;
  }
  if (Attached()) {
    return true;
  }
  this->Reset();
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
  this->Reset();
  SetState(PortStateType::ACTIVE);
  return true;
}
