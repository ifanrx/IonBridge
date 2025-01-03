#include "port.h"

#include <sys/param.h>

#include <algorithm>
#include <cinttypes>
#include <cstdint>
#include <cstring>
#include <string>

#include "data_types.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ionbridge.h"
#include "machine_info.h"
#include "port_data.h"
#include "rpc.h"
#include "sdkconfig.h"

#ifdef CONFIG_MCU_MODEL_SW3566
#include "sw3566_data_types.h"
#include "utils.h"
#endif

#define FC_HANDSHAKE_USAGE_TIME 5e6
#define LIMITED_PORT_POWER_BUDGET CONFIG_LIMITED_PORT_POWER_BUDGET
#define PORT_HISTORICAL_DATA_INTERVAL \
  (CONFIG_PORT_HISTORICAL_DATA_SAMPLE_RATE) * 1e6

static const char *TAG = "Port";

PortConfig default_port_config = {
    .version = 0,
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
            .EnableOverCompensation = false,
            .LimitedCurrentMode = false,
        },
};

PortConfig default_port_a_config = {
    .version = 0,
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
            .EnableOverCompensation = false,
            .LimitedCurrentMode = false,
        },
};

Port::Port(uint8_t port_id)
    : id_(port_id), historical_stats_(PORT_HISTORICAL_DATA_SIZE) {
  data_ = PortPowerData();
  details_ = PortDetails();
  average_data_ = PortAverageData();
  stats_last_updated_at_ = esp_timer_get_time();
  state_ = &PortActiveState::GetInstance();
  this->Reset();
}

// Singleton, cannot destroy!
Port::~Port() { return; }

bool Port::Initialize(bool active, uint8_t initial_power_budget) {
  if (!active) {
    SetState(PortStateType::DEAD);
    return true;
  }
  esp_err_t ret __attribute__((unused));
#ifdef CONFIG_MCU_MODEL_SW3566
  ESP_GOTO_ON_ERROR(
      rpc::mcu::get_bootloader_info(id_, &bootloader_info_.version,
                                    &bootloader_info_.flash_timestamp),
      RECOVER, TAG, "rpc::mcu::get_bootloader_info(%d)", id_);
#endif
  ESP_GOTO_ON_ERROR(rpc::mcu::get_power_features(id_, &features_), RECOVER, TAG,
                    "rpc::mcu::get_power_features(%d)", id_);

  if (id_ == 1 || id_ == 3) {
    // Check hardware version
    const std::string &hw_version = MachineInfo::GetInstance().GetHwRev();
    // pcb layout issue, enable limited current mode on port 1 and 3
    if (hw_version.compare("B0P0") == 0 || hw_version.compare("PVT1") == 0 ||
        hw_version.compare("PVT2") == 0) {
      features_.LimitedCurrentMode = true;
      ESP_GOTO_ON_ERROR(rpc::mcu::set_power_features(id_, features_), RECOVER,
                        TAG, "rpc::mcu::set_power_features(%d)", id_);
      ESP_LOGW(TAG, "Limited current mode enabled for port %d", id_);
    }
  }

#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
  ESP_GOTO_ON_ERROR(
      rpc::mcu::set_subscription(id_, {.EnablePortDetailsUpdate = true,
                                       .EnablePDStatusUpdate = true,
                                       .unused = 0}),
      RECOVER, TAG, "Unable to subscribe to port %d", id_);
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
  updated_at_ = esp_timer_get_time();

  ESP_LOGD(TAG, "#PORTSTATS[%d] T=%dC, I=%dmA, V=%dmV, connected=%d", id_,
           details.die_temperature, details.iout_value, details.vout_value,
           details.connected);
  details_ = details;
  attached_ = details.connected;
  if (attached_ && this->Actived()) {
    if (attached_at_ == -1) {
      attached_at_ = esp_timer_get_time() / 1e3;
      ESP_LOGD(TAG, "#PORTSTATS[%d] attached_at: %" PRIi32, id_,
               (int32_t)attached_at_);
    }
    data_.SetFCProtocol(static_cast<uint8_t>(details.fc_protocol));
    data_.SetCurrent(details.iout_value);
    data_.SetVoltage(details.vout_value);
  } else {
    data_.Reset();
    attached_at_ = -1;
  }

  if (this->Actived()) {
    data_.SetTemperature(details.die_temperature);
    data_.SetVinValue(details.vin_value);
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

void Port::Reset() {
  attached_ = false;
  attached_at_ = -1;
  updated_at_ = -1;
  memset(&pd_status_, 0, sizeof(pd_status_));
  memset(&details_, 0, sizeof(details_));
  data_.Reset();
}

void Port::EnsureBudgetInRange(uint8_t *power) const {
  uint8_t rounded_power = *power;
  *power = std::min(std::max(rounded_power, MinPowerCap()), MaxPowerCap());
}

bool Port::SetMaxPowerBudget(uint8_t max_power_budget, uint8_t watermark,
                             bool force_rebroadcast) {
  esp_err_t __attribute__((unused)) ret;
  int64_t now = esp_timer_get_time();
  if (now - cap_updated_at_ < 1e6) {
    return false;
  }
  cap_updated_at_ = now;
  EnsureBudgetInRange(&max_power_budget);
  if (max_power_budget == power_budget_) {
    return false;
  }
  if (!watermark || watermark > max_power_budget) {
    watermark = max_power_budget;
  }

  ESP_LOGD(TAG, "Port %d power budget: %dW => %dW", id_, power_budget_,
           max_power_budget);
  uint8_t set_budget_ = 0;
  ESP_GOTO_ON_ERROR(
      rpc::mcu::set_max_power_budget(id_, max_power_budget, watermark,
                                     force_rebroadcast, &set_budget_),
      RESET_STATE, TAG, "rpc::mcu::set_max_power_budget(%d, %d)", id_,
      max_power_budget);
  if (set_budget_) {
    ESP_LOGD(TAG, "Port %d set_max_power_budget_return: %dW", id_, set_budget_);
  }
  power_budget_ = max_power_budget;
  ESP_LOGD(TAG, "Port %d power budget changed: %dW", id_, power_budget_);
  return true;
RESET_STATE:
  this->SetState(PortStateType::CHECKING);
  return false;
}

bool Port::DecreasePowerBudget(uint8_t power) {
  // Ensure we don't go below the minimum power budget
  power = (power + 4) / 5 * 5;
  uint8_t new_budget =
      (power_budget_ > power) ? power_budget_ - power : this->MinPowerCap();
  return SetMaxPowerBudget(new_budget);
}

bool Port::IncreasePowerBudget(uint8_t power) {
  // Calculate new budget and ensure it does not exceed the max
  power = (power + 4) / 5 * 5;
  uint8_t new_budget = power_budget_ + power;
  if (new_budget <= MaxPowerCap()) {
    return SetMaxPowerBudget(new_budget);
  }
  return false;
}

bool Port::ApplyMaxPowerBudget(uint8_t max_power_budget) {
  esp_err_t __attribute__((unused)) ret;

  if (this->Dead()) {
    ESP_LOGE(TAG, "Cannot apply max power budget for dead port %d", id_);
    return false;
  }

  EnsureBudgetInRange(&max_power_budget);
  ESP_GOTO_ON_ERROR(rpc::mcu::set_max_power_budget(id_, max_power_budget,
                                                   max_power_budget, false),
                    RESET_STATE, TAG, "rpc::mcu::set_max_power_budget(%d, %d)",
                    id_, max_power_budget);
  ESP_GOTO_ON_ERROR(rpc::mcu::get_max_power_budget(id_, &power_budget_),
                    RESET_STATE, TAG, "rpc::mcu::get_max_power_budget(%d)",
                    id_);

  ESP_LOGD(TAG, "Port %d: Maximum power budget is %dW, raw budget is %dW", id_,
           max_power_budget, power_budget_);

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
  return SetMaxPowerBudget(initial_power_budget_);
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

void Port::UpdatePowerFeatures(const PowerFeatures &features) {
  esp_err_t err = rpc::mcu::set_power_features(id_, features);
  if (err != ESP_OK) {
    ESP_ERROR_COMPLAIN(err, "rpc::mcu::set_power_features(%d)", id_);
    this->SetState(PortStateType::CHECKING);
    return;
  }

  features_ = features;
}

void Port::Shutdown() {
#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
  rpc::fpga::toggle_mcu_gpio(id_);
#endif
  this->SetState(PortStateType::DEAD);
}

void Port::EnterPowerLimiting() {
  ESP_LOGW(TAG, "Port %d enter power limiting", id_);
  esp_err_t ret = ESP_OK;
  uint8_t max_power_budget = MinPowerCap();
  uint8_t set_budget_ = 0;
  features_.LimitedCurrentMode = true;
  ESP_GOTO_ON_ERROR(rpc::mcu::set_power_features(id_, features_), RESET_STATE,
                    TAG, "rpc::mcu::set_power_features(%d)", id_);
  ESP_GOTO_ON_ERROR(
      rpc::mcu::set_max_power_budget(id_, max_power_budget, max_power_budget,
                                     true, &set_budget_),
      RESET_STATE, TAG, "rpc::mcu::set_max_power_budget(%d, %d)", id_,
      max_power_budget);
  power_budget_ = max_power_budget;
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
  features_.LimitedCurrentMode = false;
  ESP_GOTO_ON_ERROR(rpc::mcu::set_power_features(id_, features_), DEAD_PORT,
                    TAG, "rpc::mcu::set_power_features(%d)", id_);
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
DEAD_PORT:
  ESP_LOGE(TAG, "Port %d boot failed: %d", id_, ret);
  // Port is at unknown state, mark it as dead
  SetState(PortStateType::DEAD);
  return;

#endif
  Reinitialize();
  SetState(PortStateType::ACTIVE);
  return;
}
