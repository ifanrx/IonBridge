#ifndef PORT_H_
#define PORT_H_

#include <cstdint>
#include <cstring>
#include <string>

#include "data_types.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/timers.h"
#include "port_data.h"
#include "ring_buffer.h"
#include "sdkconfig.h"
#include "singleton.h"

#ifdef CONFIG_MCU_MODEL_SW3566
#include "sw3566_data_types.h"

#define PORT_MAX_POWER SW3566_MAX_POWER
#define PORT_MIN_POWER SW3566_MIN_POWER
#define PORT_MAX_CAP SW3566_MAX_CAP
#define PORT_MIN_CAP SW3566_MIN_CAP
#endif

#ifdef CONFIG_MCU_MODEL_FAKE_SW3566
#include "fake_sw3566_data_types.h"

#define PORT_MAX_POWER FAKE_SW3566_MAX_POWER
#define PORT_MIN_POWER FAKE_SW3566_MIN_POWER
#define PORT_MAX_CAP FAKE_SW3566_MAX_CAP
#define PORT_MIN_CAP FAKE_SW3566_MIN_CAP
#endif

#define PORT_HISTORICAL_DATA_SIZE CONFIG_PORT_HISTORICAL_DATA_SIZE

typedef RingBuffer<PortStatsData> HistoricalStatsData;

typedef struct __attribute__((packed)) {
  uint8_t version;  // 目前取 0
  PowerFeatures features;
} PortConfig;

extern PortConfig default_port_config;
extern PortConfig default_port_a_config;

enum PortStateType {
  UNKNOWN = -1,
  ACTIVE,
  INACTIVE,
  ATTACHED,
  OPENING,
  CLOSING,
  OVER_TEMP_WARNING,
  OVER_TEMP_ALERT,
  COOLING,
  CHECKING,
  RECOVERING,
  DEAD,
  POWER_LIMITING,
};

class Port;

class PortState {
 protected:
  PortStateType type_;

 public:
  virtual ~PortState() = default;
  virtual void Handle(Port &port) = 0;
  virtual void Entering(Port &port) {}
  virtual void Exiting(Port &port) {}

  explicit PortState(PortStateType type) : type_(type) {}
  PortStateType Type() const { return type_; }
  std::string ToString() const {
    switch (type_) {
      case UNKNOWN:
        return "UNKNOWN";
      case ACTIVE:
        return "ACTIVE";
      case INACTIVE:
        return "INACTIVE";
      case ATTACHED:
        return "ATTACHED";
      case OPENING:
        return "OPENING";
      case CLOSING:
        return "CLOSING";
      case OVER_TEMP_WARNING:
        return "OVER_TEMP_WARNING";
      case OVER_TEMP_ALERT:
        return "OVER_TEMP_ALERT";
      case COOLING:
        return "COOLING";
      case CHECKING:
        return "CHECKING";
      case RECOVERING:
        return "RECOVERING";
      case DEAD:
        return "DEAD";
      case POWER_LIMITING:
        return "LIMITED_POWER";
      default:
        return "";
    }
  }
};

class PortActiveState : public PortState, public Singleton<PortActiveState> {
 public:
  void Handle(Port &port) override;

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<PortActiveState>;
  // Private constructor to prevent external instantiation
  PortActiveState() : PortState(PortStateType::ACTIVE) {}
};

class PortInactiveState : public PortState,
                          public Singleton<PortInactiveState> {
 public:
  void Handle(Port &port) override {}

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<PortInactiveState>;
  // Private constructor to prevent external instantiation
  PortInactiveState() : PortState(PortStateType::INACTIVE) {}
};

class PortAttachedState : public PortState,
                          public Singleton<PortAttachedState> {
 public:
  void Handle(Port &port) override;

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<PortAttachedState>;
  // Private constructor to prevent external instantiation
  PortAttachedState() : PortState(PortStateType::ATTACHED) {}
};

class PortOpeningState : public PortState, public Singleton<PortOpeningState> {
 public:
  void Handle(Port &port) override;

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<PortOpeningState>;
  // Private constructor to prevent external instantiation
  PortOpeningState() : PortState(PortStateType::OPENING) {}
};

class PortClosingState : public PortState, public Singleton<PortClosingState> {
 public:
  void Handle(Port &port) override;

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<PortClosingState>;
  // Private constructor to prevent external instantiation
  PortClosingState() : PortState(PortStateType::CLOSING) {}
};

class PortOverTempWarningState : public PortState,
                                 public Singleton<PortOverTempWarningState> {
 public:
  void Handle(Port &port) override;

 private:
  uint8_t exit_offset_ = 5;
  uint8_t exit_temperature_ = CONFIG_PORT_TEMP_WARNING_THRESHOLD - exit_offset_;
  uint8_t alert_temperature_ = CONFIG_PORT_TEMP_ALERT_THRESHOLD;
  uint8_t max_temperature_ = CONFIG_PORT_TEMP_MAX_THRESHOLD;

  // Allow Singleton to access the constructor
  friend class Singleton<PortOverTempWarningState>;
  // Private constructor to prevent external instantiation
  PortOverTempWarningState() : PortState(PortStateType::OVER_TEMP_WARNING) {}
};

class PortOverTempAlertState : public PortState,
                               public Singleton<PortOverTempAlertState> {
 public:
  void Handle(Port &port) override;

 private:
  uint8_t exit_offset_ = 5;
  uint8_t exit_temperature_ = CONFIG_PORT_TEMP_ALERT_THRESHOLD - exit_offset_;
  uint8_t max_temperature_ = CONFIG_PORT_TEMP_MAX_THRESHOLD;

  // Allow Singleton to access the constructor
  friend class Singleton<PortOverTempAlertState>;
  // Private constructor to prevent external instantiation
  PortOverTempAlertState() : PortState(PortStateType::OVER_TEMP_ALERT) {}
};

class PortCoolingState : public PortState, public Singleton<PortCoolingState> {
 public:
  void Handle(Port &port) override;

 private:
  uint8_t exit_offset_ = 5;
  uint8_t exit_temperature_ = CONFIG_PORT_TEMP_WARNING_THRESHOLD - exit_offset_;
  bool closed_ = false;

  // Allow Singleton to access the constructor
  friend class Singleton<PortCoolingState>;
  // Private constructor to prevent external instantiation
  PortCoolingState() : PortState(PortStateType::COOLING) {}
};

class PortCheckingState : public PortState,
                          public Singleton<PortCheckingState> {
 public:
  void Handle(Port &port) override;
  void Entering(Port &port) override;
  void Exiting(Port &port) override;

 private:
  int64_t enter_time_ = -1;

  // Allow Singleton to access the constructor
  friend class Singleton<PortCheckingState>;
  // Private constructor to prevent external instantiation
  PortCheckingState() : PortState(PortStateType::CHECKING) {}

  TimerHandle_t checking_timer_ = nullptr;
  void StartWatchdog(Port &port);
  void StopWatchdog();
  static void WatchdogCallback(TimerHandle_t timer);
};

class PortRecoveringState : public PortState,
                            public Singleton<PortRecoveringState> {
 public:
  void Handle(Port &port) override;

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<PortRecoveringState>;
  // Private constructor to prevent external instantiation
  PortRecoveringState() : PortState(PortStateType::RECOVERING) {}
};

class PortDeadState : public PortState, public Singleton<PortDeadState> {
 public:
  void Handle(Port &port) override {}

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<PortDeadState>;
  // Private constructor to prevent external instantiation
  PortDeadState() : PortState(PortStateType::DEAD) {}
};

class PortPowerLimitingState : public PortState,
                               public Singleton<PortPowerLimitingState> {
 public:
  void Handle(Port &port) override {}

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<PortPowerLimitingState>;
  // Private constructor to prevent external instantiation
  PortPowerLimitingState() : PortState(PortStateType::POWER_LIMITING) {}
};

class Port {
  uint8_t id_;
  bool attached_ = false;
  uint8_t power_budget_ = 0xFF;
  uint8_t initial_power_budget_ = 0xFF;
  bool initialized_ = false;
  PortPowerData data_;
  PortDetails details_;
  PortState *state_;
  uint32_t attached_at_ = -1;
  uint32_t collect_count_ = 0;
  ClientPDStatus pd_status_ = {};
  PowerFeatures features_ = {};
  bool update_power_features_ = false;
  uint8_t failure_count_ = 0;
  const uint8_t max_failure_count_ = 5;
  int64_t updated_at_ = 0;
  int64_t cap_updated_at_ = 0;

  PortAverageData average_data_;
  int64_t stats_last_updated_at_ = 0;
  HistoricalStatsData historical_stats_;

  PortStateType last_state_ = PortStateType::UNKNOWN;

#ifdef CONFIG_MCU_MODEL_SW3566
  struct {
    uint32_t version;
    uint32_t flash_timestamp;
  } bootloader_info_ = {};
#endif

 public:
  explicit Port(uint8_t port_id);
  ~Port();
  uint8_t Id() const { return id_; }
  bool Initialize(bool active, uint8_t initial_power_budget);
  bool Reinitialize();
  bool IsTypeA() const { return id_ == 0; }
  bool IsInitialized() const { return initialized_; }
  bool IsAdjustable();

  void SetState(PortStateType type);
  void RevertToPreviousState() {
    if (last_state_ != PortStateType::UNKNOWN) {
      SetState(last_state_);
    }
  }
  const PortState &GetState() const { return *state_; }
  const PortPowerData &GetData() const { return data_; }
  uint8_t GetPowerUsage() const { return data_.GetPower(); }
  void GetPDStatus(ClientPDStatus *status) const { *status = pd_status_; }
  void GetPowerFeatures(PowerFeatures *features) const {
    *features = features_;
  }
  void UpdatePowerFeatures(const PowerFeatures &features);
  uint32_t GetChargingDurationSeconds() const;

  uint8_t MaxPowerCap() const { return PORT_MAX_CAP(id_); }
  uint8_t MinPowerCap() const { return PORT_MIN_CAP(id_); }
  void EnsureBudgetInRange(uint8_t *power) const;
  bool RestoreInitialPowerBudget();
  bool SetMaxPowerBudget(uint8_t max_power_budget, uint8_t watermark = 0,
                         bool force_rebroadcast = false);
  bool ApplyMaxPowerBudget(uint8_t max_power_budget);
  bool ApplyMinPowerBudget() { return SetMaxPowerBudget(MinPowerCap()); }
  uint8_t MaxPowerBudget() const { return power_budget_; }
  uint8_t RemainingPowerBudget() const {
    return power_budget_ > GetPowerUsage() ? power_budget_ - GetPowerUsage()
                                           : 0;
  }
  bool DecreasePowerBudget(uint8_t power);
  bool IncreasePowerBudget(uint8_t power);
  bool TrickleCharging() const {
    return Attached() &&
           GetPowerUsage() < CONFIG_PORT_TRICKLE_CHARGING_THRESHOLD;
  }

  bool Actived() const {
    return state_ != nullptr && state_->Type() != PortStateType::INACTIVE &&
           state_->Type() != PortStateType::DEAD;
  }
  bool Attached() const { return Actived() && attached_; }
  uint32_t AttachedAtMS() const { return attached_at_; }
  uint8_t Temperature() const { return data_.GetTemperature(); }

  bool Dead() const {
    return state_ == nullptr || state_->Type() == PortStateType::DEAD;
  }
  bool Recovering() const {
    return state_ != nullptr && state_->Type() == PortStateType::RECOVERING;
  }

  bool Closing() const {
    return state_ != nullptr && state_->Type() == PortStateType::CLOSING;
  }
  bool Opening() const {
    return state_ != nullptr && state_->Type() == PortStateType::OPENING;
  }
  bool IsOpen() const {
    if (state_ == nullptr) {
      return false;
    }
    return state_->Type() == PortStateType::ACTIVE ||
           state_->Type() == PortStateType::ATTACHED ||
           state_->Type() == PortStateType::OVER_TEMP_WARNING;
  }
  bool IsClose() const {
    if (state_ == nullptr) {
      return false;
    }
    return state_->Type() == PortStateType::INACTIVE;
  }

  bool IsChecking() const {
    if (state_ == nullptr) {
      return false;
    }
    return state_->Type() == PortStateType::CHECKING;
  }

  bool Warning() const {
    if (state_ == nullptr) {
      return false;
    }
    return state_->Type() == PortStateType::OVER_TEMP_WARNING;
  }
  bool Alert() const {
    if (state_ == nullptr) {
      return false;
    }
    return state_->Type() == PortStateType::OVER_TEMP_ALERT;
  }
  bool OverHighTemp() const {
    if (state_ == nullptr) {
      return false;
    }
    return state_->Type() == PortStateType::OVER_TEMP_WARNING ||
           state_->Type() == PortStateType::OVER_TEMP_ALERT ||
           state_->Type() == PortStateType::COOLING;
  }
  bool IsPowerLimiting() const {
    if (state_ == nullptr) {
      return false;
    }
    return state_->Type() == PortStateType::POWER_LIMITING;
  }

  bool Close();
  bool Open();
  bool Toggle();
  void Shutdown();
  void Reset();
  void Update() {
    if (state_) {
      state_->Handle(*this);
    }
  }

  esp_err_t Reconnect();

  int64_t UpdatedAt() const { return updated_at_; }
  void GetData(PortPowerData *data) const {
    if (data) {
      memcpy(data, &data_, sizeof(PortPowerData));
    }
  }
  void GetDetails(PortDetails *details) const {
    if (details) {
      memcpy(details, &details_, sizeof(PortDetails));
    }
  }
  void UpdateData(const PortDetails &details);
  void SetPDStatus(const ClientPDStatus &status) {
    if (this->Attached()) {
      updated_at_ = esp_timer_get_time();
      pd_status_ = status;
    }
  }
  void SetPowerFeatures(const PowerFeatures &features) { features_ = features; }

  const HistoricalStatsData &GetHistoricalStats() const {
    return historical_stats_;
  }
  size_t GetHistoricalStatsSize() const { return historical_stats_.Size(); }

  esp_err_t EnableLimitedCurrentMode();
  esp_err_t DisableLimitedCurrentMode();
  void EnterPowerLimiting();
  void ExitPowerLimiting();
  void Reboot();

#ifdef CONFIG_MCU_MODEL_SW3566
  void GetBootloaderInfo(uint32_t *version, uint32_t *timestamp) const {
    if (version) {
      *version = bootloader_info_.version;
    }
    if (timestamp) {
      *timestamp = bootloader_info_.flash_timestamp;
    }
  }
#endif

 private:
  void SetState(PortState *state);
};

void handle_port_config_compatibility(int i, PortConfig &config);

#endif
