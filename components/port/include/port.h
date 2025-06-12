#ifndef PORT_H_
#define PORT_H_

#include <cstdint>
#include <cstring>

#include "data_types.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "port_data.h"
#include "port_state.h"
#include "ring_buffer.h"
#include "sdkconfig.h"

#ifdef CONFIG_MCU_MODEL_SW3566
#include "sw3566_data_types.h"

#define PORT_MAX_POWER SW3566_MAX_POWER
#define PORT_MIN_POWER SW3566_MIN_POWER
#define PORT_MAX_CAP SW3566_MAX_CAP
#define PORT_USB_A_MAX_CAP 60
#define PORT_MIN_CAP SW3566_MIN_CAP
#endif

#ifdef CONFIG_MCU_MODEL_FAKE_SW3566
#include "fake_sw3566_data_types.h"

#define PORT_MAX_POWER FAKE_SW3566_MAX_POWER
#define PORT_MIN_POWER FAKE_SW3566_MIN_POWER
#define PORT_USB_A_MAX_CAP 60
#define PORT_MAX_CAP FAKE_SW3566_MAX_CAP
#define PORT_MIN_CAP FAKE_SW3566_MIN_CAP
#endif

#define PORT_HISTORICAL_DATA_SIZE CONFIG_PORT_HISTORICAL_DATA_SIZE

typedef RingBuffer<PortStatsData> HistoricalStatsData;

typedef struct __attribute__((packed)) {
  uint8_t version;  // 目前取 0
  PowerFeatures features;
} PortConfig;

class Port {
  uint8_t id_;
  bool attached_ = false;
  uint8_t power_budget_ = 0xFF, power_budget_watermark_ = 0xFF;
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
  PortConfig config_;

#ifdef CONFIG_MCU_MODEL_SW3566
  SystemFlags system_flags_;
#endif

#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
  Subscriptions subscriptions_ = {
      .EnablePortDetailsUpdate = true,
      .EnablePDStatusUpdate = true,
      .EnablePDPcapStreaming = false,
      .unused = 0,
  };
#endif

 public:
  explicit Port(uint8_t port_id);
  ~Port() = default;
  uint8_t Id() const { return id_; }
  bool Initialize(bool active, uint8_t initial_power_budget);
  bool Reinitialize();
  bool IsTypeA() const {
#if defined(CONFIG_MCU_MODEL_SW3566)
    return system_flags_.port_type == PortType::PORT_TYPE_A;
#elif defined(CONFIG_MCU_MODEL_FAKE_SW3566)
    return id_ == 0;
#else
    return false;
#endif
  }
  bool IsInitialized() const { return initialized_; }
  bool IsAdjustable() const;

  void SetState(PortStateType type);
  void RevertToPreviousState() {
    if (last_state_ != PortStateType::UNKNOWN) {
      SetState(last_state_);
    }
  }
  const PortState &GetState() const { return *state_; }
  const PortPowerData &GetData() const { return data_; }
  void GetData(uint8_t *fc_protocol, uint8_t *temperature, uint16_t *current,
               uint16_t *voltage) const;
  uint8_t GetPowerUsage() const { return data_.GetPower(); }
  void GetPDStatus(ClientPDStatus *status) const {
    if (data_.GetFCProtocol() >= FastChargingProtocol::FC_PD_Fix5V) {
      *status = pd_status_;
    }
  }
  void GetPowerFeatures(PowerFeatures *features) const {
    *features = features_;
  }
  esp_err_t UpdatePowerFeatures(const PowerFeatures &features);
  uint32_t GetChargingDurationSeconds() const;

  uint8_t MaxPowerCap() const {
    if (IsTypeA()) {
      return PORT_USB_A_MAX_CAP;
    } else {
      return PORT_MAX_CAP(id_);
    }
  }
  uint8_t MinPowerCap() const { return PORT_MIN_CAP(id_); }
  void EnsureBudgetInRange(uint8_t *power) const;
  bool RestoreInitialPowerBudget();
  esp_err_t SetMaxPowerBudget(uint8_t MaxPowerBudget, uint8_t watermark = 0,
                              bool forceRebroadcast = false,
                              bool enterCheckingOnFailed = true);
  bool ApplyMaxPowerBudget(uint8_t max_power_budget);
  bool ApplyMinPowerBudget() {
    return SetMaxPowerBudget(MinPowerCap()) == ESP_OK;
  }
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

  bool Abnormal() const {
    return state_ == nullptr || state_->Type() == PortStateType::CHECKING ||
           state_->Type() == PortStateType::UNKNOWN ||
           state_->Type() == PortStateType::DEAD ||
           state_->Type() == PortStateType::RECOVERING;
  }
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
  bool Attach();
  bool Detach();

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
  void SetInactiveHistoricalData();
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

  esp_err_t SubscribePDPcapStreaming(bool subscribe);
  esp_err_t UnsubscribePDPcapStreaming();

  PowerFeatures MigrateFeatures(const PortConfig &config) const;
  const PortConfig &GetConfig() const { return config_; }
  esp_err_t SetConfig(const PortConfig &config);
  esp_err_t ApplyConfig(const PortConfig *config = nullptr);
  esp_err_t ResetConfig();
  esp_err_t SetPortType(PortType type);
  PortType GetPortType() const {
#if defined(CONFIG_MCU_MODEL_SW3566)
    return static_cast<PortType>(system_flags_.port_type);
#elif defined(CONFIG_MCU_MODEL_FAKE_SW3566)
    return IsTypeA() ? PortType::PORT_TYPE_A : PortType::PORT_TYPE_C;
#else
    return PortType::PORT_TYPE_C;
#endif
  }

 private:
  void SetState(PortState *state);
};

#endif
