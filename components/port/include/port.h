#ifndef PORT_H_
#define PORT_H_

#include <array>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>

#include "data_types.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "internal/sw3566_command_types.h"
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

#ifdef CONFIG_MCU_MODEL_SW3516
#include "sw3516_data_types.h"

#define PORT_MAX_POWER SW3516_MAX_POWER
#define PORT_MIN_POWER SW3516_MIN_POWER
#define PORT_USB_A_MAX_CAP 60
#define PORT_MAX_CAP(_) (PORT_MAX_POWER)
#define PORT_MIN_CAP(_) (PORT_MIN_POWER)
#endif

#ifdef CONFIG_MCU_MODEL_SW3526
#include "sw3526_data_types.h"

#define PORT_MAX_POWER SW3526_MAX_POWER
#define PORT_MIN_POWER SW3526_MIN_POWER
#define PORT_USB_A_MAX_CAP 60
#define PORT_MAX_CAP(_) (PORT_MAX_POWER)
#define PORT_MIN_CAP(_) (PORT_MIN_POWER)
#endif

#ifdef CONFIG_MCU_MODEL_SW3536
#include "sw3536_data_types.h"

#define PORT_MAX_POWER SW3536_MAX_POWER
#define PORT_MIN_POWER SW3536_MIN_POWER
#define PORT_USB_A_MAX_CAP 60
#define PORT_MAX_CAP(_) (PORT_MAX_POWER)
#define PORT_MIN_CAP(_) (PORT_MIN_POWER)
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
#define PORT_HISTORICAL_DETAILS_SIZE CONFIG_PORT_HISTORICAL_DETAILS_SIZE

typedef RingBuffer<PortStatsData> HistoricalStatsData;
typedef RingBuffer<PortDetails> HistoricalDetailsData;

typedef struct __attribute__((packed)) {
  uint8_t version;
  PowerFeatures features;
} PortConfig;

class Port {
  uint8_t id_;
  bool attached_ = false;
  uint8_t power_budget_ = 0, power_budget_watermark_ = 0;
  uint8_t initial_power_budget_ = 0;
  bool initialized_ = false;
  PortPowerData data_;
  PortDetails details_;
  PortState *state_;
  uint32_t attached_at_ = -1;
  ClientPDStatus pd_status_ = {};
  int64_t updated_at_ = 0;
  int64_t cap_updated_at_ = 0;

  PortAverageData average_data_;
  int64_t stats_last_updated_at_ = 0;
  HistoricalStatsData historical_stats_;
  HistoricalDetailsData historical_details_;

  PortStateType last_state_ = PortStateType::UNKNOWN;
  PortConfig config_;                  // Current configuration
  PortConfig initial_config_;          // First applied config (for reset)
  bool initial_config_saved_ = false;  // Track if initial config has been saved
  uint16_t custom_pdo_voltage_mv_ = 0;
  using StateCallback = std::function<void(Port &port)>;

  std::array<StateCallback, PortStateType::COUNT> state_callbacks_;

#ifdef CONFIG_MCU_MODEL_SW3566
  struct {
    uint32_t version;
    uint32_t flash_timestamp;
  } bootloader_info_ = {};

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

  // Checking state timer
  esp_timer_handle_t checking_timer_ = nullptr;
  int64_t checking_at_ = -1;
  std::string checking_timer_name_;

  // Detaching state timer (debounce for quick reattachments)
  static constexpr uint32_t DETACH_DEBOUNCE_MS = 500;
  esp_timer_handle_t detaching_timer_ = nullptr;
  std::string detaching_timer_name_;
  static void DetachingTimerCallback(void *arg);

 public:
  explicit Port(uint8_t port_id);
  ~Port() = default;
  uint8_t Id() const { return id_; }
  bool Initialize(bool active, uint8_t initial_power_budget);
  bool IsTypeA() const {
#if defined(CONFIG_MCU_MODEL_SW3566)
    return system_flags_.port_type == PortType::PORT_TYPE_A;
#elif defined(CONFIG_MCU_MODEL_FAKE_SW3566)
    return FAKE_SW3566_IS_TYPE_A(CONFIG_MCU_MODEL_FAKE_SW3566);
#else
    return false;
#endif
  }
  bool IsInitialized() const { return initialized_; }
  bool IsAdjustable() const;

  void SetState(PortStateType type);
  void SetState(PortState *state);
  void RevertToPreviousState() {
    if (last_state_ != PortStateType::UNKNOWN) {
      SetState(last_state_);
    }
  }
  const PortState &GetState() const { return *state_; }
  void RegisterStateChangeCallback(PortStateType type, StateCallback cb) {
    if (type >= 0 && type < PortStateType::COUNT) {
      state_callbacks_[type] = cb;
    }
  }
  void UnregisterAll() { state_callbacks_.fill(nullptr); }
  PortStateType GetPreviousStateType() const { return last_state_; }
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
    *features = config_.features;
  }
  esp_err_t UpdatePowerFeatures(const PowerFeatures &features);
  uint32_t GetChargingDurationSeconds() const;

  esp_err_t SetCustomPdoVoltage(uint16_t voltage_mv);
  uint16_t GetCustomPdoVoltage() const {
#ifdef CONFIG_MCU_MODEL_SW3566
    if (custom_pdo_voltage_mv_ == 0) {
      return 12000;  // Default to 12V if not set
    }
#endif
    return custom_pdo_voltage_mv_;
  }

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

  bool Activated() const {
    return state_ != nullptr && state_->Type() != PortStateType::INACTIVE &&
           state_->Type() != PortStateType::DEAD;
  }
  bool Attached() const { return Activated() && attached_; }
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
  bool Booting() const {
    return state_ != nullptr && state_->Type() == PortStateType::BOOTING;
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
           state_->Type() == PortStateType::DETACHING ||
           state_->Type() == PortStateType::OVER_TEMP_WARNING;
  }
  bool IsClose() const {
    if (state_ == nullptr) {
      return false;
    }
    return state_->Type() == PortStateType::INACTIVE;
  }

  void Checking();
  bool IsChecking() const {
    if (state_ == nullptr) {
      return false;
    }
    return state_->Type() == PortStateType::CHECKING;
  }
  // Timer management methods
  void StartCheckingWatchdog();
  void StopCheckingWatchdog();
  bool IsCheckingWatchdogActive() const { return checking_timer_ != nullptr; }
  // Checking state time management
  int64_t CheckingAt() const { return checking_at_; }

  // Detaching timer management (debounce for quick reattachments)
  void StartDetachingTimer();
  void StopDetachingTimer();
  void CompleteDetach();
  bool IsDetaching() const {
    if (state_ == nullptr) {
      return false;
    }
    return state_->Type() == PortStateType::DETACHING;
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

  esp_err_t Shutdown();
  void Reset();
  esp_err_t ResetState();
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
  bool IsConnected() const { return details_.connected; }
  esp_err_t FetchDetails(PortDetails *details);
  void UpdateData(const PortDetails &details);
  void SetInactiveHistoricalData();
  void SetPDStatus(const ClientPDStatus &status) {
    if (this->Attached()) {
      updated_at_ = esp_timer_get_time();
      pd_status_ = status;
    }
  }
  void SetPowerFeatures(const PowerFeatures &features) {
    config_.features = features;
  }

  const HistoricalStatsData &GetHistoricalStats() const {
    return historical_stats_;
  }
  size_t GetHistoricalStatsSize() const { return historical_stats_.Size(); }
  const HistoricalDetailsData &GetHistoricalDetails() const {
    return historical_details_;
  }
  size_t GetHistoricalDetailsSize() const { return historical_details_.Size(); }

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

  esp_err_t SubscribePDPcapStreaming(bool subscribe);

  PowerFeatures MigrateFeatures(const PortConfig &config) const;
  const PortConfig &GetConfig() const { return config_; }
  void ResetConfig();           // Reset to default values
  void ResetToInitialConfig();  // Reset to first applied config
  esp_err_t SetConfig(const PortConfig &config);
  esp_err_t ApplyConfig(const PortConfig *config = nullptr);
  esp_err_t SetCableCompensation(bool enable, CableCompensation resistance,
                                 FixedVoltageOffset voltage_offset,
                                 bool safety = false);
  void GetCableCompensation(bool &enable, uint8_t &resistance_mohm,
                            uint8_t &voltage_offset_level) const;
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
  static void CheckingWatchdogCallback(void *arg);
  PortDetails ValidateAndSanitizeDetails(const PortDetails &details);
};

#endif
