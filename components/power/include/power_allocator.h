#ifndef POWER_ALLOCATOR_H_
#define POWER_ALLOCATOR_H_

#include <sys/types.h>

#include <cstdint>
#include <memory>
#include <type_traits>

#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "port_data.h"
#include "port_manager.h"
#include "ring_buffer.h"
#include "strategy.h"

using HistoricalStatsData = RingBuffer<PortStatsData>;

enum TemperatureMode : uint8_t {
  POWER_PRIORITY = 0x00,
  TEMPERATURE_PRIORITY = 0x01,
};

enum PowerAllocatorType : uint8_t {
  FLEXAI_FAST_ALLOCATOR = 0x00,
  FLEXAI_SLOW_ALLOCATOR = 0x01,
  STATIC_ALLOCATOR = 0x02,
  TEMPORARY_ALLOCATOR = 0x03,
  ULTRA_SLOW_ALLOCATOR = 0x04,
  ULTRA_FAST_ALLOCATOR = 0x05,
  USBA_ALLOCATOR = 0x06,
};

enum class PowerAllocatorEventType {
  EXECUTE = (1 << 0),
  PORT_ATTACHED = (1 << 1),
  PORT_DETACHED = (1 << 2),
  REALLOCATE = (1 << 3),
};

struct PowerAllocatorEvent {
  PowerAllocatorEventType event_type;
  uint8_t port_id;  // 0xff indicates all ports
};

class PowerAllocator {
  PortManager &port_manager_;
  // Use smart pointers for automatic memory management
  std::unique_ptr<PowerStrategy> strategy_;
  std::unique_ptr<PowerStrategy> strategy_buffer_;
  uint32_t cooldown_period_secs_;
  uint32_t apply_period_;
  esp_timer_handle_t timer_handle_;
  uint32_t timer_period_;
  uint32_t charging_at_ = 0;
  PowerAllocatorType type_;
  bool executing_ = false;
  TemperatureMode temperature_mode_ = TemperatureMode::POWER_PRIORITY;
  bool power_allocation_enabled_ = true;
  uint16_t apply_count_ = 0;
  bool configured_ = false;

 public:
  static PowerAllocator &GetInstance();
  PowerAllocator(const PowerAllocator &) = delete;
  PowerAllocator &operator=(const PowerAllocator &) = delete;

  esp_err_t Start();
  void Stop();
  void Execute();
  bool IsRunning() const { return taskRunning_; }
  void PortAttached(uint8_t port_id);
  void PortDetached(uint8_t port_id);
  void Reallocate();
  void HighTempDetect();
  bool OverTempLimitDetect();
  void ReducePowerBudget(uint8_t max_power, uint8_t decreasement,
                         uint8_t min_limit, const char *scenario);
  esp_err_t EnqueueEvent(PowerAllocatorEvent event);
  esp_err_t EnqueueEvent(PowerAllocatorEventType event_type, uint8_t port_id);
  void ReportPowerStats();
  uint16_t AddApplyCount();
  void Configure(
      PowerStrategy *strategy, uint32_t cooldown_period_secs,
      uint32_t apply_period, uint32_t timer_period,
      TemperatureMode temperature_mode = TemperatureMode::POWER_PRIORITY);
  bool IsConfigured() const { return configured_; }

  // Power strategy
  // Generic SetStrategy using variadic templates
  template <
      typename T, typename... Args,
      typename = std::enable_if_t<std::is_base_of<PowerStrategy, T>::value &&
                                  !std::is_abstract<T>::value>>
  void SetStrategy(Args &&...args) {
    // Ensure that the first argument is cooldown_period_secs_
    strategy_buffer_ =
        std::make_unique<T>(cooldown_period_secs_, std::forward<Args>(args)...);
  }
  void SetCooldownPeriod(uint32_t cooldown_period_secs) {
    cooldown_period_secs_ = cooldown_period_secs;
    if (strategy_ != nullptr) {
      strategy_->SetCooldownPeriod(cooldown_period_secs);
    }
  }
  bool SwitchStrategy();
  // Templated getter function using SFINAE
  template <typename T>
  typename std::enable_if<std::is_base_of<PowerStrategy, T>::value,
                          const T *>::type
  GetStrategyAs() const {
    // Check if strategy is set
    if (!strategy_) {
      return nullptr;
    }
    // Compare the static TYPE
    if (strategy_->Type() != T::TYPE) {
      return nullptr;
    }
    // Safe to cast
    return static_cast<const T *>(strategy_.get());
  }
  StrategyType GetStrategyType() const { return strategy_->Type(); }

  PowerAllocatorType Type() const { return type_; }

  esp_err_t SetTemperatureMode(TemperatureMode mode);
  TemperatureMode GetTemperatureMode() const { return temperature_mode_; }

  void DisablePowerAllocation() { power_allocation_enabled_ = false; }
  void GetAllocationData(uint16_t *power_budget,
                         uint16_t *remaining_power) const;

  ~PowerAllocator() { Stop(); }

 private:
  PowerAllocator();

  static void timer_callback_(void *arg);
  static void TaskLoopWrapper(void *pvParameter);
  void TaskLoop();
  bool taskRunning_ = false;
  QueueHandle_t eventQueue_;
  TaskHandle_t taskHandle_;
};

class AllocatorBuilder {
  uint8_t port_count_;
  bool *ports_active_;
  uint32_t timer_period_;
  PowerStrategy *strategy_;
  uint32_t cooldown_period_secs_;
  uint32_t apply_period_;

 public:
  AllocatorBuilder()
      : port_count_(0),
        ports_active_(nullptr),
        timer_period_(1000),
        strategy_(nullptr),
        cooldown_period_secs_(1),
        apply_period_(1000) {}

  AllocatorBuilder &SetPortCount(uint8_t port_count) {
    port_count_ = port_count;
    ports_active_ = new bool[port_count_];
    return *this;
  }

  AllocatorBuilder &SetPortActiveState(bool *ports_active, uint8_t port_count) {
    if (port_count == port_count_) {
      memcpy(ports_active_, ports_active, port_count_ * sizeof(bool));
    }
    return *this;
  }

  AllocatorBuilder &SetTimerPeriod(uint32_t timer_period_ms) {
    timer_period_ = timer_period_ms;
    return *this;
  }

  AllocatorBuilder &SetCooldownPeriod(uint32_t cooldown_period_secs) {
    cooldown_period_secs_ = cooldown_period_secs;
    return *this;
  }

  template <
      typename T, typename... Args,
      typename = std::enable_if_t<std::is_base_of<PowerStrategy, T>::value &&
                                  !std::is_abstract<T>::value>>
  AllocatorBuilder &SetStrategy(Args &&...args) {
    strategy_ = new T(cooldown_period_secs_, std::forward<Args>(args)...);
    return *this;
  }

  AllocatorBuilder &SetApplyPeriod(uint32_t apply_period_ms) {
    apply_period_ = apply_period_ms / timer_period_;
    return *this;
  }

  PowerAllocator &Build();

  ~AllocatorBuilder() { delete[] ports_active_; }
};

#endif
