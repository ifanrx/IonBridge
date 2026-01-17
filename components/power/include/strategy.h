#ifndef POWER_STRATEGY_H_
#define POWER_STRATEGY_H_

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <functional>

#include "port.h"
#include "port_manager.h"
#include "power_config.h"
#include "utils.h"

#define POWER_BUDGET (power_config.power_budget)

// Forward declaration to avoid circular dependency
enum TemperatureMode : uint8_t;

enum StrategyType : uint8_t {
  SLOW_CHARGING = 0x01,
  STATIC_CHARGING = 0x02,
  TEMPORARY_CHARGING = 0x03,
  LEGACY_CHARGING = 0x04,
  USBA_CHARGING = 0x06,
  HIGH_PERFORMANCE_CHARGING = 0x07,
  ULTRA_FAST_SINGLE_PORT_CHARGING = 0x08,
  UNKNOWN_STRATEGY = 0xFF,
};

enum StrategyStage : uint8_t {
  STAGE_INIT = 0,
  STAGE_UNDER_THRESHOLD = 1,
  STAGE_OVER_THRESHOLD = 2
};

typedef struct {
  uint16_t power_allocation[NUM_PORTS];  // Array representing power allocation
                                         // for each port
} PowerAllocationEntry;

/* connection_status: bitmask, LSB is port A (id is 0), bit 1 is port C1, etc.
 *
 * example:
 *   0b00001 -> A is connected
 *   0b10001 -> C4 and A are connected
 */
typedef struct __attribute__((packed)) {
  uint8_t version;
  PowerAllocationEntry entries[32];  // index as connection status
} PowerAllocationTable;

class PowerStrategy {
  uint16_t max_power_ = POWER_BUDGET;

 public:
  // Exit callback type
  using ExitCallback = std::function<void()>;

 protected:
  uint8_t initial_power_ = 0;
  StrategyType stratege_type_ = UNKNOWN_STRATEGY;
  ExitCallback exit_callback_;

 public:
  PowerStrategy() = default;
  virtual ~PowerStrategy() = default;

  // Set exit callback
  void OnExit(ExitCallback cb) { exit_callback_ = std::move(cb); }

  virtual int16_t Allocate(const Port &port, uint16_t remaining_power) = 0;
  /* Periodic apply at default, could be override */
  virtual bool ShouldApply(PortManager &port_manager, bool is_periodic) const {
    return is_periodic;
  }
  virtual uint16_t Apply(PortManager &port_manager) = 0;
  virtual void Reallocate(PortManager &port_manager) {}
  virtual void Setup(PortManager &port_manager) = 0;
  virtual void Teardown(PortManager &port_manager) {};
  virtual bool IsSameStrategy(const PowerStrategy &other) const {
    if (stratege_type_ == TEMPORARY_CHARGING) {
      // For TEMPORARY_CHARGING, always return false to allow reapplication
      return false;
    }
    return stratege_type_ == other.stratege_type_;
  }

  uint8_t InitialPower() const { return initial_power_; }
  StrategyType Type() const { return stratege_type_; }
  void SetPowerBudget(uint16_t budget) {
    // Ensure budget is within valid range: [PORT_MIN_POWER, POWER_BUDGET]
    max_power_ = std::max((uint16_t)PORT_MIN_POWER,
                          std::min(budget, (uint16_t)POWER_BUDGET));
  }
  void DecreasePowerBudget(uint16_t decrease) {
    // Ensure we don't go below minimum power
    if (max_power_ > decrease && max_power_ - decrease >= PORT_MIN_POWER) {
      SetPowerBudget(max_power_ - decrease);
    } else {
      SetPowerBudget(PORT_MIN_POWER);
    }
  }
  void IncreasePowerBudget(uint16_t increase) {
    SetPowerBudget(max_power_ + increase);
  }
  uint16_t MaxPowerBudget() const { return max_power_; }
};

class PowerSlowChargingStrategy : public PowerStrategy {
 public:
  static constexpr StrategyType TYPE = SLOW_CHARGING;
  PowerSlowChargingStrategy() : PowerStrategy() {
    initial_power_ = 20;
    stratege_type_ = SLOW_CHARGING;
  }
  void Setup(PortManager &port_manager) override;
  int16_t Allocate(const Port &port, uint16_t remaining_power) override;
  uint16_t Apply(PortManager &port_manager) override;
};

class PowerStaticChargingStrategy : public PowerStrategy {
  PowerAllocationTable power_table_;
  uint16_t identifier_ = 0;

 public:
  static constexpr StrategyType TYPE = STATIC_CHARGING;
  PowerStaticChargingStrategy(PowerAllocationTable table, uint16_t identifier)
      : PowerStrategy() {
    initial_power_ = PORT_MAX_POWER;
    identifier_ = identifier;
    stratege_type_ = STATIC_CHARGING;
    std::memcpy(&power_table_, &table, sizeof(PowerAllocationTable));
  }
  bool IsSameStrategy(const PowerStrategy &other) const override {
    if (other.Type() != STATIC_CHARGING) {
      return false;
    }
    const PowerStaticChargingStrategy &other_strategy =
        static_cast<const PowerStaticChargingStrategy &>(other);
    return identifier_ == other_strategy.identifier_;
  }
  int16_t Allocate(const Port &port, uint16_t remaining_power) override {
    return 0;
  };
  bool ShouldApply(PortManager &port_manager, bool is_periodic) const override;
  uint16_t Apply(PortManager &port_manager) override;
  void Setup(PortManager &port_manager) override;
  uint16_t GetIdentifier() const { return identifier_; }
  uint8_t GetVersion() const { return power_table_.version; }
  const PowerAllocationTable &GetTable() const { return power_table_; }
};

class PowerTemporaryChargingStrategy : public PowerStrategy {
  uint8_t power_allocation_[NUM_PORTS] = {};

  // State tracking for event-driven exit logic
  uint8_t pending_closing_ports_ = 0;     // Bitmask of ports that are closing
  uint8_t initial_connected_status_ = 0;  // Initial connection status
  uint64_t status_stable_since_ =
      0;  // Timer for status different from initial (us)

 public:
  static constexpr StrategyType TYPE = TEMPORARY_CHARGING;
  static constexpr uint64_t CONNECTION_CHANGE_TIMEOUT = 2000000;  // 2 seconds
  PowerTemporaryChargingStrategy(uint8_t power_allocation[NUM_PORTS])
      : PowerStrategy() {
    initial_power_ = PORT_MIN_POWER;
    stratege_type_ = TEMPORARY_CHARGING;
    std::memcpy(power_allocation_, power_allocation,
                NUM_PORTS * sizeof(uint8_t));
    for (size_t i = 0; i < NUM_PORTS; i++) {
      if (power_allocation_[i] == 0) {
        power_allocation_[i] = PORT_MIN_CAP(i);
      }
    }
  }
  int16_t Allocate(const Port &port, uint16_t remaining_power) override {
    return 0;
  };
  uint16_t Apply(PortManager &port_manager) override;
  void Setup(PortManager &port_manager) override;
  void Teardown(PortManager &port_manager) override;
  bool ShouldApply(PortManager &port_manager, bool is_periodic) const override;
  void PortAttached(Port &port);
  void PortDetached(Port &port);

 private:
  void PortOpening(Port &port);
  void PortClosing(Port &port);
  void CheckConnectionChangeDebounce(PortManager &pm);
};

class PowerUSBAChargingStrategy : public PowerSlowChargingStrategy {
static constexpr uint8_t SIMULATE_PORT_ID = 4;

 public:
  static constexpr StrategyType TYPE = USBA_CHARGING;
  PowerUSBAChargingStrategy()
: PowerSlowChargingStrategy() {
stratege_type_ = USBA_CHARGING;
  }

  void Setup(PortManager &port_manager) override;
  void Teardown(PortManager &port_manager) override;
};

class PowerHighPerformanceStrategy : public PowerSlowChargingStrategy {
 public:
  static constexpr StrategyType TYPE = HIGH_PERFORMANCE_CHARGING;
  PowerHighPerformanceStrategy() : PowerSlowChargingStrategy() {
    stratege_type_ = HIGH_PERFORMANCE_CHARGING;
  }
  void Setup(PortManager &pm) override;
  void PortDetached(Port &port);
  void PortClosed(Port &port);
  void Teardown(PortManager &pm) override;

 private:
  static constexpr uint8_t TARGET_PORT = 1;
};

class PowerUltraFastSinglePortStrategy : public PowerSlowChargingStrategy {
 public:
  static constexpr StrategyType TYPE = ULTRA_FAST_SINGLE_PORT_CHARGING;
  PowerUltraFastSinglePortStrategy() : PowerSlowChargingStrategy() {
    stratege_type_ = ULTRA_FAST_SINGLE_PORT_CHARGING;
  }
  void Setup(PortManager &pm) override;
  void Teardown(PortManager &pm) override;

 private:
  void PortOpening(Port &port);
  static constexpr uint8_t TARGET_PORT = 1;
};

#endif
