#ifndef POWER_STRATEGY_H_
#define POWER_STRATEGY_H_

#include <algorithm>
#include <cstdint>
#include <cstring>

#include "port.h"
#include "port_manager.h"
#include "power_config.h"

#define POWER_BUDGET (power_config.power_budget)

enum StrategyType : uint8_t {
  SLOW_CHARGING = 0x01,
  STATIC_CHARGING = 0x02,
  TEMPORARY_CHARGING = 0x03,
  LEGACY_CHARGING = 0x04,
  USBA_CHARGING = 0x06,
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
  // Safety margin for non-adjustable ports
  uint8_t safety_margin_ = 5;
  uint64_t last_stage_switch_time_ = 0;
  uint8_t strategy_stage_ = STAGE_INIT;

 protected:
  int64_t last_applied_time_;
  // Sufficient time before reapplying power changes
  uint64_t cooldown_period_us_;
  uint8_t initial_power_ = 0;
  StrategyType stratege_type_ = UNKNOWN_STRATEGY;

 public:
  virtual ~PowerStrategy() = default;
  virtual int16_t Allocate(const Port &port, uint16_t remaining_power) = 0;

  PowerStrategy(uint32_t cooldown_period_secs);
  void SetCooldownPeriod(uint32_t cooldown_period_secs);
  bool IsChangeSignificant(const Port &port, int8_t power_adjustment);
  virtual uint16_t Apply(PortManager &port_manager);
  virtual void PortAttached(PortManager &port_manager, uint8_t port_id);
  virtual void PortDetached(PortManager &port_manager, uint8_t port_id);
  virtual void Reallocate(PortManager &port_manager);
  uint8_t HandleActualProvisioning(PortManager &port_manager,
                                   uint16_t total_usage);
  virtual void SetupInitialPower(PortManager &port_manager);
  virtual void Teardown(PortManager &port_manager) {};
  uint8_t InitialPower() const { return initial_power_; }
  StrategyType Type() const { return stratege_type_; }
  void SetPowerBudget(uint16_t budget) {
    max_power_ = std::min(budget, (uint16_t)POWER_BUDGET);
  }
  void DecreasePowerBudget(uint16_t decrease) {
    if (max_power_ > decrease) {
      SetPowerBudget(max_power_ - decrease);
    }
  }
  void IncreasePowerBudget(uint16_t increase) {
    SetPowerBudget(max_power_ + increase);
  }
  uint16_t MaxPowerBudget() const { return max_power_; }
  uint8_t Stage() const { return strategy_stage_; }
  uint64_t LastStageSwitchTime() const { return last_stage_switch_time_; }
};

class PowerSlowChargingStrategy : public PowerStrategy {
 public:
  static constexpr StrategyType TYPE = SLOW_CHARGING;
  PowerSlowChargingStrategy(uint32_t cooldown_period_secs)
      : PowerStrategy(cooldown_period_secs) {
    initial_power_ = 20;
    stratege_type_ = SLOW_CHARGING;
  }
  void SetupInitialPower(PortManager &port_manager) override;
  void Teardown(PortManager &port_manager) override;
  int16_t Allocate(const Port &port, uint16_t remaining_power) override;
  uint16_t Apply(PortManager &port_manager) override;
};

class PowerStaticChargingStrategy : public PowerStrategy {
  PowerAllocationTable power_table_;
  uint16_t identifier_ = 0;

 public:
  static constexpr StrategyType TYPE = STATIC_CHARGING;
  PowerStaticChargingStrategy(uint32_t cooldown_period_secs,
                              PowerAllocationTable table, uint16_t identifier)
      : PowerStrategy(cooldown_period_secs) {
    initial_power_ = PORT_MAX_POWER;
    identifier_ = identifier;
    stratege_type_ = STATIC_CHARGING;
    std::memcpy(&power_table_, &table, sizeof(PowerAllocationTable));
  }
  int16_t Allocate(const Port &port, uint16_t remaining_power) override {
    return 0;
  };
  uint16_t Apply(PortManager &port_manager) override;
  void SetupInitialPower(PortManager &port_manager) override;
  void GetIdentifier(uint16_t *identifier) const { *identifier = identifier_; }
  void GetVersion(uint8_t *version) const { *version = power_table_.version; }
};

class PowerTemporaryChargingStrategy : public PowerStrategy {
  uint8_t power_allocation_[NUM_PORTS] = {};

 public:
  static constexpr StrategyType TYPE = TEMPORARY_CHARGING;
  PowerTemporaryChargingStrategy(uint32_t cooldown_period_secs,
                                 uint8_t power_allocation[NUM_PORTS])
      : PowerStrategy(cooldown_period_secs) {
    initial_power_ = PORT_MIN_POWER;
    stratege_type_ = TEMPORARY_CHARGING;
    std::memcpy(power_allocation_, power_allocation,
                NUM_PORTS * sizeof(uint8_t));
  }
  int16_t Allocate(const Port &port, uint16_t remaining_power) override {
    return 0;
  };
  uint16_t Apply(PortManager &port_manager) override;
  void SetupInitialPower(PortManager &port_manager) override;
};

class LegacyChargingStrategy : public PowerStrategy {
  uint8_t max_power_ = POWER_BUDGET;

 public:
  static constexpr StrategyType TYPE = LEGACY_CHARGING;
  LegacyChargingStrategy(uint32_t cooldown_period_secs)
      : PowerStrategy(cooldown_period_secs) {
    initial_power_ = PORT_MAX_POWER;
    stratege_type_ = LEGACY_CHARGING;
  }
  uint16_t Apply(PortManager &pm) override;
};

class PowerUSBAChargingStrategy : public PowerSlowChargingStrategy {
  static constexpr uint8_t SIMULATE_PORT_ID = 4;

 public:
  static constexpr StrategyType TYPE = USBA_CHARGING;
  PowerUSBAChargingStrategy(uint32_t cooldown_period_secs)
      : PowerSlowChargingStrategy(cooldown_period_secs) {
    stratege_type_ = USBA_CHARGING;
  }

  void SetupInitialPower(PortManager &port_manager) override;
  void Teardown(PortManager &port_manager) override;
};

#endif
