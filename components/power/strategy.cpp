// We do not open source some patented strategies.
// However, you are welcome to use this as a foundation
// to implement your own custom strategies.

#include "strategy.h"

#include <sys/types.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <limits>

#include "data_types.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "port.h"
#include "port_manager.h"
#include "power_config.h"
#include "sdkconfig.h"
#include "utils.h"

#define OVERPROVISIONING_THRESHOLD (power_config.overprovisioning_threshold)
#define CAP_INCREASE_MIN_THRESHOLD (power_config.cap_increase_min_threshold)
#define CAP_DECREASE_MIN_THRESHOLD (power_config.cap_decrease_min_threshold)
#define PORT_POWER_CAP_INCREASE_THRESHOLD \
  (power_config.port_power_cap_increase_threshold)
#define PORT_POWER_CAP_INCREASE_STEP (power_config.port_power_cap_increase_step)
#define PORT_POWER_CAP_DECREASE_THRESHOLD \
  (power_config.port_power_cap_decrease_threshold)
#define HIGH_POWER_USAGE_THRESHOLD (power_config.high_power_usage_threshold)
#define LOW_POWER_USAGE_THRESHOLD (power_config.low_power_usage_threshold)
#define DEFAULT_PORT_POWER_BUDGET CONFIG_DEFAULT_PORT_POWER_BUDGET
#define PORT_BUDGET_MARGIN 5
#define IDLE_PORT_BUDGET_MARGIN 3
#ifdef CONFIG_INTERFACE_TYPE_UART
#define PORT_MIN_BUDGET 20
#define PORT_MIN_WATERMARK 15
#endif
#define HIGH_POWER_PORT_POWER_THRESHOLD 40
#define HIGH_POWER_PORT_POWER_THRESHOLD_MIN 15
#define LOW_POWER_PORT_BUDGET_THRESHOLD 25
#define PORT_POWER_USAGE_RATE_THRESHOLD 0.8
#define PORT_POWER_ADJUSTMENT_THRESHOLD 10

static const char *TAG = "PowerStrategy";
static std::array<MovingAverage, NUM_PORTS> ma_calculator = {MovingAverage()};

#ifdef CONFIG_INTERFACE_TYPE_UART
static std::array<uint8_t, NUM_PORTS> port_usage_moving_avg = {0};
static bool ignore_budget_increase = true;
static std::array<uint64_t, NUM_PORTS> last_budget_set_time = {0};
static std::array<uint32_t, NUM_PORTS> last_attched_at = {0};
static uint8_t threshold = 0;
#endif

PowerStrategy::PowerStrategy(uint32_t cooldown_period_secs) {
  cooldown_period_us_ = cooldown_period_secs * 1e6;
  last_applied_time_ = esp_timer_get_time();
}

void PowerStrategy::SetCooldownPeriod(uint32_t cooldown_period_secs) {
  cooldown_period_us_ = cooldown_period_secs * 1e6;
}

#ifdef CONFIG_INTERFACE_TYPE_UART
uint16_t PowerStrategy::Apply(PortManager &pm) { return 0; }

void PowerStrategy::PortAttached(PortManager &pm, uint8_t port_id) {
  // use event queue to handle the reallocation event
}
#endif

void PowerStrategy::PortDetached(PortManager &pm, uint8_t port_id) {
  ESP_LOGD(TAG, "Port %d detached, reallocate power budget", port_id);
}

void PowerStrategy::Reallocate(PortManager &pm) {
  ESP_LOGD(TAG, "Reallocate power budget");
}

bool PowerStrategy::IsChangeSignificant(const Port &port,
                                        int8_t power_adjustment) {
  return abs(power_adjustment) >= PORT_POWER_ADJUSTMENT_THRESHOLD;
}

void PowerStrategy::SetupInitialPower(PortManager &pm) {
  for (Port &port : pm.GetAlivePorts()) {
    port.Reset();
    uint8_t port_budget = initial_power_;
    if (port.IsTypeA()) {
      port_budget = PORT_USB_A_MAX_CAP;
    }
    port.ApplyMaxPowerBudget(port_budget);
  }
}

static uint16_t apply_static_allocation(
    const char *strategy_name, PortManager &pm, uint8_t max_power,
    uint8_t port_power_allocations[NUM_PORTS]) {
  int16_t total_attached_allocation = 0;
  int16_t remain_budget = max_power;
  for (const Port &port : pm) {
    if (port.Attached()) {
      total_attached_allocation += port_power_allocations[port.Id()];
      remain_budget -= PORT_MIN_CAP(port.Id());
    }
  }
  remain_budget = std::max(remain_budget, (int16_t)0);
  for (Port &port : pm.GetAlivePorts()) {
    uint8_t budget = port_power_allocations[port.Id()];
    if (total_attached_allocation > max_power) {
      if (port.Attached()) {
        // If allocation exceeds limit, first ensure minimum power for port,
        // then distribute remaining power proportionally
        budget = PORT_MIN_CAP(port.Id()) + port_power_allocations[port.Id()] *
                                               remain_budget /
                                               total_attached_allocation;
        budget = std::min(budget, port_power_allocations[port.Id()]);
      } else {
        // If port is not attached, set to minimum power
        budget = PORT_MIN_CAP(port.Id());
      }
    }
    uint8_t old_budget = port.MaxPowerBudget();
    if (budget != old_budget && port.SetMaxPowerBudget(budget) == ESP_OK) {
      ESP_LOGI(TAG,
               "[%s] Port %d: Power budget changed from %dW to %dW. "
               "Power allocation: %dW. Device max power: %dW. "
               "Total usage: %dW.",
               strategy_name, port.Id(), old_budget, budget,
               port_power_allocations[port.Id()], max_power,
               pm.GetPortsPowerUsage());
    }
  }
  return 0;
}

void PowerSlowChargingStrategy::SetupInitialPower(PortManager &pm) {
  for (Port &port : pm.GetAlivePorts()) {
    PowerFeatures features;
    port.GetPowerFeatures(&features);
    features.EnablePdLVPPS = false;
    features.EnablePdHVPPS = false;
    if (port.UpdatePowerFeatures(features) == ESP_OK) {
      port.Reset();
      port.ApplyMaxPowerBudget(initial_power_);
    }
  }
}

void PowerSlowChargingStrategy::Teardown(PortManager &pm) {
  for (Port &port : pm.GetAlivePorts()) {
    PowerFeatures features;
    port.GetPowerFeatures(&features);
    features.EnablePdLVPPS = true;
    features.EnablePdHVPPS = true;
    port.UpdatePowerFeatures(features);
  }
}

uint16_t PowerSlowChargingStrategy::Apply(PortManager &pm) {
#if CONFIG_MCU_MODEL_FAKE_SW3566
  return 0;
#endif
  uint8_t max_power = MaxPowerBudget();
  uint8_t port_power_allocations[NUM_PORTS];
  for (uint8_t i = 0; i < NUM_PORTS; i++) {
    port_power_allocations[i] = initial_power_;
  }
  return apply_static_allocation("PowerSlowChargingStrategy", pm, max_power,
                                 port_power_allocations);
}

int16_t PowerSlowChargingStrategy::Allocate(const Port &port,
                                            uint16_t remaining_power) {
  int adjustment = port.MinPowerCap() - port.MaxPowerBudget();
  ESP_LOGD(
      TAG,
      "[PowerSlowChargingStrategy] Port %d: actual power: %dW, power budget: "
      "%dW, adjustment: %d",
      port.Id(), port.GetPowerUsage(), port.MaxPowerBudget(), adjustment);
  return adjustment;
}

uint16_t PowerStaticChargingStrategy::Apply(PortManager &pm) {
#if CONFIG_MCU_MODEL_FAKE_SW3566
  return 0;
#endif
  uint8_t max_power = MaxPowerBudget();
  uint8_t ports_connected_status = pm.GetAttachedPortsBitMask();
  uint8_t port_power_allocations[NUM_PORTS] = {};
  for (Port &port : pm.GetAlivePorts()) {
    if (port.Attached()) {
      PowerAllocationEntry entry = power_table_.entries[ports_connected_status];
      port_power_allocations[port.Id()] = entry.power_allocation[port.Id()];
    } else {
      PowerAllocationEntry next_entry =
          power_table_.entries[ports_connected_status | (1 << port.Id())];
      port_power_allocations[port.Id()] =
          next_entry.power_allocation[port.Id()];
    }
  }
  return apply_static_allocation("PowerStaticChargingStrategy", pm, max_power,
                                 port_power_allocations);
}

void PowerStaticChargingStrategy::SetupInitialPower(PortManager &pm) {
  uint8_t ports_connected_status = pm.GetAttachedPortsBitMask();
  for (Port &port : pm.GetAlivePorts()) {
    port.Reset();
    uint8_t initial_power =
        power_table_.entries[ports_connected_status | 1 << port.Id()]
            .power_allocation[port.Id()];
    port.ApplyMaxPowerBudget(initial_power);
  }
}

uint16_t PowerTemporaryChargingStrategy::Apply(PortManager &pm) {
#if CONFIG_MCU_MODEL_FAKE_SW3566
  return 0;
#endif
  uint8_t max_power = MaxPowerBudget();
  return apply_static_allocation("PowerTemporaryChargingStrategy", pm,
                                 max_power, power_allocation_);
}

void PowerTemporaryChargingStrategy::SetupInitialPower(PortManager &pm) {
  for (Port &port : pm) {
    if (port.Dead()) {
      continue;
    }
    port.Reset();
    uint8_t budget = power_allocation_[port.Id()];
    port.ApplyMaxPowerBudget(budget > 0 ? budget : initial_power_);
  }
}

void PowerUSBAChargingStrategy::SetupInitialPower(PortManager &pm) {
  for (Port &port : pm.GetAlivePorts()) {
    if (port.Dead()) {
      continue;
    }
    if (port.Id() == SIMULATE_PORT_ID) {
      port.SetPortType(PortType::PORT_TYPE_A);
    }
    port.Reset();
    uint8_t port_budget = initial_power_;
    if (port.IsTypeA()) {
      port_budget = PORT_USB_A_MAX_CAP;
    }
    port.ApplyMaxPowerBudget(port_budget);
  }
}

void PowerUSBAChargingStrategy::Teardown(PortManager &pm) {
  Port *port = pm.GetPort(SIMULATE_PORT_ID);
  if (port && !port->Dead()) {
    port->SetPortType(PortType::PORT_TYPE_C);
  }
}
