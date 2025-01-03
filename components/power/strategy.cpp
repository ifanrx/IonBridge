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

#include "esp_log.h"
#include "esp_timer.h"
#include "port.h"
#include "port_manager.h"
#include "power_config.h"
#include "sdkconfig.h"
#include "utils.h"

#ifdef READ_TOTAL_USAGE_FROM_ADC
#include "esp_err.h"
#include "rpc.h"
#endif

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
#define PORT_MIN_BUDGET 20
#define HIGH_POWER_PORT_POWER_THRESHOLD 40
#define LOW_POWER_PORT_BUDGET_THRESHOLD 25
#define PORT_POWER_USAGE_RATE_THRESHOLD 0.8
#define PORT_POWER_ADJUSTMENT_THRESHOLD 10

static const char *TAG = "PowerStrategy";
static std::array<uint8_t, NUM_PORTS> port_usage_moving_avg = {0};
static std::array<MovingAverage, NUM_PORTS> ma_calculator = {MovingAverage()};
static bool ignore_budget_increase = true;
static std::array<uint64_t, NUM_PORTS> last_budget_set_time = {0};
static uint8_t threshold = 0;

enum StrategyStage : uint8_t {
  STAGE_INIT = 0,
  STAGE_UNDER_THRESHOLD = 1,
  STAGE_OVER_THRESHOLD = 2
};
uint8_t strategy_stage = STAGE_INIT;

PowerStrategy::PowerStrategy(uint32_t cooldown_period_secs) {
  cooldown_period_us_ = cooldown_period_secs * 1e6;
  last_applied_time_ = esp_timer_get_time();
}

void PowerStrategy::SetCooldownPeriod(uint32_t cooldown_period_secs) {
  cooldown_period_us_ = cooldown_period_secs * 1e6;
}

uint8_t PowerStrategy::Apply(PortManager &pm) {
#if CONFIG_MCU_MODEL_FAKE_SW3566
  return 0;
#endif
  return 0;
}

void PowerStrategy::PortAttached(PortManager &pm) {
  // use event queue to handle the reallocation event
  ESP_LOGD(TAG, "Port attached, reallocate power budget");
}

void PowerStrategy::PortDetached(PortManager &pm) {
  ESP_LOGD(TAG, "Port detached, reallocate power budget");
}

void PowerStrategy::Reallocate(PortManager &pm) {
  ESP_LOGD(TAG, "Reallocate power budget");
}

uint8_t PowerStrategy::HandleActualProvisioning(PortManager &pm,
                                                uint8_t total_usage) {
  uint8_t power_reduction_per_port = 0, attached_port_count = 0,
          attached_ports = 0, adjusted_ports = 0, idle_port_cap = 0;
  int8_t remaining_power = max_power_ - total_usage;
  // Limits power budget to the remaining power for all IDLE ports
  for (Port &port : pm.GetAlivePorts()) {
    if (adjusted_ports & (1 << port.Id())) {
      continue;
    }
    if (port.OverHighTemp()) {
      continue;
    }
    if (!port.Attached()) {
      idle_port_cap = remaining_power > port.MinPowerCap() ? remaining_power
                                                           : port.MinPowerCap();
      if (port.SetMaxPowerBudget(idle_port_cap)) {
        adjusted_ports |= (1 << port.Id());
      }
      continue;
    }

    attached_port_count++;
    attached_ports |= (1 << port.Id());
  }

  if (total_usage <= max_power_) {
    if (attached_port_count < 2) {
      // No need to adjust power budget
      return adjusted_ports;
    }

    for (Port &port : pm) {
      if (adjusted_ports & (1 << port.Id())) {
        continue;
      }
      if (attached_ports & (1 << port.Id())) {
        // Set the actual power cap for the port to the nearest level
        uint8_t new_cap = port.GetPowerUsage() + PORT_BUDGET_MARGIN;
        if (port.SetMaxPowerBudget(new_cap)) {
          ESP_LOGI(TAG, "Port %d new src cap %dW", port.Id(), new_cap);
          adjusted_ports |= (1 << port.Id());
        }
      }
    }
    return adjusted_ports;
  }

  power_reduction_per_port =
      (total_usage - max_power_ + attached_port_count - 1) /
      attached_port_count;
  ESP_LOGW(TAG,
           "Current power usage of %dW exceeds the maximum power of %dW. "
           "Number of attached ports: %d. Reducing power by %dW per port.",
           total_usage, max_power_, attached_port_count,
           power_reduction_per_port);
  for (Port &port : pm) {
    if (adjusted_ports & (1 << port.Id())) {
      continue;
    }
    bool attached = attached_ports & (1 << port.Id());
    if (attached) {
      ESP_LOGI(TAG, "Reducing power cap for attached port %d by %dW (CAP: %dW)",
               port.Id(), power_reduction_per_port, port.MaxPowerBudget());
      if (port.DecreasePowerBudget(power_reduction_per_port)) {
        adjusted_ports |= (1 << port.Id());
      }
    }
  }
  return adjusted_ports;
}

bool PowerStrategy::IsChangeSignificant(const Port &port,
                                        int8_t power_adjustment) {
  return abs(power_adjustment) >= PORT_POWER_ADJUSTMENT_THRESHOLD;
}

void PowerStrategy::SetupInitialPower(PortManager &pm) {
  for (Port &port : pm.GetAlivePorts()) {
    port.Reset();
    port.ApplyMaxPowerBudget(initial_power_);
  }
}

static uint8_t apply_static_allocation(
    const char *strategy_name, PortManager &pm, uint8_t max_power,
    uint8_t port_power_allocations[NUM_PORTS]) {
  uint8_t total_usage = pm.GetPortsPowerUsage();
  uint8_t total_attached_allocation = 0;
  uint8_t remain_budget = max_power;
  for (const Port &port : pm) {
    if (port.Attached()) {
      total_attached_allocation += port_power_allocations[port.Id()];
    }
    remain_budget -= PORT_MIN_CAP(port.Id());
  }
  remain_budget = std::max(remain_budget, (uint8_t)0);
  for (Port &port : pm.GetAlivePorts()) {
    uint8_t budget = port_power_allocations[port.Id()];
    if (total_attached_allocation > max_power) {
      // 分配的超过上限，先保证 5 口最小功率，再将剩余的功率按比例分配
      budget = PORT_MIN_CAP(port.Id()) + port_power_allocations[port.Id()] *
                                             remain_budget /
                                             total_attached_allocation;
      budget = std::min(budget, port_power_allocations[port.Id()]);
    }
    uint8_t old_budget = port.MaxPowerBudget();
    if (budget != old_budget && port.SetMaxPowerBudget(budget)) {
      ESP_LOGI(TAG,
               "[%s] Port %d: Power budget changed from %dW to %dW. "
               "Power allocation: %dW. Device max power: %dW. "
               "Total usage: %dW.",
               strategy_name, port.Id(), old_budget, budget,
               port_power_allocations[port.Id()], max_power, total_usage);
    }
  }
  return 0;
}

uint8_t PowerSlowChargingStrategy::Apply(PortManager &pm) {
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

int8_t PowerSlowChargingStrategy::Allocate(const Port &port,
                                           uint8_t remaining_power) {
  int adjustment = port.MinPowerCap() - port.MaxPowerBudget();
  ESP_LOGD(
      TAG,
      "[PowerSlowChargingStrategy] Port %d: actual power: %dW, power budget: "
      "%dW, adjustment: %d",
      port.Id(), port.GetPowerUsage(), port.MaxPowerBudget(), adjustment);
  return adjustment;
}

uint8_t PowerStaticChargingStrategy::Apply(PortManager &pm) {
#if CONFIG_MCU_MODEL_FAKE_SW3566
  return 0;
#endif
  uint8_t max_power = MaxPowerBudget();
  uint8_t ports_connected_status = pm.GetPortsAttachedStatus();
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
  uint8_t ports_connected_status = pm.GetPortsAttachedStatus();
  for (Port &port : pm.GetAlivePorts()) {
    port.Reset();
    uint8_t initial_power =
        power_table_.entries[ports_connected_status | 1 << port.Id()]
            .power_allocation[port.Id()];
    port.ApplyMaxPowerBudget(initial_power);
  }
}

uint8_t PowerTemporaryChargingStrategy::Apply(PortManager &pm) {
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
