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
#include "port_state.h"
#include "power_allocator.h"
#include "power_config.h"
#include "sdkconfig.h"

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
#ifdef CONFIG_INTERFACE_TYPE_I2C
#define PORT_MIN_BUDGET 15
#define PORT_REALLOCATION_THRESHOLD 8
#endif
#define HIGH_POWER_PORT_POWER_THRESHOLD 40
#define HIGH_POWER_PORT_POWER_THRESHOLD_MIN 15
#define LOW_POWER_PORT_BUDGET_THRESHOLD 25
#define PORT_POWER_USAGE_RATE_THRESHOLD 0.8
#define PORT_POWER_ADJUSTMENT_THRESHOLD 10
#define STAGE_SWITCH_COOLDOWN 50e6 * 2
#define BUDGET_SET_COOLDOWN 60e6

static const char *TAG = "PowerStrategy";

static uint16_t apply_static_allocation(
    const char *strategy_name, PortManager &pm, uint8_t max_power,
    uint8_t port_power_allocations[NUM_PORTS], uint8_t ports_connected_status) {
  int16_t total_attached_allocation = 0;

  // add up total power allocation base on ports_connected_status
  for (uint8_t i = 0; i < NUM_PORTS; i++) {
    if (ports_connected_status & (1 << i)) {
      total_attached_allocation += port_power_allocations[i];
    }
  }
  float scale = 1.0;
  // because the power allocation table is based on POWER_BUDGET,
  // if the actual max power is lower, we need to scale down the allocation
  if (total_attached_allocation > max_power) {
    scale = (float)max_power / total_attached_allocation;
    ESP_LOGD(TAG,
             "[%s] Total power allocation %dW exceeds max power %dW, "
             "scaling down by factor %.2f",
             strategy_name, total_attached_allocation, max_power, scale);
  }

  for (Port &port : pm) {
    uint8_t budget =
        port_power_allocations[port.Id()] * scale;  // apply scaling if needed
    budget = MAX(budget, port.MinPowerCap());

    uint8_t old_budget = port.MaxPowerBudget();

    if (port.Abnormal()) {
      continue;
    }

    if (budget != old_budget &&
        port.SetMaxPowerBudget(budget, budget, true) == ESP_OK) {
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

void PowerSlowChargingStrategy::Setup(PortManager &pm) {
  for (Port &port : pm.GetAlivePorts()) {
    port.Detach();
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
                                 port_power_allocations,
                                 pm.GetAttachedPortsBitMask());
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

/**
 * Checks if the set of connected ports has changed since the last call.
 *
 * @param pm Reference to the PortManager to query attached ports.
 * @param Unused boolean parameter.
 * @return True if the set of connected ports has changed, false otherwise.
 *
 * This function uses a static variable to track the previous state of connected
 * ports. It returns true only when the current state differs from the previous
 * state.
 */
bool PowerStaticChargingStrategy::ShouldApply(PortManager &pm, bool) const {
  static uint8_t previous_connected_ports = 0;
  uint8_t current_connected_ports = pm.GetAttachedPortsBitMask();
  bool changed = current_connected_ports != previous_connected_ports;
  previous_connected_ports = current_connected_ports;
  return changed;
}

uint16_t PowerStaticChargingStrategy::Apply(PortManager &pm) {
#if CONFIG_MCU_MODEL_FAKE_SW3566
  return 0;
#endif
  uint8_t max_power = MaxPowerBudget();
  uint8_t ports_connected_status = pm.GetAttachedPortsBitMask();
  uint8_t port_power_allocations[NUM_PORTS] = {};
  for (Port &port : pm.GetAlivePorts()) {
    if (ports_connected_status & (1 << port.Id())) {
      if (ports_connected_status >= 32 || port.Id() >= NUM_PORTS) {
        ESP_LOGE(TAG, "Invalid port status %d or port ID %d",
                 ports_connected_status, port.Id());
        continue;
      }
      PowerAllocationEntry entry = power_table_.entries[ports_connected_status];
      port_power_allocations[port.Id()] = entry.power_allocation[port.Id()];
    } else {
      // Update the power allocation for a port that is being attached.
      // 1. Compute the new port connection status by setting the bit
      //    corresponding to the current port.
      // 2. Use this status as an index into the power allocation table to get
      //    the next allocation entry.
      // 3. Assign the allocated power for this port from the new entry.
      uint8_t next_status = ports_connected_status | (1 << port.Id());
      if (next_status >= 32 || port.Id() >= NUM_PORTS) {
        ESP_LOGE(TAG, "Invalid next status %d or port ID %d", next_status,
                 port.Id());
        port_power_allocations[port.Id()] = 0;
      } else {
        PowerAllocationEntry next_entry = power_table_.entries[next_status];
        port_power_allocations[port.Id()] =
            next_entry.power_allocation[port.Id()];
      }
    }
  }
  return apply_static_allocation("PowerStaticChargingStrategy", pm, max_power,
                                 port_power_allocations,
                                 ports_connected_status);
}

void PowerStaticChargingStrategy::Setup(PortManager &pm) {
  uint8_t ports_connected_status = pm.GetAttachedPortsBitMask();
  for (Port &port : pm.GetAlivePorts()) {
    port.Detach();
    uint8_t setup_status = ports_connected_status | (1 << port.Id());
    uint8_t initial_power;
    if (setup_status >= 32 || port.Id() >= NUM_PORTS) {
      ESP_LOGE(TAG, "Invalid setup status %d or port ID %d", setup_status,
               port.Id());
      initial_power = initial_power_;  // Fallback to configured default
    } else {
      initial_power =
          power_table_.entries[setup_status].power_allocation[port.Id()];
    }
    port.ApplyMaxPowerBudget(initial_power);
  }
}

uint16_t PowerTemporaryChargingStrategy::Apply(PortManager &pm) {
#if CONFIG_MCU_MODEL_FAKE_SW3566
  return 0;
#endif
  uint8_t max_power = MaxPowerBudget();
  return apply_static_allocation("PowerTemporaryChargingStrategy", pm,
                                 max_power, power_allocation_,
                                 pm.GetAttachedPortsBitMask());
}

void PowerTemporaryChargingStrategy::Setup(PortManager &pm) {
  // Record initial connection status for debounce
  initial_connected_status_ = pm.GetAttachedPortsBitMask();
  pending_closing_ports_ = 0;
  status_stable_since_ = 0;

  // First, set up all ports (including Reset which may trigger state changes)
  for (Port &port : pm) {
    if (port.Dead()) {
      continue;
    }
    port.Detach();
    uint8_t budget = power_allocation_[port.Id()];
    port.ApplyMaxPowerBudget(budget > 0 ? budget : initial_power_);
  }

  // Then, register callbacks after all ports are set up
  // This avoids callbacks being triggered by Reset() above
  for (Port &port : pm.GetAlivePorts()) {
    port.RegisterStateChangeCallback(
        PortStateType::ATTACHED, [this](Port &p) { this->PortAttached(p); });
    port.RegisterStateChangeCallback(PortStateType::ACTIVE, [this](Port &p) {
      if (p.GetPreviousStateType() == PortStateType::ATTACHED) {
        this->PortDetached(p);
      }
    });
    port.RegisterStateChangeCallback(PortStateType::OPENING,
                                     [this](Port &p) { this->PortOpening(p); });
    port.RegisterStateChangeCallback(PortStateType::CLOSING,
                                     [this](Port &p) { this->PortClosing(p); });
  }
}

void PowerTemporaryChargingStrategy::Teardown(PortManager &pm) {
  // Unregister all callbacks to prevent memory leaks
  for (Port &port : pm) {
    port.UnregisterAll();
  }

  // Trigger exit callback
  if (exit_callback_) {
    exit_callback_();
  }
}

bool PowerTemporaryChargingStrategy::ShouldApply(PortManager &port_manager,
                                                 bool is_periodic) const {
  if (is_periodic) {
    // Periodic check for connection status change
    const_cast<PowerTemporaryChargingStrategy *>(this)
        ->CheckConnectionChangeDebounce(port_manager);
  }
  // No need to actually reapply allocation, handled by event callbacks
  return false;
}

void PowerTemporaryChargingStrategy::PortAttached(Port &port) {
  ESP_LOGD(TAG, "Port %d attached", port.Id());
  CheckConnectionChangeDebounce(PortManager::GetInstance());
}

void PowerTemporaryChargingStrategy::PortDetached(Port &port) {
  ESP_LOGD(TAG, "Port %d detached", port.Id());
  CheckConnectionChangeDebounce(PortManager::GetInstance());
}

void PowerTemporaryChargingStrategy::CheckConnectionChangeDebounce(
    PortManager &pm) {
  uint8_t current_status = pm.GetAttachedPortsBitMask();
  uint64_t now = esp_timer_get_time();

  // Status equals initial, reset timer
  if (current_status == initial_connected_status_) {
    status_stable_since_ = 0;
    return;
  }

  // Status is different from initial
  // First time seeing different status, start timer
  if (status_stable_since_ == 0) {
    status_stable_since_ = now;
    ESP_LOGD(TAG,
             "Connection status different from initial (current: 0x%02X, "
             "initial: 0x%02X), start timer",
             current_status, initial_connected_status_);
    return;
  }

  // Status has been different for 2 seconds, exit
  if (now - status_stable_since_ >= CONNECTION_CHANGE_TIMEOUT) {
    ESP_LOGI(TAG,
             "Exit temporary allocator: status different from initial "
             "(current: 0x%02X, initial: 0x%02X) for 2s",
             current_status, initial_connected_status_);
    PowerAllocator::GetInstance().SetDefaultStrategy();
  }
}

void PowerTemporaryChargingStrategy::PortOpening(Port &port) {
  pending_closing_ports_ &= ~(1 << port.Id());  // Clear closing flag
  ESP_LOGI(TAG, "Exit temporary allocator on port %d opening", port.Id());
  PowerAllocator::GetInstance().SetDefaultStrategy();
}

void PowerTemporaryChargingStrategy::PortClosing(Port &port) {
  PortManager &pm = PortManager::GetInstance();
  pending_closing_ports_ |= (1 << port.Id());

  // Calculate alive ports mask dynamically
  uint8_t alive_mask = pm.GetPortsBitMask(
      [](const Port &p) { return !p.Abnormal() && !p.IsPowerLimiting(); });

  if (pending_closing_ports_ != alive_mask) {
    // Not all ports are closing = manual port close, exit
    ESP_LOGI(TAG, "Exit temporary allocator on port %d closing (manual)",
             port.Id());
    PowerAllocator::GetInstance().SetDefaultStrategy();
  } else {
    // All ports are closing = device power off, keep strategy
    ESP_LOGD(TAG, "All ports closing (power off), keeping temporary allocator");
  }
}

void PowerUSBAChargingStrategy::Setup(PortManager &pm) {
  for (Port &port : pm.GetAlivePorts()) {
    if (port.Dead()) {
      continue;
    }
    if (port.Id() == SIMULATE_PORT_ID) {
      port.SetPortType(PortType::PORT_TYPE_A);
    }
    port.Detach();
    port.ApplyMaxPowerBudget(initial_power_);
  }
}

void PowerUSBAChargingStrategy::Teardown(PortManager &pm) {
  Port *port = pm.GetPort(SIMULATE_PORT_ID);
  if (port && !port->Dead()) {
    port->SetPortType(PortType::PORT_TYPE_C);
  }
}

void PowerHighPerformanceStrategy::Setup(PortManager &pm) {
  PowerAllocator::GetInstance().SetTemperatureMode(
      TemperatureMode::POWER_PRIORITY);

  // Call parent setup first to get basic PowerSlowChargingStrategy
  PowerSlowChargingStrategy::Setup(pm);

  // Configure cable compensation for all ports
  for (uint8_t i = 0; i < NUM_PORTS; i++) {
    Port *port = pm.GetPort(i);
    if (!port) continue;

    CableCompensation resistance = CableCompensation::VALUE_65MOHM;
    FixedVoltageOffset offset = FixedVoltageOffset::OFFSET_0MV;
    if (i == TARGET_PORT) {
      resistance = CableCompensation::VALUE_100MOHM;
      offset = FixedVoltageOffset::OFFSET_200MV;
    }
    port->SetCableCompensation(true, resistance, offset, true);
  }

  // Add our additional callbacks for C1 port state changes
  Port *target_port = pm.GetPort(TARGET_PORT);
  if (target_port) {
    target_port->RegisterStateChangeCallback(
        PortStateType::INACTIVE, [this](Port &p) { this->PortClosed(p); });
    // Register callback for device disconnection (ATTACHED -> ACTIVE)
    target_port->RegisterStateChangeCallback(
        PortStateType::ACTIVE, [this](Port &p) {
          if (p.GetPreviousStateType() == PortStateType::ATTACHED) {
            this->PortDetached(p);
          }
        });
  }
}

void PowerHighPerformanceStrategy::PortDetached(Port &port) {
  if (port.Id() == TARGET_PORT) {
    ESP_LOGI(TAG, "C1 port detached, exiting high performance mode");
    PowerAllocator::GetInstance().SetStrategy<PowerSlowChargingStrategy>();
  }
}

void PowerHighPerformanceStrategy::PortClosed(Port &port) {
  // Check if C1 port was closed
  if (port.Id() == TARGET_PORT) {
    PowerAllocator::GetInstance().SetStrategy<PowerSlowChargingStrategy>();
  }
}

void PowerHighPerformanceStrategy::Teardown(PortManager &pm) {
  ESP_LOGI(TAG, "Tearing down High Performance Charging Strategy");

  PowerSlowChargingStrategy::Teardown(pm);

  // Restore all ports to default cable compensation: 65mΩ + 0mV
  for (uint8_t i = 0; i < NUM_PORTS; i++) {
    Port *port = pm.GetPort(i);
    if (!port) continue;

    port->SetCableCompensation(true, CableCompensation::VALUE_65MOHM,
                               FixedVoltageOffset::OFFSET_0MV, true);
  }
}

void PowerUltraFastSinglePortStrategy::Setup(PortManager &pm) {
  PowerAllocator &allocator = PowerAllocator::GetInstance();

  allocator.SetTemperatureMode(TemperatureMode::POWER_PRIORITY);

  // Call parent setup first to get basic PowerSlowChargingStrategy behavior
  PowerSlowChargingStrategy::Setup(pm);

  // Close all ports first
  for (Port &port : pm) {
    port.Close();
  }

  // Reset and reconfigure C1 port
  Port *target_port = pm.GetPort(TARGET_PORT);
  if (target_port) {
    // Reset C1 state (disconnect, reset data, reconnect to ACTIVE)
    target_port->ResetState();

    // Configure cable compensation: 100mΩ + 200mV
    target_port->SetCableCompensation(true, CableCompensation::VALUE_100MOHM,
                                      FixedVoltageOffset::OFFSET_200MV, true);

    // Ensure C1 is open (ResetState already sets to ACTIVE, but explicit for
    // clarity)
    target_port->Open();
  }

  // Register PortOpening callback for non-C1 ports
  for (Port &port : pm) {
    if (port.Id() == TARGET_PORT) continue;
    port.RegisterStateChangeCallback(PortStateType::OPENING,
                                     [this](Port &p) { this->PortOpening(p); });
  }
}

void PowerUltraFastSinglePortStrategy::Teardown(PortManager &pm) {
  ESP_LOGI(TAG, "Tearing down Ultra Fast Single Port Charging Strategy");

  // Call parent teardown first (this will unregister all callbacks)
  PowerSlowChargingStrategy::Teardown(pm);

  // Restore C1 port cable compensation to default: 65mΩ + 0mV
  Port *target_port = pm.GetPort(TARGET_PORT);
  if (target_port) {
    target_port->SetCableCompensation(true, CableCompensation::VALUE_65MOHM,
                                      FixedVoltageOffset::OFFSET_0MV, true);
  }

  // Open all non-C1 ports
  for (Port &port : pm) {
    port.Open();
  }
}

void PowerUltraFastSinglePortStrategy::PortOpening(Port &port) {
  ESP_LOGI(TAG, "Non-C1 port %d opening, exiting to fast charging mode",
           port.Id());
  PowerAllocator::GetInstance().SetStrategy<PowerSlowChargingStrategy>();
}
