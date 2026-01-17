#include "port_state.h"

#include <cstdint>

#include "animation_data_source.h"
#include "animation_manager.h"
#include "display_manager.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ionbridge.h"
#include "port.h"
#include "rpc.h"
#include "sdkconfig.h"

#ifdef CONFIG_MCU_MODEL_SW3566
#include "esp_check.h"
#include "sw3566_data_types.h"
#endif

static const char *TAG = "PortState";

#define UPGRADE_TASK_STACK_SIZE CONFIG_UPGRADE_TASK_STACK_SIZE
#define UPGRADE_TASK_PRIORITY CONFIG_UPGRADE_TASK_PRIORITY

void PortAttachedState::Entering(Port &port) {
  AnimationDataSource::SetLastInsertedPortId(port.Id());
  DisplayManager::GetInstance().SetNextAnimation(AnimationType::DEVICE_INSERTED,
                                                 false);
}

void PortAttachedState::Exiting(Port &port) {
  // Notify animation to detect port change immediately
  AnimationManager::GetInstance().NotifyUpdate();
}

void PortDetachingState::Entering(Port &port) { port.StartDetachingTimer(); }

void PortDetachingState::Handle(Port &port) {
  // Check if device reconnected during debounce window
  if (port.IsConnected()) {
    ESP_LOGI(TAG,
             "Port %d: Device reconnected during detaching, reverting to "
             "ATTACHED",
             port.Id());
    port.SetState(PortStateType::ATTACHED);
  }
  // Otherwise wait for timer to expire
}

void PortDetachingState::Exiting(Port &port) { port.StopDetachingTimer(); }

void PortInactiveState::Handle(Port &port) {
  // update port historical data while port is inactive
  port.SetInactiveHistoricalData();
}

void PortOpeningState::Handle(Port &port) {
  esp_err_t err = rpc::mcu::port_connect(port.Id());
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Unable to turn on port %d, error: 0x%04X", port.Id(), err);
    port.SetState(PortStateType::CHECKING);
    return;
  }
  ESP_LOGD(TAG, "Port %d is turned on", port.Id());
  port.Reset();
  port.SetState(PortStateType::ACTIVE);
}

void PortClosingState::Handle(Port &port) {
  esp_err_t err = rpc::mcu::port_disconnect(port.Id());
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Unable to turn off port %d, error: 0x%04X", port.Id(), err);
    port.SetState(PortStateType::CHECKING);
    return;
  }
  ESP_LOGD(TAG, "Port %d is turned off", port.Id());
  port.Reset();
  port.SetState(PortStateType::INACTIVE);
}

void PortOverTempWarningState::Handle(Port &port) {
  uint16_t temperature = port.Temperature();
  if (temperature > max_temperature_) {
    port.SetState(PortStateType::COOLING);
    return;
  }
  if (temperature > alert_temperature_) {
    port.SetState(PortStateType::OVER_TEMP_ALERT);
    return;
  }
  if (temperature < exit_temperature_) {
    port.SetState(PortStateType::ACTIVE);
    return;
  }
}

void PortOverTempAlertState::Handle(Port &port) {
  static int64_t decrease_power_at = 0;
  uint16_t temperature = port.Temperature();
  if (temperature > max_temperature_) {
    port.SetState(PortStateType::COOLING);
    return;
  }
  if (temperature < exit_temperature_) {
    port.SetState(PortStateType::OVER_TEMP_WARNING);
    return;
  }
  int64_t now = esp_timer_get_time();
  if (now - decrease_power_at > CONFIG_PORT_TEMP_COOLDOWN_TIME * 1e6) {
    decrease_power_at = now;
    port.DecreasePowerBudget(CONFIG_PORT_OVER_TEMP_DECREASE);
  }
}

void PortCoolingState::Handle(Port &port) {
  esp_err_t err;
  if (!closed_) {
    err = rpc::mcu::port_disconnect(port.Id());
    if (err != ESP_OK) {
      ESP_ERROR_COMPLAIN(err, "Unable to disconnect port %d", port.Id());
      port.SetState(PortStateType::CHECKING);
      return;
    }
    closed_ = true;
    return;
  }

  // Port is closed, now check the port temperature
  if (port.Temperature() < exit_temperature_) {
    port.SetState(PortStateType::OPENING);
  }
}

void PortRecoveringState::Handle(Port &port) {
  esp_err_t ret = ESP_FAIL;
#ifdef CONFIG_MCU_MODEL_SW3566
  KeepAliveStatus status;
  ESP_GOTO_ON_ERROR(rpc::mcu::keep_alive(port.Id(), &status), BRINGUP, TAG,
                    "Port %d is not responding to keep-alive. Attempting to "
                    "bring it up",
                    port.Id());
  ESP_GOTO_ON_FALSE(status == KeepAliveStatus::USER_APPLICATION_ALIVE,
                    ESP_ERR_INVALID_STATE, BRINGUP, TAG,
                    "Port %d is not in user application state, attempting to "
                    "bring it up",
                    port.Id());

RECOVERED:
#endif
  // Recovered, reset port
  port.Reset();
  if (port.ApplyMaxPowerBudget(port.MinPowerCap())) {
    port.SetState(PortStateType::ACTIVE);
    return;
  }
  goto DEAD;
#ifdef CONFIG_MCU_MODEL_SW3566
BRINGUP:
  ESP_GOTO_ON_ERROR(rpc::hard_reset_mcu(port.Id()), DEAD, TAG,
                    "Failed to perform a hard reset on port %d.", port.Id());
  ret = rpc::mcu::bringup(port.Id(), true);
  if (ret == ESP_OK) {
    goto RECOVERED;
  }
#else
#endif
DEAD:
  ESP_ERROR_COMPLAIN(ret, "Failed to resurrect port %d.", port.Id());
  port.SetState(PortStateType::DEAD);
}

void PortCheckingState::Entering(Port &port) {
  port.StopCheckingWatchdog();
  port.StartCheckingWatchdog();
}

void PortCheckingState::Handle(Port &port) {
  if (port.UpdatedAt() > port.CheckingAt()) {
    // Seems the port is still alive, revert to previous state
    port.RevertToPreviousState();
    return;
  }
  // Do nothing, wait for the port to be updated or timer to be executed
}

void PortCheckingState::Exiting(Port &port) { port.StopCheckingWatchdog(); }

void PortDeadState::Entering(Port &port) { port.Reset(); }
