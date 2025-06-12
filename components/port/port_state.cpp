#include "port_state.h"

#include <cstdint>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ionbridge.h"
#include "port.h"
#include "power_allocator.h"
#include "rpc.h"
#include "sdkconfig.h"
#include "utils.h"

#ifdef CONFIG_MCU_MODEL_SW3566
#include "esp_check.h"
#include "sw3566_data_types.h"
#endif

#define PORT_CHECKING_TIMER_MS CONFIG_PORT_CHECKING_TIMER_MS

static const char *TAG = "PortState";

void PortActiveState::Handle(Port &port) {
  if (port.Attached()) {
    port.SetState(PortStateType::ATTACHED);
    PowerAllocator::GetInstance().EnqueueEvent(
        PowerAllocatorEventType::PORT_ATTACHED, port.Id());
  }
}

void PortAttachedState::Handle(Port &port) {
  if (!port.Attached()) {
    ESP_LOGD(TAG, "Port %d is detached", port.Id());
    port.UnsubscribePDPcapStreaming();
    port.Reset();
    port.SetState(PortStateType::ACTIVE);
    PowerAllocator::GetInstance().EnqueueEvent(
        PowerAllocatorEventType::PORT_DETACHED, port.Id());
  }
}

void PortInactiveState::Handle(Port &port) {
  // update port historical data while port is inactive
  port.SetInactiveHistoricalData();
}

void PortOpeningState::Handle(Port &port) {
  esp_err_t err = rpc::mcu::port_connect(port.Id());
  if (err != ESP_OK) {
    ESP_ERROR_COMPLAIN(err, "Unable to turn on port %d", port.Id());
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
    ESP_ERROR_COMPLAIN(err, "Unable to turn off port %d", port.Id());
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
  StopWatchdog();
  enter_time_ = esp_timer_get_time();
  StartWatchdog(port);
}

void PortCheckingState::Handle(Port &port) {
  if (port.UpdatedAt() > enter_time_) {
    // Seems the port is still alive, revert to previous state
    port.RevertToPreviousState();
    return;
  }
  // Do nothing, wait for the port to be updated or timer to be executed
}

void PortCheckingState::Exiting(Port &port) {
  StopWatchdog();
  enter_time_ = -1;
}

void PortCheckingState::StartWatchdog(Port &port) {
  if (checking_timer_ != nullptr) {
    ESP_LOGW(TAG, "Port watchdog has already started for port %d.", port.Id());
    return;
  }
  const esp_timer_create_args_t timer_args = {
      .callback = WatchdogCallback,
      .arg = (void *)&port,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "port_watchdog",
      .skip_unhandled_events = true,
  };
  esp_err_t err = esp_timer_create(&timer_args, &checking_timer_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create port watchdog timer: %s",
             esp_err_to_name(err));
    return;
  }
  err = esp_timer_start_once(checking_timer_, PORT_CHECKING_TIMER_MS * 1000);
  ESP_LOGD(TAG, "Port watchdog has started for port %d.", port.Id());
}

void PortCheckingState::StopWatchdog() {
  if (checking_timer_ != nullptr) {
    esp_timer_stop(checking_timer_);
    esp_timer_delete(checking_timer_);
    DELAY_MS(50);
    checking_timer_ = nullptr;
    ESP_LOGD(TAG, "Port watchdog stopped");
  }
}

void PortCheckingState::WatchdogCallback(void *arg) {
#ifdef CONFIG_MCU_MODEL_SW3566
  Port *port = (Port *)arg;
  // Send the keep-alive message to the port
  KeepAliveStatus status;
  uint32_t uptime_ms;
  uint32_t reboot_reason;
  esp_err_t ret = ESP_OK;
  ESP_LOGD(TAG, "Port %d watchdog callback started.", port->Id());
  for (int i = 0; i < 3; i++) {
    ret = rpc::mcu::keep_alive(port->Id(), &status, &uptime_ms, &reboot_reason);
    if (ret == ESP_OK) {
      break;
    }
    DELAY_MS(20);
  }
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Port %d keep-alive failed: %d, recovering", port->Id(), ret);
    port->SetState(PortStateType::RECOVERING);
    return;
  }
  if (status == KeepAliveStatus::USER_APPLICATION_ALIVE) {
    // Port is still alive, revert to previous state
    port->RevertToPreviousState();
    return;
  }
  if (status == KeepAliveStatus::BOOTLOADER_ALIVE) {
    ESP_LOGW(TAG,
             "Port %d is in bootloader, uptime: %" PRIu32
             "ms, reboot_reason: 0x%" PRIx32,
             port->Id(), uptime_ms, reboot_reason);
    port->Reboot();
    return;
  }
#endif
}
