#include <cstdint>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "ionbridge.h"
#include "port.h"
#include "power_allocator.h"
#include "rpc.h"
#include "sdkconfig.h"

#ifdef CONFIG_MCU_MODEL_SW3566
#include "sw3566_data_types.h"
#include "utils.h"
#endif

#define PORT_CHECKING_TIMER_MS CONFIG_PORT_CHECKING_TIMER_MS

static const char *TAG = "PortStage";

void PortActiveState::Handle(Port &port) {
  if (port.Attached()) {
    port.SetState(PortStateType::ATTACHED);
    PowerAllocatorEvent event = {
        .event_type = PowerAllocatorEventType::PORT_ATTACHED,
        .port_id = port.Id(),
    };
    uint8_t retry_limit = 3;
    while (retry_limit--) {
      esp_err_t res = PowerAllocator::GetInstance().EnqueueEvent(event);
      if (res == pdPASS) {
        break;
      }
    }
  }
}

void PortAttachedState::Handle(Port &port) {
  if (!port.Attached()) {
    ESP_LOGI(TAG, "Port %d is detached", port.Id());
    port.Reset();
    port.SetState(PortStateType::ACTIVE);
    PowerAllocatorEvent event = {
        .event_type = PowerAllocatorEventType::PORT_DETACHED,
        .port_id = port.Id(),
    };
    PowerAllocator::GetInstance().EnqueueEvent(event);
  }
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
  ESP_GOTO_ON_ERROR(
      rpc::mcu::keep_alive(port.Id(), &status), BRINGUP, TAG,
      "Port %d isn't responding to keep-alive, trying to bring it up again",
      port.Id());
  ESP_GOTO_ON_FALSE(status == KeepAliveStatus::USER_APPLICATION_ALIVE,
                    ESP_ERR_INVALID_STATE, BRINGUP, TAG,
                    "Port %d isn't in user application, trying to "
                    "bring it up again",
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
                    "Unable to hard reset port %d", port.Id());
  ret = rpc::mcu::bringup(port.Id(), true);
  if (ret == ESP_OK) {
    goto RECOVERED;
  }
#else
#endif
DEAD:
  ESP_ERROR_COMPLAIN(ret, "Unable to resurrect port %d", port.Id());
  port.SetState(PortStateType::DEAD);
}

void PortCheckingState::Entering(Port &port) {
  StopCheckingTimer();
  enter_time_ = esp_timer_get_time();
  StartCheckingTimer(port);
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
  StopCheckingTimer();
  enter_time_ = -1;
}

void PortCheckingState::StartCheckingTimer(Port &port) {
  if (checking_timer_ != nullptr) {
    ESP_LOGW(TAG, "Checking timer already started for port %d", port.Id());
    return;
  }
  checking_timer_ =
      xTimerCreate("checking_timer", pdMS_TO_TICKS(PORT_CHECKING_TIMER_MS),
                   pdFALSE, (void *)&port, CheckingTimerCallback);
  xTimerStart(checking_timer_, 0);
  ESP_LOGI(TAG, "Checking timer started for port %d", port.Id());
}

void PortCheckingState::StopCheckingTimer() {
  if (checking_timer_ != nullptr) {
    xTimerStop(checking_timer_, 0);
    xTimerDelete(checking_timer_, 0);
    checking_timer_ = nullptr;
    ESP_LOGI(TAG, "Checking timer stopped");
  }
}

void PortCheckingState::CheckingTimerCallback(TimerHandle_t timer) {
#ifdef CONFIG_MCU_MODEL_SW3566
  Port *port = (Port *)pvTimerGetTimerID(timer);
  // Send the keep-alive message to the port
  KeepAliveStatus status;
  uint32_t uptime_ms;
  uint32_t reboot_reason;
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "Port %d checking timer callback started.", port->Id());
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
