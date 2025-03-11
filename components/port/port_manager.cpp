#include "port_manager.h"

#include <sys/param.h>

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cstdint>
#include <functional>
#include <memory>
#include <numeric>

#include "data_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "port.h"
#include "port_data.h"
#include "portmacro.h"
#include "sdkconfig.h"

#ifdef CONFIG_MCU_MODEL_SW3566
#include "sw3566_data_types.h"
#endif

#ifdef CONFIG_INTERFACE_TYPE_UART
#include "uart.h"
#endif

#ifdef CONFIG_MCU_MODEL_FAKE_SW3566
#include "rpc.h"
#include "utils.h"
#endif

#define TASK_STACK_SIZE CONFIG_PORT_MANAGER_TASK_STACK_SIZE
#define EVENT_QUEUE_SIZE CONFIG_PORT_MANAGER_EVENT_QUEUE_SIZE
#define EVENT_ENQUEUE_TIMEOUT \
  pdMS_TO_TICKS(CONFIG_PORT_MANAGER_EVENT_ENQUEUE_TIMEOUT_MS)
#define EVENT_DEQUEUE_TIMEOUT \
  pdMS_TO_TICKS(CONFIG_PORT_MANAGER_EVENT_DEQUEUE_TIMEOUT_MS)

#define ENSURE_PORT_ID(port_id, ...)                   \
  do {                                                 \
    if ((port_id) >= NUM_PORTS) {                      \
      ESP_LOGE(TAG, "Invalid port ID: %d", (port_id)); \
      return __VA_ARGS__;                              \
    }                                                  \
  } while (0)
#define SET_BITS(original, mask, value) \
  ((original) = ((original) & ~(mask)) | ((value) & (mask)))

static const char* TAG = "PortManager";

PortManager::PortManager() : dummy_stats_data_(PORT_HISTORICAL_DATA_SIZE) {
  // Initialize ports_
  for (int i = 0; i < NUM_PORTS; i++) {
    ports_[i] = std::make_unique<Port>(i);
  }

  // Create unified event queue
  eventQueue_ = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(PortManagerEvent));
  if (eventQueue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create event queue");
  }
}

PortManager::~PortManager() {
  // Ports will be automatically deleted since we're using unique_ptr
  if (eventQueue_ != nullptr) {
    vQueueDelete(eventQueue_);
  }
}

void PortManager::StartTask() {
  if (taskHandle_ != nullptr) {
    return;
  }
  taskRunning_ = false;
  BaseType_t res = xTaskCreate(&PortManager::TaskLoopWrapper, "port_manager",
                               TASK_STACK_SIZE, this,
                               CONFIG_UART_EVENT_TASK_PRIORITY, &taskHandle_);
  if (res != pdPASS) {
    ESP_LOGE(TAG, "Failed to create PortManager task");
    return;
  }
  taskRunning_ = true;
}

void PortManager::TaskLoopWrapper(void* pvParameters) {
  static_cast<PortManager*>(pvParameters)->TaskLoop();
}

void PortManager::TaskLoop() {
  ESP_LOGI(TAG, "Starting PortManager");
  PortManagerEvent event;
  while (eventQueue_ != nullptr) {
#ifndef CONFIG_MCU_MODEL_FAKE_SW3566
    // Wait indefinitely for any events to be posted to the queue
    if (xQueueReceive(eventQueue_, &event, EVENT_DEQUEUE_TIMEOUT) == pdPASS) {
      ESP_LOGD(TAG, "Received event: %d", static_cast<int>(event.event_type));
      switch (event.event_type) {
#ifdef CONFIG_INTERFACE_TYPE_UART
        case PortManagerEventType::UART_MESSAGE: {
          this->HandleUARTMessage(event);
        } break;
#endif
        case PortManagerEventType::TERMINATE_EVENT: {
          ESP_LOGI(TAG, "Terminating PortManager");
          vTaskDelete(nullptr);
          return;
        } break;
      }
    } else {
      // timeout, update alive ports
      this->UpdateAlivePortsStage();
    }
#else
    for (Port& port : *this) {
      if (!port.IsInitialized()) {
        continue;
      }
      PortDetails details;
      ClientPDStatus pd_status;
      rpc::mcu::get_port_details(port.Id(), &details);
      if (details.connected) {
        port.UpdateData(details);
        rpc::mcu::get_pd_status(port.Id(), &pd_status);
        port.SetPDStatus(pd_status);
      } else {
        port.Reset();
      }
      port.Update();
    }
    DELAY_MS(100);
#endif
  }

  // Should never reach here
  ESP_LOGE(TAG, "PortManager event queue is null");
  vTaskDelete(nullptr);
}

bool PortManager::InitializePort(uint8_t port_id, bool active,
                                 uint8_t initial_power_budget) {
  ENSURE_PORT_ID(port_id, false);
  return ports_[port_id]->Initialize(active, initial_power_budget);
}

Port* PortManager::GetPort(uint8_t port_id) {
  ENSURE_PORT_ID(port_id, nullptr);
  return ports_[port_id].get();
}

esp_err_t PortManager::GetPortData(uint8_t port_id, uint8_t* fc_protocol,
                                   uint8_t* temperature, uint16_t* current,
                                   uint16_t* voltage) const {
  ENSURE_PORT_ID(port_id, ESP_ERR_INVALID_ARG);
  bool attached = ports_[port_id]->Attached();
  const PortPowerData& data = ports_[port_id]->GetData();
  if (fc_protocol != nullptr) {
    *fc_protocol =
        attached ? data.GetFCProtocol() : static_cast<uint8_t>(FC_NOT_CHARGING);
  }
  if (temperature != nullptr) {
    *temperature = attached ? data.GetTemperature() : 0;
  }
  if (current != nullptr) {
    *current = attached ? data.GetCurrent() : 0;
  }
  if (voltage != nullptr) {
    *voltage = attached ? data.GetVoltage() : 0;
  }
  return ESP_OK;
}

uint32_t PortManager::GetPortChargingDurationSeconds(uint8_t port_id) const {
  ENSURE_PORT_ID(port_id, 0);
  return ports_[port_id]->GetChargingDurationSeconds();
}

esp_err_t PortManager::GetPortPDStatus(uint8_t port_id,
                                       ClientPDStatus* pd_status) const {
  ENSURE_PORT_ID(port_id, ESP_ERR_INVALID_ARG);
  if (pd_status != nullptr) {
    ports_[port_id]->GetPDStatus(pd_status);
  }
  return ESP_OK;
}

esp_err_t PortManager::GetPortPowerFeatures(uint8_t port_id,
                                            PowerFeatures* features) const {
  ENSURE_PORT_ID(port_id, ESP_ERR_INVALID_ARG);
  if (features != nullptr) {
    ports_[port_id]->GetPowerFeatures(features);
  }
  return ESP_OK;
}

esp_err_t PortManager::SetPortPowerFeatures(uint8_t port_id,
                                            const PowerFeatures& features) {
  ENSURE_PORT_ID(port_id, ESP_ERR_INVALID_ARG);
  ports_[port_id]->UpdatePowerFeatures(features);
  return ESP_OK;
}

esp_err_t PortManager::SetPortConfig(uint8_t port_id,
                                     const PortConfig& config) {
  ENSURE_PORT_ID(port_id, ESP_ERR_INVALID_ARG);
  Port* port = ports_[port_id].get();
  PowerFeatures features;
  port->GetPowerFeatures(&features);

  // Directly assign the first byte
  ((uint8_t*)&features)[0] = ((uint8_t*)&config.features)[0];
  // Assign the first 6 bits of the second byte
  SET_BITS(((uint8_t*)&features)[1], 0x3F, ((uint8_t*)&config.features)[1]);
  // Assign the first 1 bits of the third byte
  SET_BITS(((uint8_t*)&features)[2], 0x01, ((uint8_t*)&config.features)[2]);

  port->UpdatePowerFeatures(features);
  return port->Reconnect();
}

uint8_t PortManager::GetPortsPowerUsage() const {
  return std::accumulate(ports_.begin(), ports_.end(), 0,
                         [](uint8_t sum, const std::unique_ptr<Port>& port) {
                           return sum + port->GetPowerUsage();
                         });
}

uint8_t PortManager::GetPortsStatus(
    const std::function<bool(const std::unique_ptr<Port>&)>& predicate) const {
  uint8_t status = 0;
  for (uint8_t i = 0; i < NUM_PORTS; i++) {
    if (predicate(ports_[i])) {
      status |= (1 << i);
    }
  }
  return status;
}

uint8_t PortManager::GetPortsOpenStatus() const {
  return GetPortsStatus(
      [](const std::unique_ptr<Port>& port) { return port->IsOpen(); });
}

uint8_t PortManager::GetPortsAttachedStatus() const {
  return GetPortsStatus(
      [](const std::unique_ptr<Port>& port) { return port->Attached(); });
}

uint8_t PortManager::GetActivePortCount() const {
  return static_cast<uint8_t>(std::count_if(
      ports_.begin(), ports_.end(),
      [](const std::unique_ptr<Port>& port) { return !port->Dead(); }));
}

uint8_t PortManager::GetAttachedAndAdjustablePortCount() const {
  return static_cast<uint8_t>(std::count_if(
      ports_.begin(), ports_.end(), [](const std::unique_ptr<Port>& port) {
        return port->Attached() && port->IsAdjustable();
      }));
}

uint32_t PortManager::GetChargingDurationSeconds() const {
  if (charging_at_ == -1) {
    return 0;
  }
  uint32_t now = esp_timer_get_time() / 1e3;  // to milliseconds
  if (now < charging_at_) {
    return 0;
  }
  return (now - charging_at_) / 1e3;  // to seconds
}

std::array<uint8_t, NUM_PORTS> PortManager::GetPortsMinPower() const {
  std::array<uint8_t, NUM_PORTS> min_power;
  for (const Port& port : *this) {
    min_power[port.Id()] = port.MinPowerCap();
  }
  return min_power;
}

std::array<uint8_t, NUM_PORTS> PortManager::GetPortsMaxPower() const {
  std::array<uint8_t, NUM_PORTS> max_power;
  for (const Port& port : *this) {
    max_power[port.Id()] = port.MaxPowerCap();
  }
  return max_power;
}

std::array<uint8_t, NUM_PORTS> PortManager::GetAllPortsPowerUsage() {
  std::array<uint8_t, NUM_PORTS> power_usage;
  for (Port& port : *this) {
    power_usage[port.Id()] = port.GetPowerUsage();
  }
  return power_usage;
}

#ifdef CONFIG_INTERFACE_TYPE_UART
void PortManager::EnqueueUARTMessage(const uart_message_t& uart_msg,
                                     uint8_t port_id) {
  if (!taskRunning_) {
    return;
  }
  PortManagerEvent event;
  event.event_type = PortManagerEventType::UART_MESSAGE;
  event.uart_msg = uart_msg;
  ESP_LOG_BUFFER_HEXDUMP(TAG, &event.uart_msg, event.uart_msg.length + 10,
                         ESP_LOG_DEBUG);
  event.port_id = port_id;
  BaseType_t res = xQueueSend(eventQueue_, &event, EVENT_ENQUEUE_TIMEOUT);
  if (res != pdPASS) {
    ESP_LOGE(TAG,
             "Failed to enqueue UART message: 0x%04x from port %d, res: %d",
             uart_msg.command, port_id, res);
  }
}

void PortManager::HandleUARTMessage(const PortManagerEvent& event) {
  uart_message_t uart_msg = event.uart_msg;
  ESP_LOG_BUFFER_HEXDUMP(TAG, &uart_msg, uart_msg.length + 10, ESP_LOG_DEBUG);
  uint8_t port_id = event.port_id;
  ENSURE_PORT_ID(port_id);
  Port* port = ports_[port_id].get();
  if (port == nullptr) {
    ESP_LOGE(TAG, "Invalid port ID: %d", port_id);
    return;
  }
  switch (uart_msg.command) {
    case SW3566Command::GET_PORT_DETAILS: {
      HandlePortDetailsMessage(*port, uart_msg);
    } break;
    case SW3566Command::GET_PD_STATUS: {
      HandlePDStatusMessage(*port, uart_msg);
    } break;
    case SW3566Command::INVALID_COMMAND: {
      HandleInvalidCommandError(*port, uart_msg);
    } break;
    case SW3566Command::UNCORRECTABLE: {
      HandleUncorrectableError(*port, uart_msg);
    } break;
    case SW3566Command::CHARGING_ALERT: {
      HandleChargingAlert(*port, uart_msg);
    } break;
    case SW3566Command::KEEP_ALIVE: {
      HandleKeepAliveMessage(*port, uart_msg);
    } break;
    default: {
      ESP_LOGW(TAG, "Unhandled UART command: 0x%04x from port %d",
               uart_msg.command, port_id);
      ESP_LOG_BUFFER_HEXDUMP(TAG, uart_msg.payload, uart_msg.length,
                             ESP_LOG_WARN);
    } break;
  }
  port->Update();
}

void PortManager::HandlePortDetailsMessage(Port& port,
                                           const uart_message_t& uart_msg) {
  ESP_LOGD(TAG, "Received GET_PORT_DETAILS from port %d", port.Id());
  PortDetailsResponse response;
  size_t length = MIN(sizeof(PortDetailsResponse), uart_msg.length);
  memcpy(&response, uart_msg.payload, length);

  port.UpdateData(response.details);
  CheckAttachedPort(port);
}

void PortManager::HandlePDStatusMessage(Port& port,
                                        const uart_message_t& uart_msg) {
  PDStatusResponse response;
  size_t length = MIN(sizeof(PDStatusResponse), uart_msg.length);
  memcpy(&response, uart_msg.payload, length);
  if (response.status == GenericStatus::OK) {
    ESP_LOGD(TAG, "Received PD_STATUS from port %d", port.Id());
    ESP_LOG_BUFFER_HEXDUMP(TAG, uart_msg.payload, length - 1, ESP_LOG_DEBUG);
    port.SetPDStatus(response.data);
  }
}

void PortManager::HandleInvalidCommandError(Port& port,
                                            const uart_message_t& uart_msg) {
  InvalidCommandError error;
  size_t length = MIN(sizeof(InvalidCommandError), uart_msg.length);
  if (length == 0) {
    ESP_LOGE(TAG, "Invalid command error from port %d, empty payload",
             port.Id());
    return;
  }
  memcpy(&error, uart_msg.payload, length);
  ESP_LOGW(
      TAG,
      "Invalid command error from port %d, raw command: 0x%04x, raw length: %d",
      port.Id(), error.raw_command, error.raw_length);
  // u16 command + u8 length + u8 marker
  ESP_LOG_BUFFER_HEXDUMP(
      TAG, error.raw_payload,
      uart_msg.length > 4 ? uart_msg.length - 4 : uart_msg.length,
      ESP_LOG_WARN);
}

void PortManager::HandleUncorrectableError(Port& port,
                                           const uart_message_t& uart_msg) {
  ESP_LOGE(TAG, "Uncorrectable error from port %d", port.Id());
  UncorrectableError error;
  size_t length = MIN(sizeof(UncorrectableError), uart_msg.length);
  memcpy(&error, uart_msg.payload, length);
  ESP_LOGE(TAG,
           "head: %d, tail: %d, overrun: %d, overrun_flag: %d, "
           "uncorrectable_flag: %d, overrun_count: %d",
           error.header.head, error.header.tail, error.header.overrun,
           error.header.overrun_flag, error.header.uncorrectable_flag,
           error.header.overrun_count);
  ESP_LOG_BUFFER_HEXDUMP(TAG, error.payload,
                         uart_msg.length - sizeof(UARTHeader), ESP_LOG_ERROR);
}

void PortManager::HandleChargingAlert(Port& port,
                                      const uart_message_t& uart_msg) {
  ChargingAlert alert;
  size_t length = MIN(sizeof(ChargingAlert), uart_msg.length);
  memcpy(&alert, uart_msg.payload, length);

  ESP_LOGI(TAG,
           "ChargingAlert[%d] hard_reset=%d, err=%d, cable_reset=%d, "
           "rebroadcast=%d, rebroadcast_reason=%d, usage=%d, output=%.2f, "
           "max_pwr=%d, watermark=%d, gpio=%d, rapid_recnt=%d",
           port.Id(), alert.pd_rx_hard_reset, alert.pd_rx_error,
           alert.pd_rx_cable_reset, alert.rebroadcast, alert.rebroadcast_reason,
           port.GetPowerUsage(),
           static_cast<float>(alert.power_output * 3 * 8) / 1000000,
           alert.max_power, alert.watermark, alert.gpio_toggled,
           alert.rapid_reconnect);

  pd_rx_hard_reset_count_ += alert.pd_rx_hard_reset;
  pd_rx_error_count_ += alert.pd_rx_error;
  pd_rx_cable_reset_count_ += alert.pd_rx_cable_reset;
}

void PortManager::HandleKeepAliveMessage(Port& port,
                                         const uart_message_t& uart_msg) {
  KeepAliveResponse resp;
  memcpy(&resp, uart_msg.payload, sizeof(KeepAliveResponse));
  ESP_LOGI(TAG,
           "KeepAlive from port %d, status: %d, uptime: %" PRIu32
           "ms, reboot reason: 0x%" PRIX32,
           port.Id(), resp.status, resp.uptimeMS, resp.rebootReason);
}
#endif

void PortManager::CheckAttachedPort(const Port& port) {
  if (port.Attached()) {
    charging_at_ = std::min(port.AttachedAtMS(), charging_at_);
    attaced_ports_ |= (1 << port.Id());
  } else {
    attaced_ports_ &= ~(1 << port.Id());
  }
  if (attaced_ports_ == 0) {
    ESP_LOGD(TAG, "No ports attached");
    charging_at_ = -1;
  }
}
