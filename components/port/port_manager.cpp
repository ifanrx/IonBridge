#include "port_manager.h"

#include <sys/param.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "port.h"
#include "portmacro.h"
#include "sdkconfig.h"

#ifdef CONFIG_MCU_MODEL_SW3566
#include "sw3566_data_types.h"
#endif

#ifdef CONFIG_INTERFACE_TYPE_UART
#include <cinttypes>

#include "esp_log_buffer.h"
#include "esp_log_level.h"
#include "telemetry_task.h"
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

#define ENSURE_PORT_ID(port_id, ...)                                      \
  do {                                                                    \
    if ((port_id) >= NUM_PORTS) {                                         \
      ESP_LOGE(TAG, "%s(%d) Invalid port ID: %d", __FUNCTION__, __LINE__, \
               (port_id));                                                \
      return __VA_ARGS__;                                                 \
    }                                                                     \
  } while (0)

static const char* TAG = "PortManager";

PortManager::PortManager() {
  // Initialize ports_
  for (int i = 0; i < NUM_PORTS; i++) {
    ports_[i] = std::make_unique<Port>(i);
  }

  // Create unified event queue
  eventQueue_ = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(PortManagerEvent));
  if (eventQueue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create event queue");
  }

  // Create mutex
  mutex_ = xSemaphoreCreateMutex();
  if (mutex_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create mutex");
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
  BaseType_t res =
      xTaskCreate(&PortManager::TaskLoopWrapper, "port_mgr", TASK_STACK_SIZE,
                  this, CONFIG_PORT_MANAGER_TASK_PRIORITY, &taskHandle_);
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
      for (Port& port : this->GetAlivePorts()) {
        port.Update();
      }
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
  if (port_id >= NUM_PORTS) {
    return nullptr;
  }
  return ports_[port_id].get();
}

uint16_t PortManager::GetPortsPowerUsage() const {
  uint16_t usage = 0;
  this->ForEach([&usage](const Port& port) {
    if (!port.Dead()) {
      usage += port.GetPowerUsage();
    }
  });
  return usage;
}

uint8_t PortManager::GetPortsBitMask(PortPredicate predicate) const {
  uint8_t bitMask = 0;
  this->ForEach([&bitMask, &predicate](const Port& port) {
    if (predicate(port)) {
      bitMask |= (1 << port.Id());
    }
  });
  return bitMask;
}

uint8_t PortManager::GetOpenPortsBitMask() const {
  return GetPortsBitMask([](const Port& port) { return port.IsOpen(); });
}

uint8_t PortManager::GetAttachedPortsBitMask() const {
  return GetPortsBitMask([](const Port& port) { return port.Attached(); });
}

uint8_t PortManager::CountPorts(PortPredicate predicate) const {
  return __builtin_popcount(this->GetPortsBitMask(predicate));
}

uint8_t PortManager::GetAttachedPortCount() const {
  return CountPorts([](const Port& port) { return port.Attached(); });
}

uint8_t PortManager::GetActivePortCount() const {
  return CountPorts([](const Port& port) { return !port.Dead(); });
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
             uart_msg.command, port_id, (int)res);
  }
}

void PortManager::HandleUARTMessage(const PortManagerEvent& event) {
  uart_message_t uart_msg = event.uart_msg;
  ESP_LOGD(TAG, "Received UART message: 0x%04x from port %d", uart_msg.command,
           event.port_id);
  ESP_LOG_BUFFER_HEXDUMP(TAG, &uart_msg, uart_msg.length + 10, ESP_LOG_DEBUG);

  uint8_t port_id = event.port_id;
  if (port_id == BCAST_ADDR) {
    return HandleBroadcastMessage(uart_msg);
  }

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
    case SW3566Command::GET_PD_PCAP_DATA: {
      HandlePDPCapDataMessage(*port, uart_msg);
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

void PortManager::HandleBroadcastMessage(const uart_message_t& uart_msg) {
  ESP_LOGD(TAG, "Received broadcast message: 0x%04x", uart_msg.command);
  ESP_LOG_BUFFER_HEXDUMP(TAG, uart_msg.payload, uart_msg.length, ESP_LOG_DEBUG);
  uint8_t src = uart_msg.addr >> 4;
  switch (uart_msg.command) {
    case SW3566Command::INVALID_COMMAND: {
      ESP_LOGW(TAG, "Invalid command error from port %d", src);
      ESP_LOG_BUFFER_HEXDUMP(TAG, (const uint8_t*)&uart_msg,
                             UART_MESSAGE_LENGTH(uart_msg), ESP_LOG_WARN);
    } break;
    default: {
      ESP_LOGW(TAG, "Unhandled broadcast command: 0x%04x", uart_msg.command);
    } break;
  }
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
  ESP_LOGI(TAG, "Port[%d] requests voltage: %d", port.Id(),
           alert.vin_voltage_request);

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

#ifdef CONFIG_MCU_MODEL_SW3566
void PortManager::HandlePDPCapDataMessage(Port& port,
                                          const uart_message_t& uart_msg) {
  ESP_LOGD(TAG, "Received PD_PCAP_DATA from port %d", port.Id());

  constexpr size_t fixed_size = sizeof(PcapEntry);
  if (uart_msg.length < fixed_size) {
    ESP_LOGE(TAG, "Invalid PD_PCAP_DATA length: %d", uart_msg.length);
    return;
  }
  ESP_LOG_BUFFER_HEXDUMP(TAG, uart_msg.payload, uart_msg.length, ESP_LOG_DEBUG);

#ifndef CONFIG_IDF_TARGET_LINUX
  ESP_LOGD(TAG, "UART msg len: %d, header len: %d, payload len: %d",
           uart_msg.length, fixed_size, uart_msg.length - fixed_size);
#endif

  PcapEntry* entry = static_cast<PcapEntry*>(malloc(uart_msg.length));
  memcpy(entry, uart_msg.payload, uart_msg.length);
  ESP_LOGD(TAG,
           "PcapEntry - ts: %d, dir: %d, sop: %d, ser: %d, len: %d, vout: %d, "
           "iout: %d",
           entry->timestamp, entry->header.direction, entry->header.sop,
           entry->header.serial, entry->length, entry->vout, entry->iout);
  TelemetryTask& task = TelemetryTask::GetInstance();
  task.ReportPDPCapData(port.Id(), *entry);
  free(entry);
}
#endif
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

void PortManager::ForEach(ForEachCallback cb) {
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
    // Handle failure
    return;
  }

  for (Port& port : *this) {
    cb(port);
  }

  xSemaphoreGive(mutex_);
  return;
}

void PortManager::ForEach(ForEachConstCallback cb) const {
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
    // Handle failure
    return;
  }

  for (const Port& port : *this) {
    cb(port);
  }

  xSemaphoreGive(mutex_);
  return;
}
