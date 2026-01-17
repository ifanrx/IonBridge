#include "port_manager.h"

#include <sys/param.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>

#include "acdc.h"
#include "data_types.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "internal/sw3566_command_types.h"
#include "port.h"
#include "portmacro.h"
#include "sdkconfig.h"
#include "utils.h"

#ifdef CONFIG_ENABLE_PROGRAMMER
#include "firmware_programmer.h"
#include "sw3566_data_types.h"
#endif


#ifdef CONFIG_INTERFACE_TYPE_UART
#include <cinttypes>

#include "esp_log_buffer.h"
#include "esp_log_level.h"
#include "uart.h"

#endif

#ifdef CONFIG_MCU_MODEL_FAKE_SW3566
#include "rpc.h"
#endif

#define TASK_STACK_SIZE CONFIG_PORT_MANAGER_TASK_STACK_SIZE
#define EVENT_QUEUE_SIZE CONFIG_PORT_MANAGER_EVENT_QUEUE_SIZE
#define EVENT_ENQUEUE_TIMEOUT \
  pdMS_TO_TICKS(CONFIG_PORT_MANAGER_EVENT_ENQUEUE_TIMEOUT_MS)
#define EVENT_DEQUEUE_TIMEOUT \
  pdMS_TO_TICKS(CONFIG_PORT_MANAGER_EVENT_DEQUEUE_TIMEOUT_MS)
#define UPGRADE_TASK_STACK_SIZE CONFIG_UPGRADE_TASK_STACK_SIZE
#define UPGRADE_TASK_PRIORITY CONFIG_UPGRADE_TASK_PRIORITY

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

void PortManager::Start() {
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
#ifndef CONFIG_MCU_MODEL_FAKE_SW3566
  PortManagerEvent event;
#endif
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
      for (Port& port : *this) {
        port.Update();
      }
    }
    DELAY_MS(1);  // Yield to other tasks
#else
    for (Port& port : *this) {
      if (!port.IsInitialized()) {
        continue;
      }
      PortDetails details;
      memset(&details, 0, sizeof(details));
      ClientPDStatus pd_status;
      memset(&pd_status, 0, sizeof(pd_status));
      rpc::mcu::get_port_details(port.Id(), &details);
      port.UpdateData(details);
      if (details.connected) {
        rpc::mcu::get_pd_status(port.Id(), &pd_status);
        port.SetPDStatus(pd_status);
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

void PortManager::Stop() {
  PortManagerEvent event;
  event.event_type = PortManagerEventType::TERMINATE_EVENT;
  xQueueSend(eventQueue_, &event, EVENT_ENQUEUE_TIMEOUT);
  for (Port& port : *this) {
    port.Shutdown();
  }
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

uint32_t PortManager::GetTotalOperatingPower() const {
  uint32_t total_power_uw = 0;
  this->ForEach([&total_power_uw](const Port& port) {
    if (!port.Attached()) {
      return;
    }

    ClientPDStatus pd_status = {};
    port.GetPDStatus(&pd_status);

    // PD negotiated power (voltage × current) takes priority
    if (pd_status.operating_voltage > 0 && pd_status.operating_current > 0) {
      uint32_t voltage_mv = pd_status.operating_voltage * 10;
      uint32_t current_ma = pd_status.operating_current * 10;
      total_power_uw += voltage_mv * current_ma;
    } else {
      // Fallback to measured power usage
      total_power_uw += static_cast<uint32_t>(port.GetPowerUsage()) * 1000000;
    }
  });
  return total_power_uw;
}

VoltageRequirements PortManager::GetVoltageRequirements() const {
  VoltageRequirements req = {-1, false};

  this->ForEach([&req](const Port& port) {
    if (!port.Attached()) return;

    PortDetails details = {};
    port.GetDetails(&details);

    // FC_NOT_CHARGING: truly idle, allow step-down
    if (details.fc_protocol == FC_NOT_CHARGING) return;

    // Has charging port - can't stay at -1
    if (req.max_output_voltage_mv < 0) {
      req.max_output_voltage_mv = 0;
    }

    int16_t voltage_mv = 0;

    // Non-PD protocols (including FC_None): use measured voltage
    if (details.fc_protocol < FC_PD_Fix5V) {
      voltage_mv = details.vout_value;
      if (voltage_mv > req.max_output_voltage_mv) {
        req.max_output_voltage_mv = voltage_mv;
      }
      return;
    }

    // PD protocols: prefer operating_voltage
    ClientPDStatus pd_status = {};
    port.GetPDStatus(&pd_status);
    voltage_mv = pd_status.operating_voltage * 10;

    // Fallback to deterministic defaults if chip didn't report
    if (voltage_mv == 0) {
      voltage_mv = (details.fc_protocol == FC_PD_Fix5V) ? 5000 : 20000;
    }

    if (voltage_mv > req.max_output_voltage_mv) {
      req.max_output_voltage_mv = voltage_mv;
    }
    req.has_pps_avs = req.has_pps_avs || pd_status.request_ppsavs;
  });

  return req;
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
  if (port_id == GPU_ADDR) {
    ESP_LOGD(TAG, "Ignoring GPU address UART message");
    return;
  }

  ENSURE_PORT_ID(port_id);
  Port* port = ports_[port_id].get();
  if (port == nullptr) {
    ESP_LOGE(TAG, "Invalid port ID: %d", port_id);
    return;
  }

#ifdef CONFIG_ENABLE_PROGRAMMER
  // Check programmer mode first
  FirmwareProgrammer& prog_mgr = FirmwareProgrammer::GetInstance();
  if (prog_mgr.IsActivated()) {
    // Forward ALL messages to FirmwareProgrammer
    prog_mgr.HandleUARTMessage(uart_msg, port_id);
    return;
  }
#endif

  // Normal mode: handle all normal commands
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

  // Detect abnormal session_charge values from MCU
  uint64_t session_charge = response.details.session_charge;
  if (session_charge > NEAR_OVERFLOW_THRESHOLD) {
    ESP_LOGW(TAG,
             "[Port %d] session_charge near overflow from MCU: 0x%" PRIx32
             "%08" PRIx32 " µWs",
             port.Id(), (uint32_t)(session_charge >> 32),
             (uint32_t)session_charge);
  } else if (session_charge > MAX_REASONABLE_SESSION_CHARGE) {
    ESP_LOGW(TAG,
             "[Port %d] Abnormal session_charge from MCU: 0x%" PRIx32
             "%08" PRIx32 " µWs",
             port.Id(), (uint32_t)(session_charge >> 32),
             (uint32_t)session_charge);
  }

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
  ESP_LOGW(TAG,
           "Invalid command error from port %d, raw command: 0x%04x, raw "
           "length: %d",
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

  if (alert.gpio_toggled) {
    // port shutoff by GPIO, entering checking state
    port.Checking();
    return;
  }

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
  if (!port.Attached()) {
    attaced_ports_ &= ~(1 << port.Id());
    return;
  }

  bool is_new_attach = !(attaced_ports_ & (1 << port.Id()));
  charging_at_ = std::min(port.AttachedAtMS(), charging_at_);
  attaced_ports_ |= (1 << port.Id());
  if (is_new_attach) {
    request_highest_voltage();
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
