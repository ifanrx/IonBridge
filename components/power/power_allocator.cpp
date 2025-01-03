#include "power_allocator.h"

#include <algorithm>
#include <cinttypes>
#include <cstdint>
#include <utility>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "machine_info.h"
#include "mqtt_message.h"
#include "port.h"
#include "port_manager.h"
#include "portmacro.h"
#include "power_config.h"
#include "sdkconfig.h"
#include "strategy.h"
#include "telemetry_task.h"

#ifdef CONFIG_MCU_MODEL_SW3566
#include "ionbridge.h"
#include "rpc.h"
#endif

#define DEVICE_LOW_TEMP_THRESHOLD (power_config.device_low_temp_threshold)
#define DEVICE_OVER_TEMP_DECREMENT (power_config.device_over_temp_decrement)
#define DEVICE_UNDER_TEMP_INCREMENT (power_config.device_under_temp_increment)
#define DEVICE_POWER_COOLDOWN_TIME \
  (power_config.device_power_cooldown_time * 1e6)

#define DEVICE_HIGH_TEMP_THRESHOLD (power_config.device_high_temp_threshold)
#define DEVICE_HIGH_TEMP_MIN_POWER_LIMIT \
  (power_config.device_high_temp_min_power)
#define DEVICE_HIGHEST_TEMP_THRESHOLD CONFIG_DEVICE_HIGHEST_TEMP_THRESHOLD
#define DEVICE_HIGHEST_TEMP_MIN_POWER_LIMIT \
  CONFIG_DEVICE_HIGHEST_TEMP_MIN_POWER_LIMIT
#define DEVICE_DEAD_TEMP_THRESHOLD CONFIG_DEVICE_DEAD_TEMP_THRESHOLD
#define DEVICE_DEAD_TEMP_THRESHOLD_IF_POWER_ALLOCATION_DISABLED \
  CONFIG_DEVICE_DEAD_TEMP_THRESHOLD_IF_POWER_ALLOCATION_DISABLED
#define DEVICE_RECOVER_TEMP_THRESHOLD CONFIG_DEVICE_RECOVER_TEMP_THRESHOLD
#define EVENT_QUEUE_SIZE CONFIG_POWER_ALLOCATOR_EVENT_QUEUE_SIZE
#define EVENT_ENQUEUE_TIMEOUT \
  pdMS_TO_TICKS(CONFIG_PORT_MANAGER_EVENT_ENQUEUE_TIMEOUT_MS)
#define FAST_CHARGING_STRATEGY_REALLOCATION_APPLY_COUNT \
  CONFIG_FAST_CHARGING_STRATEGY_REALLOCATION_APPLY_COUNT

static const char *TAG = "PowerAllocator";
static uint8_t remaining_power = 0;

PowerAllocator &PowerAllocator::GetInstance() {
  static PowerAllocator instance;
  return instance;
}

PowerAllocator::PowerAllocator()
    : port_manager_(PortManager::GetInstance()),
      strategy_(nullptr),
      cooldown_period_secs_(0),
      apply_period_(0),
      timer_handle_(nullptr),
      timer_period_(0),
      temperature_mode_(TemperatureMode::POWER_PRIORITY) {}

esp_err_t PowerAllocator::Start() {
  if (taskRunning_) {
    ESP_LOGW(TAG, "PowerAllocator is already running");
    return ESP_ERR_INVALID_STATE;
  }

  eventQueue_ = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(PowerAllocatorEvent));
  if (eventQueue_ == nullptr) {
    ESP_LOGE(TAG, "Unable to create event queue");
    return ESP_ERR_NO_MEM;
  }

  mqttQueue_ = xQueueCreate(1, sizeof(MQTTMessage));
  if (mqttQueue_ == nullptr) {
    ESP_LOGE(TAG, "Unable to create MQTT message queue");
    return ESP_ERR_NO_MEM;
  }

  if (timer_handle_ == nullptr) {
    timer_handle_ =
        xTimerCreate("allocator_timer", pdMS_TO_TICKS(timer_period_), pdTRUE,
                     static_cast<void *>(this), timer_callback_);
    if (timer_handle_ != nullptr) {
      xTimerStart(timer_handle_, 0);  // Start the timer with no delay
    } else {
      ESP_LOGE(TAG, "Unable to create timer");
      return ESP_ERR_NO_MEM;
    }
  }

  BaseType_t res =
      xTaskCreate(&PowerAllocator::TaskLoopWrapper, "pa", 1024 * 2, this,
                  CONFIG_UART_EVENT_TASK_PRIORITY, &taskHandle_);
  if (res != pdPASS) {
    ESP_LOGE(TAG, "Unable to create power_allocator task");
    return ESP_ERR_NO_MEM;
  }

  res = xTaskCreate(&PowerAllocator::MqttTaskLoopWrapper, "pa_mqtt", 1024 * 2,
                    this, CONFIG_UART_EVENT_TASK_PRIORITY, &mqttTaskHandle_);
  if (res != pdPASS) {
    ESP_LOGE(TAG, "Unable to create power_allocator_mqtt task");
    return ESP_ERR_NO_MEM;
  }
  return ESP_OK;
}

uint8_t PowerAllocator::AddApplyCount() {
  apply_count_++;
  if (apply_count_ >
      FAST_CHARGING_STRATEGY_REALLOCATION_APPLY_COUNT * apply_period_) {
    apply_count_ = 0;
  }
  return apply_count_;
}

void PowerAllocator::Configure(PowerStrategy *strategy,
                               uint32_t cooldown_period_secs,
                               uint32_t apply_period, uint32_t timer_period,
                               TemperatureMode temperature_mode) {
  strategy_ = std::unique_ptr<PowerStrategy>(strategy);
  cooldown_period_secs_ = cooldown_period_secs;
  apply_period_ = apply_period;
  timer_period_ = timer_period;
  temperature_mode_ = temperature_mode;
  if (strategy_->Type() == StrategyType::SLOW_CHARGING) {
    type_ = FLEXAI_SLOW_ALLOCATOR;
  } else {
    type_ = FLEXAI_FAST_ALLOCATOR;
  }
}

void PowerAllocator::TaskLoopWrapper(void *pvParameters) {
  PowerAllocator *allocator = static_cast<PowerAllocator *>(pvParameters);
  if (allocator != nullptr) {
    allocator->TaskLoop();
  }
}

void PowerAllocator::MqttTaskLoopWrapper(void *pvParameters) {
  PowerAllocator *allocator = static_cast<PowerAllocator *>(pvParameters);
  if (allocator != nullptr) {
    allocator->MqttTaskLoop();
  }
}

void PowerAllocator::TaskLoop() {
  ESP_LOGI(TAG, "power_allocator task is starting");
  taskRunning_ = true;
  PowerAllocatorEvent event;

  while (taskRunning_ && eventQueue_ != nullptr) {
    if (xQueueReceive(eventQueue_, &event, portMAX_DELAY) == pdPASS) {
      ESP_LOGD(TAG, "Received event: %d", static_cast<int>(event.event_type));

      switch (event.event_type) {
        case PowerAllocatorEventType::EXECUTE:
          Execute();
          break;

        case PowerAllocatorEventType::PORT_ATTACHED:
          PortAttached();
          break;

        case PowerAllocatorEventType::PORT_DETACHED:
          PortDetached();
          break;

        case PowerAllocatorEventType::REALLOCATE:
          Reallocate();
          break;

        default:
          ESP_LOGW(TAG, "power_allocator: Unknown event type received: %d",
                   static_cast<int>(event.event_type));
          break;
      }
    }
  }

  ESP_LOGW(TAG, "power_allocator task is terminating");
  if (eventQueue_ != nullptr) {
    xQueueReset(eventQueue_);
    vQueueDelete(eventQueue_);
    eventQueue_ = nullptr;
  }
  vTaskDelete(nullptr);
}

void PowerAllocator::MqttTaskLoop() {
  ESP_LOGI(TAG, "power_allocator_mqtt task is starting");
  mqttTaskRunning_ = true;
  PowerAllocatorMqttEvent event;

  while (mqttTaskRunning_ && mqttQueue_ != nullptr) {
    if (xQueueReceive(mqttQueue_, &event, portMAX_DELAY) == pdPASS) {
      ESP_LOGD(TAG, "Received event: %d", static_cast<int>(event.event_type));
      switch (event.event_type) {
        case PowerAllocatorMqttEventType::REPORT_STATS:
          ReportPowerStats();
          break;

        default:
          ESP_LOGW(TAG, "power_allocator_mqtt: Unknown event type received: %d",
                   static_cast<int>(event.event_type));
          break;
      }
    }
  }

  ESP_LOGW(TAG, "power_allocator_mqtt task is terminating");
  if (mqttQueue_ != nullptr) {
    xQueueReset(mqttQueue_);
    vQueueDelete(mqttQueue_);
    mqttQueue_ = nullptr;
  }
  vTaskDelete(nullptr);
}

void PowerAllocator::Stop() {
  if (timer_handle_ != nullptr) {
    xTimerStop(timer_handle_, 0);    // Stop the timer
    xTimerDelete(timer_handle_, 0);  // Delete the timer
    timer_handle_ = nullptr;
  }
  taskRunning_ = false;
  mqttTaskRunning_ = false;
}

esp_err_t PowerAllocator::EnqueueEvent(PowerAllocatorEvent event) {
  // Check if the event queue is initialized
  if (eventQueue_ == nullptr) {
    ESP_LOGE(TAG, "Event queue is not initialized");
    return errQUEUE_FULL;
  }

  BaseType_t res;

  // Prioritize port attach/detach events by sending them to the front of the
  // queue
  if (event.event_type == PowerAllocatorEventType::PORT_ATTACHED ||
      event.event_type == PowerAllocatorEventType::PORT_DETACHED) {
    res = xQueueSendToFront(eventQueue_, &event, EVENT_ENQUEUE_TIMEOUT);

    if (res == errQUEUE_FULL) {
      // Remove the oldest event to make room for the new one
      PowerAllocatorEvent oldest_event;
      xQueueReceive(eventQueue_, &oldest_event, 0);
      ESP_LOGW(TAG,
               "Queue full: removed oldest event to enqueue new port event");
      res = xQueueSendToFront(eventQueue_, &event, EVENT_ENQUEUE_TIMEOUT);
    }
  } else {
    // Enqueue other events normally
    res = xQueueSend(eventQueue_, &event, EVENT_ENQUEUE_TIMEOUT);
  }

  // Log a warning if the queue remains full after attempts to enqueue
  if (res == errQUEUE_FULL) {
    ESP_LOGW(TAG, "Event queue is full; dropping event");
  } else if (res != pdPASS) {
    // Log an error if any other queue operation fails
    ESP_LOGE(TAG, "Failed to enqueue power allocator event: %d", res);
  }

  return res;
}

void PowerAllocator::EnqueueMqttEvent(PowerAllocatorMqttEvent event) {
  // If the queue is full, drop the event
  BaseType_t res = xQueueSend(mqttQueue_, &event, EVENT_ENQUEUE_TIMEOUT);

  if (res == errQUEUE_FULL) {
    // MQTT queue length is 1, so silently drop the event
    return;
  }

  if (res != pdPASS) {
    ESP_LOGE(TAG, "Failed to enqueue Power Allocator MQTT event: %d", res);
  }
}

void PowerAllocator::timer_callback_(TimerHandle_t timer) {
  // Retrieve the Allocator instance from the timer ID
  PowerAllocator *allocator =
      static_cast<PowerAllocator *>(pvTimerGetTimerID(timer));

  // feed the event queue
  PowerAllocatorEvent event = {
      .event_type = PowerAllocatorEventType::EXECUTE,
      .port_id = 0xff,
  };
  allocator->EnqueueEvent(event);
  if (allocator->AddApplyCount() ==
      FAST_CHARGING_STRATEGY_REALLOCATION_APPLY_COUNT *
          allocator->apply_period_) {
    PowerAllocatorEvent event = {
        .event_type = PowerAllocatorEventType::REALLOCATE,
        .port_id = 0xff,
    };
    allocator->EnqueueEvent(event);
  }
}

void PowerAllocator::Execute() {
  static uint8_t prev_ports_connected_status = 0;
  static uint32_t executed = 0;
  uint8_t ports_connected_status = 0;
  if (OverTempLimitDetect()) {
    return;
  }

  if (this->executing_) {
    ESP_LOGW(TAG, "Power allocator is already executing");
    return;
  }

  this->executing_ = true;
  executed++;
  bool switched = SwitchStrategy();
  if (power_allocation_enabled_ && switched) {
    ESP_LOGI(TAG, "Power strategy changed to %d, initializing power allocator",
             strategy_->Type());
    strategy_->SetupInitialPower(port_manager_);
    prev_ports_connected_status = port_manager_.GetPortsAttachedStatus();
    goto RET;
  }

  if (power_allocation_enabled_) {
    HighTempDetect();
  }

  ports_connected_status = port_manager_.GetPortsAttachedStatus();

  if (((executed % apply_period_) == 0 ||
       ports_connected_status != prev_ports_connected_status) &&
      strategy_ != nullptr) {
    if (power_allocation_enabled_) {
      ESP_LOGD(TAG, "Power strategy applied, execution count: %" PRIu32,
               executed);
      remaining_power = strategy_->Apply(port_manager_);
    }

    PowerAllocatorMqttEvent event = {
        .event_type = PowerAllocatorMqttEventType::REPORT_STATS,
    };
    EnqueueMqttEvent(event);
  }
  prev_ports_connected_status = ports_connected_status;

  if (!power_allocation_enabled_ && executed % 600 == 0 &&
      esp_timer_get_time() < 10 * 60 * 1e6) {
    ESP_LOGI(TAG, "Power allocation is disabled");
  }

RET:
  this->executing_ = false;
}

void PowerAllocator::PortAttached() {
  if (strategy_ != nullptr) {
    Execute();
  }
}

void PowerAllocator::PortDetached() {
  if (strategy_ != nullptr) {
    Execute();
  }
}

void PowerAllocator::Reallocate() {
  if (strategy_ != nullptr) {
    Execute();
  }
}

void PowerAllocator::ReportPowerStats() {
  TelemetryTask *task = TelemetryTask::GetInstance();
  uint8_t adc_value = 0;
#ifdef CONFIG_MCU_MODEL_SW3566
  ESP_ERROR_COMPLAIN(rpc::fpga::read_adc_value(&adc_value),
                     "fpga::read_adc_value");
#else
  // FIXME:
#endif

  if (task != nullptr && port_manager_.Size() <= 8) {
    PortPowerAllocation port_power_allocations[8] = {};
    std::transform(port_manager_.begin(), port_manager_.end(),
                   port_power_allocations,
                   [](const Port &port) -> PortPowerAllocation {
                     return PortPowerAllocation{
                         .source_cap = port.MaxPowerBudget(),
                         .usage = port.GetPowerUsage(),
                     };
                   });
    task->ReportPowerAllocationData(strategy_->MaxPowerBudget(), 0,
                                    remaining_power, adc_value,
                                    port_power_allocations);
  }
}

bool PowerAllocator::SwitchStrategy() {
  bool switched = false;
  if (strategy_buffer_) {
    switch (strategy_buffer_->Type()) {
      case StrategyType::SLOW_CHARGING:
        type_ = FLEXAI_SLOW_ALLOCATOR;
        break;
      case StrategyType::STATIC_CHARGING:
        type_ = STATIC_ALLOCATOR;
        break;
      case StrategyType::TEMPORARY_CHARGING:
        type_ = TEMPORARY_ALLOCATOR;
        break;
      default:
        ESP_LOGW(TAG, "Unsupported strategy type: %d",
                 strategy_buffer_->Type());
        strategy_buffer_.reset();  // Automatically deletes the strategy
        goto RET;
        break;
    }
    strategy_ = std::move(strategy_buffer_);
    switched = true;
  }

RET:
  return switched;
}

void PowerAllocator::ReducePowerBudget(uint8_t max_power, uint8_t decreasement,
                                       uint8_t min_limit,
                                       const char *scenario) {
  if (max_power - decreasement < min_limit) {
    decreasement = max_power - min_limit;
  }
  ESP_LOGW(TAG, "Decreasing power by %dW, current max power: %dW (%s scenario)",
           decreasement, max_power, scenario);
  strategy_->DecreasePowerBudget(decreasement);
}

void PowerAllocator::HighTempDetect() {
  // Monitor device temperature and adjust power allocation
}

esp_err_t PowerAllocator::SetTemperatureMode(TemperatureMode mode) {
  temperature_mode_ = mode;
  switch (mode) {
    case TemperatureMode::POWER_PRIORITY: {
      set_power_temperature_thresholds(CONFIG_DEVICE_HIGH_TEMP_THRESHOLD,
                                       CONFIG_DEVICE_LOW_TEMP_THRESHOLD);
      break;
    }
    case TemperatureMode::TEMPERATURE_PRIORITY: {
      set_power_temperature_thresholds(
          CONFIG_DEVICE_HIGH_TEMP_THRESHOLD_IN_TEMPERATURE_PRIORITY,
          CONFIG_DEVICE_LOW_TEMP_THRESHOLD_IN_TEMPERATURE_PRIORITY);
      break;
    }
  }
  return ESP_OK;
}

// Special case for PowerStrategy base class
template <>
const PowerStrategy *PowerAllocator::GetStrategyAs<PowerStrategy>() const {
  if (!strategy_) {
    return nullptr;
  }
  return strategy_.get();
}

bool PowerAllocator::OverTempLimitDetect() {
  static bool over_temp_limit = false;
  return over_temp_limit;
}

PowerAllocator *AllocatorBuilder::Build() {
  PortManager &port_manager = PortManager::GetInstance();
  PowerAllocator &allocator = PowerAllocator::GetInstance();

  for (uint8_t i = 0; i < port_count_; i++) {
    port_manager.InitializePort(i, ports_active_[i], strategy_->InitialPower());
  }

  allocator.Configure(strategy_, cooldown_period_secs_, apply_period_,
                      timer_period_);

  ESP_LOGD(TAG, "Power allocator configured");

  return &allocator;
}
