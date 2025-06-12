#include "power_allocator.h"

#include <cinttypes>
#include <cstdint>
#include <utility>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "ionbridge.h"
#include "machine_info.h"
#include "port_manager.h"
#include "portmacro.h"
#include "power_config.h"
#include "sdkconfig.h"
#include "strategy.h"
#include "utils.h"

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
static uint16_t kRemainingPower = 0;

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

  if (timer_handle_ == nullptr) {
    const esp_timer_create_args_t allocator_timer_args = {
        .callback = &PowerAllocator::timer_callback_,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "allocator_timer",
        .skip_unhandled_events = true,
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&allocator_timer_args, &timer_handle_),
                        TAG, "Failed to create timer");
    ESP_RETURN_ON_ERROR(
        esp_timer_start_periodic(timer_handle_, timer_period_ * 1e3), TAG,
        "Failed to start allocator timer");
  }

  BaseType_t res =
      xTaskCreate(&PowerAllocator::TaskLoopWrapper, "pwr_alloc", 2.5 * 1024,
                  this, CONFIG_POWER_ALLOCATOR_TASK_PRIORITY, &taskHandle_);
  if (res != pdPASS) {
    ESP_LOGE(TAG, "Unable to create power_allocator task");
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

uint16_t PowerAllocator::AddApplyCount() {
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
  configured_ = true;
}

void PowerAllocator::TaskLoopWrapper(void *pvParameters) {
  PowerAllocator *allocator = static_cast<PowerAllocator *>(pvParameters);
  if (allocator != nullptr) {
    allocator->TaskLoop();
  }
}

void PowerAllocator::TaskLoop() {
  ESP_LOGI(TAG, "Starting power_allocator");
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
          PortAttached(event.port_id);
          break;

        case PowerAllocatorEventType::PORT_DETACHED:
          PortDetached(event.port_id);
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

void PowerAllocator::Stop() {
  if (timer_handle_ != nullptr) {
    esp_timer_stop(timer_handle_);
    esp_timer_delete(timer_handle_);
    DELAY_MS(100);
    timer_handle_ = nullptr;
  }
  taskRunning_ = false;
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
    ESP_LOGE(TAG, "Failed to enqueue power allocator event: %d", (int)res);
  }

  return res;
}

esp_err_t PowerAllocator::EnqueueEvent(PowerAllocatorEventType event_type,
                                       uint8_t port_id) {
  if (!taskRunning_) {
    ESP_LOGW(TAG,
             "Enqueuing event %d while power_allocator task is not running",
             static_cast<int>(event_type));
    return ESP_ERR_INVALID_STATE;
  }
  PowerAllocatorEvent event = {
      .event_type = event_type,
      .port_id = port_id,
  };
  return EnqueueEvent(event);
}

void PowerAllocator::timer_callback_(void *arg) {
  // Retrieve the Allocator instance from the timer ID
  PowerAllocator *allocator = static_cast<PowerAllocator *>(arg);

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
  bool switched = false;

  if (OverTempLimitDetect()) {
    return;
  }

  if (this->executing_) {
    ESP_LOGW(TAG, "Power allocator is already executing");
    return;
  }

  this->executing_ = true;
  executed++;

  switched = SwitchStrategy();
  if (power_allocation_enabled_ && switched) {
    ESP_LOGI(TAG, "Power strategy changed to %d, initializing power allocator",
             strategy_->Type());
    strategy_->SetupInitialPower(port_manager_);
    prev_ports_connected_status = port_manager_.GetAttachedPortsBitMask();
    goto RET;
  }

  if (power_allocation_enabled_) {
    HighTempDetect();
  }

  ports_connected_status = port_manager_.GetAttachedPortsBitMask();

  if (((executed % apply_period_) == 0 ||
       ports_connected_status != prev_ports_connected_status) &&
      strategy_ != nullptr) {
    if (power_allocation_enabled_) {
      ESP_LOGD(TAG, "Power strategy applied, execution count: %" PRIu32,
               executed);
      kRemainingPower = strategy_->Apply(port_manager_);
    }
  }

  prev_ports_connected_status = ports_connected_status;

  if (!power_allocation_enabled_ && executed % 600 == 0 &&
      esp_timer_get_time() < 10 * 60 * 1e6) {
    ESP_LOGI(TAG, "Power allocation is disabled");
  }

RET:
  this->executing_ = false;
}

void PowerAllocator::GetAllocationData(uint16_t *power_budget,
                                       uint16_t *remaining_power) const {
  if (strategy_ != nullptr) {
    *power_budget = strategy_->MaxPowerBudget();
    *remaining_power = kRemainingPower;
  } else {
    *power_budget = 0;
    *remaining_power = 0;
  }
}

void PowerAllocator::PortAttached(uint8_t port_id) {
  if (strategy_ != nullptr) {
    Execute();
  }
}

void PowerAllocator::PortDetached(uint8_t port_id) {
  if (strategy_ != nullptr) {
    Execute();
  }
}

void PowerAllocator::Reallocate() {
  if (strategy_ != nullptr) {
    Execute();
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
      case StrategyType::USBA_CHARGING:
        type_ = USBA_ALLOCATOR;
        break;
      default:
        ESP_LOGW(TAG, "Unsupported strategy: %d", strategy_buffer_->Type());
        strategy_buffer_.reset();  // Automatically deletes the strategy
        goto RET;
        break;
    }
    strategy_->Teardown(port_manager_);
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

PowerAllocator &AllocatorBuilder::Build() {
  PortManager &port_manager = PortManager::GetInstance();

  for (uint8_t i = 0; i < port_count_; i++) {
    port_manager.InitializePort(i, ports_active_[i], strategy_->InitialPower());
  }

  PowerAllocator &allocator = PowerAllocator::GetInstance();
  allocator.Configure(strategy_, cooldown_period_secs_, apply_period_,
                      timer_period_);

  ESP_LOGD(TAG, "Power allocator configured");
  return allocator;
}
