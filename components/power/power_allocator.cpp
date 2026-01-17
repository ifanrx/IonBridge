#include "power_allocator.h"

#include <cinttypes>
#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>

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
      apply_period_(0),
      timer_handle_(nullptr),
      timer_period_(0),
      temperature_mode_(TemperatureMode::POWER_PRIORITY) {
  }

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
    esp_err_t timer_err =
        esp_timer_create(&allocator_timer_args, &timer_handle_);
    if (timer_err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to create timer");
      vQueueDelete(eventQueue_);
      eventQueue_ = nullptr;
      return timer_err;
    }

    esp_err_t start_err =
        esp_timer_start_periodic(timer_handle_, timer_period_ * 1e3);
    if (start_err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to start allocator timer");
      esp_timer_delete(timer_handle_);
      timer_handle_ = nullptr;
      vQueueDelete(eventQueue_);
      eventQueue_ = nullptr;
      return start_err;
    }
  }

  BaseType_t res =
      xTaskCreate(&PowerAllocator::TaskLoopWrapper, "pwr_alloc", 2.5 * 1024,
                  this, CONFIG_POWER_ALLOCATOR_TASK_PRIORITY, &taskHandle_);
  if (res != pdPASS) {
    ESP_LOGE(TAG, "Unable to create power_allocator task");
    // Clean up timer and queue
    if (timer_handle_ != nullptr) {
      esp_timer_stop(timer_handle_);
      esp_timer_delete(timer_handle_);
      timer_handle_ = nullptr;
    }
    vQueueDelete(eventQueue_);
    eventQueue_ = nullptr;
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

void PowerAllocator::Configure(PowerStrategy *strategy, uint32_t apply_period,
                               uint32_t timer_period,
                               TemperatureMode temperature_mode) {
  strategy_ = std::unique_ptr<PowerStrategy>(strategy);
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
  taskRunning_ = true;  // Set running flag here in the task
  PowerAllocatorEvent event;

  while (taskRunning_ && eventQueue_ != nullptr) {
    if (xQueueReceive(eventQueue_, &event, portMAX_DELAY) == pdPASS) {
      ESP_LOGD(TAG, "Received event: %d", static_cast<int>(event.event_type));

      switch (event.event_type) {
        case PowerAllocatorEventType::EXECUTE:
          Execute();
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
  // Note: eventQueue_ will be cleaned up by the task itself in TaskLoop()
}

esp_err_t PowerAllocator::EnqueueEvent(PowerAllocatorEvent event) {
  // Check if the event queue is initialized
  if (eventQueue_ == nullptr) {
    ESP_LOGE(TAG, "Event queue is not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  BaseType_t res = xQueueSend(eventQueue_, &event, EVENT_ENQUEUE_TIMEOUT);

  // Log a warning if the queue remains full after attempts to enqueue
  if (res == errQUEUE_FULL) {
    ESP_LOGW(TAG, "Event queue is full; dropping event");
    return ESP_ERR_TIMEOUT;  // Convert to ESP error code
  } else if (res != pdPASS) {
    // Log an error if any other queue operation fails
    ESP_LOGE(TAG, "Failed to enqueue power allocator event: %d", (int)res);
    return ESP_FAIL;  // Generic failure
  }

  return ESP_OK;  // Success
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
  static uint32_t executed = 0;

  // Check if allocator has been started
  if (!IsStarted()) {
    return;
  }

  if (this->executing_) {
    ESP_LOGW(TAG, "Power allocator is already executing");
    return;
  }

  this->executing_ = true;
  executed++;

  if (!power_allocation_enabled_) {
    if (executed % 600 == 0 && esp_timer_get_time() < 10 * 60 * 1e6) {
      ESP_LOGI(TAG, "Power allocation is disabled");
    }
    goto RET;
  }

  if (SwitchStrategy() && strategy_ != nullptr) {
    ESP_LOGI(TAG, "Power strategy changed to %d, initializing power allocator",
             strategy_->Type());
    strategy_->Setup(port_manager_);
    goto RET;
  }

  HighTempDetect();

  if (strategy_ != nullptr &&
      strategy_->ShouldApply(port_manager_, (executed % apply_period_) == 0)) {
    ESP_LOGD(TAG, "Power strategy applied, execution count: %" PRIu32,
             executed);
    kRemainingPower = strategy_->Apply(port_manager_);
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

void PowerAllocator::GetChargingConfig(StrategyType &type, uint16_t *identifier,
                                       PowerAllocationTable *table) const {
  if (strategy_ == nullptr) {
    type = StrategyType::UNKNOWN_STRATEGY;
    return;
  }

  type = strategy_->Type();

  // Only populate additional fields for STATIC_CHARGING
  if (type == StrategyType::STATIC_CHARGING && strategy_ != nullptr) {
    auto static_strategy = GetStrategyAs<PowerStaticChargingStrategy>();
    if (static_strategy != nullptr) {
      if (identifier != nullptr) {
        *identifier = static_strategy->GetIdentifier();
      }
      if (table != nullptr) {
        const PowerAllocationTable &strategy_table =
            static_strategy->GetTable();
        std::memcpy(table, &strategy_table, sizeof(PowerAllocationTable));
      }
    }
  }
}

esp_err_t PowerAllocator::SetChargingConfig(StrategyType type,
                                            uint16_t identifier,
                                            const PowerAllocationTable *table) {
  // Don't set TEMPORARY_CHARGING via config sync
  if (type == StrategyType::TEMPORARY_CHARGING) {
    ESP_LOGW(TAG, "Cannot set TEMPORARY_CHARGING via config sync");
    return ESP_ERR_INVALID_ARG;
  }

  switch (type) {
    case StrategyType::SLOW_CHARGING:
      SetStrategy<PowerSlowChargingStrategy>();
      break;
    case StrategyType::STATIC_CHARGING:
      if (table != nullptr) {
        SetStrategy<PowerStaticChargingStrategy>(*table, identifier);
      } else {
        ESP_LOGE(TAG, "STATIC_CHARGING requires allocation table");
        return ESP_ERR_INVALID_ARG;
      }
      break;
    case StrategyType::LEGACY_CHARGING:
      SetStrategy<PowerSlowChargingStrategy>();
      break;
    case StrategyType::USBA_CHARGING:
      SetStrategy<PowerUSBAChargingStrategy>();
      break;
    case StrategyType::HIGH_PERFORMANCE_CHARGING:
      SetStrategy<PowerHighPerformanceStrategy>();
      break;
    default:
      ESP_LOGE(TAG, "Unknown strategy type: %d", static_cast<int>(type));
      return ESP_ERR_INVALID_ARG;
  }

  return ESP_OK;
}

void PowerAllocator::Reallocate() {
  if (strategy_ != nullptr) {
    Execute();
  }
}

bool PowerAllocator::SwitchStrategy() {
  if (!strategy_buffer_) {
    // No strategy buffer, nothing to switch
    return false;
  }
  if (strategy_ && strategy_buffer_->IsSameStrategy(*strategy_)) {
    // No need to switch, the same strategy is already in use
    strategy_buffer_.reset();
    return false;
  }

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
    case StrategyType::HIGH_PERFORMANCE_CHARGING:
      type_ = HIGH_PERFORMANCE_ALLOCATOR;
      break;
    case StrategyType::ULTRA_FAST_SINGLE_PORT_CHARGING:
      type_ = ULTRA_FAST_SINGLE_PORT_ALLOCATOR;
      break;
    default:
      ESP_LOGW(TAG, "Unsupported strategy: %d", strategy_buffer_->Type());
      strategy_buffer_.reset();  // Automatically deletes the strategy
      return false;
  }

  // Only call teardown if there's an existing strategy
  if (strategy_) {
    strategy_->Teardown(port_manager_);
  }
  strategy_ = std::move(strategy_buffer_);

  return true;
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
  // Temperature monitoring disabled - temperature component removed
}

esp_err_t PowerAllocator::SetTemperatureMode(TemperatureMode mode) {
  switch (mode) {
    case TemperatureMode::POWER_PRIORITY:
      temperature_mode_ = mode;
      set_power_temperature_thresholds(CONFIG_DEVICE_HIGH_TEMP_THRESHOLD,
                                       CONFIG_DEVICE_LOW_TEMP_THRESHOLD);
      break;
    case TemperatureMode::TEMPERATURE_PRIORITY:
      temperature_mode_ = mode;
      set_power_temperature_thresholds(
          CONFIG_DEVICE_HIGH_TEMP_THRESHOLD_IN_TEMPERATURE_PRIORITY,
          CONFIG_DEVICE_LOW_TEMP_THRESHOLD_IN_TEMPERATURE_PRIORITY);
      break;
    case TemperatureMode::MAX:
      return ESP_ERR_INVALID_ARG;
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

PowerAllocator &AllocatorBuilder::Build() {
  PortManager &port_manager = PortManager::GetInstance();

  for (uint8_t i = 0; i < port_count_; i++) {
    port_manager.InitializePort(i, ports_active_[i], strategy_->InitialPower());
  }

  PowerAllocator &allocator = PowerAllocator::GetInstance();
  allocator.Configure(strategy_, apply_period_, timer_period_);

  ESP_LOGD(TAG, "Power allocator configured");
  return allocator;
}
