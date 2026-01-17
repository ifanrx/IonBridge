#include "animation_manager.h"

#include "display_animation.h"
#include "display_manager.h"
#include "display_types.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "AnimationManager";

// Macro to generate lookup table entries with compile-time index checking
// Uses designated initializers (C99 feature) to ensure enum values match
// indices
#define ANIMATION_ENTRY(type, class) \
  [static_cast<uint8_t>(AnimationType::type)] = &class ::GetInstance()

// Animation instance lookup table (indexed by AnimationType enum value)
// Designated initializers ensure correct enum-to-instance mapping at compile
// time Each animation type maps to a unique singleton instance
static BaseAnimation *const kAnimationLookup[] = {
    ANIMATION_ENTRY(BOOT_APP, BootAppAnimation),
    ANIMATION_ENTRY(BLE_ADVERTISING, BleAdvertisingAnimation),
    ANIMATION_ENTRY(BLE_CONNECTED, BleConnectedAnimation),
    ANIMATION_ENTRY(START_OTA, StartOTAAnimation),
    ANIMATION_ENTRY(OTA_ANIMATION, OTAAnimation),
    ANIMATION_ENTRY(WIFI_CONNECTING, WifiConnectingAnimation),
    ANIMATION_ENTRY(IDLE, IdleAnimation),
    ANIMATION_ENTRY(DEVICE_INSERTED, DeviceInsertedAnimation),
    ANIMATION_ENTRY(DEVICE_REMOVED, DeviceRemovedAnimation),
    ANIMATION_ENTRY(SCREEN_OFF, ScreenOffAnimation),
    ANIMATION_ENTRY(DEVICE_INFO, DeviceInfoAnimation),
    ANIMATION_ENTRY(CHARGING, ChargingAnimation),
    ANIMATION_ENTRY(POWER_CHART, PowerChartAnimation),
    ANIMATION_ENTRY(DISPLAY_TEST, DisplayTestAnimation),
    ANIMATION_ENTRY(DATA_MATRIX, DataMatrixAnimation),
};
static constexpr size_t kAnimationCount =
    sizeof(kAnimationLookup) / sizeof(kAnimationLookup[0]);

AnimationManager &AnimationManager::GetInstance() {
  static AnimationManager instance;
  return instance;
}

AnimationManager::~AnimationManager() {
  // Ensure task is stopped before deleting synchronization primitives
  StopTask();

  if (mutex_ != nullptr) {
    vSemaphoreDelete(mutex_);
    mutex_ = nullptr;
  }
}

esp_err_t AnimationManager::StartTask() {
  animation_ = &NoAnimation::GetInstance();  // Safe default animation
  next_animation_type_ = AnimationType::NO_ANIMATIONS;
  next_animation_loop_ = false;

  mutex_ = xSemaphoreCreateMutex();
  if (mutex_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create mutex");
    return ESP_FAIL;
  }

  BaseType_t ret = xTaskCreate(TaskLoopWrapper, "animation", 3 * 1024, this,
                               tskIDLE_PRIORITY + 1, &task_handle_);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create display task");
    vSemaphoreDelete(mutex_);
    mutex_ = nullptr;
    return ESP_FAIL;
  }

  return ESP_OK;
}

void AnimationManager::SetNextAnimation(AnimationType animation_type,
                                        bool loop) {
  // Mutex must be valid - if null, AnimationManager not initialized
  if (mutex_ == nullptr) {
    ESP_LOGW(TAG, "SetNextAnimation called before StartTask()");
    return;
  }

  if (!running_) {
    return;  // Task stopped, ignore animation changes
  }

  // Product family check with fallback
  BaseAnimation *anim = GetAnimationInstance(animation_type);
  anim->SetDataSource(
      &data_source_);  // Ensure data source for dynamic fallback
  if (!anim->CanBePlayed()) {
    AnimationType fallback = anim->GetFallbackAnimation();

    if (fallback == AnimationType::NO_ANIMATIONS) {
      ESP_LOGD(TAG, "Animation %s rejected - no fallback",
               animation_type.toString());
      return;
    }
    BaseAnimation *fallback_anim = GetAnimationInstance(fallback);
    fallback_anim->SetDataSource(&data_source_);
    if (!fallback_anim->CanBePlayed()) {
      ESP_LOGD(TAG, "Fallback %s also rejected", fallback.toString());
      return;
    }
    ESP_LOGI(TAG, "Animation %s rejected, using fallback %s",
             animation_type.toString(), fallback.toString());
    animation_type = fallback;
  }

  // Pre-flight check: Reject non-critical animations when display is OFF
  if (DisplayManager::GetInstance().GetMode() == DisplayMode::OFF &&
      !IsCriticalAnimation(animation_type)) {
    ESP_LOGD(TAG, "Animation %s rejected - display mode is OFF",
             animation_type.toString());
    return;
  }

  // CanBeInterrupted check (skip for critical animations like OTA)
  if (!IsCriticalAnimation(animation_type) &&
      !animation_->CanBeInterrupted(animation_type)) {
    ESP_LOGD(TAG, "Animation %s blocked by %s", animation_type.toString(),
             animation_->Type().toString());
    return;
  }

  xSemaphoreTake(mutex_, portMAX_DELAY);
  next_animation_type_ = animation_type;
  next_animation_loop_ = loop;
  xSemaphoreGive(mutex_);

  NotifyUpdate();  // Wake up task immediately
}

void AnimationManager::StopAnimation(AnimationType animation_type) {
  // Mutex must be valid - if null, AnimationManager not initialized
  if (mutex_ == nullptr) {
    ESP_LOGW(TAG, "StopAnimation called before StartTask()");
    return;
  }

  if (!running_) {
    return;  // Task stopped, already stopped
  }

  xSemaphoreTake(mutex_, portMAX_DELAY);
  bool should_stop =
      (animation_ != nullptr && animation_->Type() == animation_type);
  xSemaphoreGive(mutex_);

  if (should_stop) {
    ESP_LOGI(TAG, "Stopping current animation: %s",
             AnimationType(animation_type).toString());
    SetNextAnimation(AnimationType::IDLE, true);
  }
}

void AnimationManager::NotifyUpdate() {
  if (task_handle_ != nullptr) {
    xTaskNotifyGive(task_handle_);
  }
}

void AnimationManager::SetOTAStage(OTAStage stage) {
  if (!running_) {
    return;  // Task stopped, ignore OTA stage changes
  }

  // Update the stage first (so it's ready when OTA_ANIMATION starts)
  OTAAnimation::GetInstance().SetStage(stage);

  // If not already in OTA mode, start START_OTA
  // START_OTA will auto-transition to OTA_ANIMATION after its duration
  AnimationType current = animation_->Type();
  if (current != AnimationType::START_OTA &&
      current != AnimationType::OTA_ANIMATION) {
    SetNextAnimation(AnimationType::START_OTA, true);
  }

  // Wake up task to render immediately
  NotifyUpdate();
}

void AnimationManager::TaskLoopWrapper(void *arg) {
  AnimationManager *aManager = static_cast<AnimationManager *>(arg);
  aManager->TaskLoop();
}

BaseAnimation *AnimationManager::GetAnimationInstance(
    AnimationType animation_type) {
  // Handle NO_ANIMATIONS: return safe NoAnimation instance
  if (animation_type == AnimationType::NO_ANIMATIONS) {
    return &NoAnimation::GetInstance();
  }

  // Use lookup table for O(1) access
  uint8_t index = static_cast<uint8_t>(animation_type);
  if (index >= kAnimationCount) {
    ESP_LOGW(TAG, "Invalid animation type: %d, falling back to NoAnimation",
             index);
    return &NoAnimation::GetInstance();
  }

  return kAnimationLookup[index];
}

bool AnimationManager::IsCriticalAnimation(AnimationType type) const {
  switch (type) {
    case AnimationType::START_OTA:
    case AnimationType::OTA_ANIMATION:
    case AnimationType::DISPLAY_TEST:
    case AnimationType::DATA_MATRIX:
      return true;
    default:
      return false;
  }
}

void AnimationManager::SetAnimation(AnimationType type, bool loop) {
  ESP_LOGI(TAG, "Display animation changed: %s => %s",
           animation_->Type().toString(), type.toString());

  animation_->Exiting();
  animation_ = GetAnimationInstance(type);
  loop_ = loop;
  animation_->SetDataSource(&data_source_);
  animation_->Entering();
  last_frame_time_ = esp_timer_get_time();
}

void AnimationManager::HandleTransition(const AnimationAction &action) {
  AnimationType type = static_cast<AnimationType::Value>(action.next_animation);
  bool loop = action.next_loop;

  // Product family check with fallback from AnimationAction
  BaseAnimation *anim = GetAnimationInstance(type);
  anim->SetDataSource(&data_source_);
  if (!anim->CanBePlayed()) {
    AnimationType fallback =
        static_cast<AnimationType::Value>(action.fallback_animation);
    if (fallback == AnimationType::NO_ANIMATIONS) {
      return;  // Stay at current
    }
    BaseAnimation *fallback_anim = GetAnimationInstance(fallback);
    fallback_anim->SetDataSource(&data_source_);
    if (!fallback_anim->CanBePlayed()) {
      return;  // Stay at current
    }
    ESP_LOGI(TAG, "Transition to %s rejected, using fallback %s",
             type.toString(), fallback.toString());
    type = fallback;
    loop = action.fallback_loop;
  }

  SetAnimation(type, loop);
}

void AnimationManager::TaskLoop() {
  BaseAnimation::Init();
  running_ = true;
  last_frame_time_ = esp_timer_get_time();

  while (running_) {
    // Skip frame execution when display is off
    if (DisplayManager::GetInstance().GetMode() == DisplayMode::OFF) {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
      // Reset frame time when display is off to prevent huge dt_us on recovery
      last_frame_time_ = esp_timer_get_time();
      continue;
    }

    // Process queued animation (inline, replaces SwitchAnimation)
    xSemaphoreTake(mutex_, portMAX_DELAY);
    AnimationType next_type = next_animation_type_;
    bool next_loop = next_animation_loop_;
    next_animation_type_ = AnimationType::NO_ANIMATIONS;
    next_animation_loop_ = false;
    xSemaphoreGive(mutex_);

    if (next_type != AnimationType::NO_ANIMATIONS &&
        next_type != animation_->Type()) {
      SetAnimation(next_type, next_loop);
    }

    // Play frame with capped delta time
    int64_t now = esp_timer_get_time();
    int64_t dt_us = now - last_frame_time_;
    // Cap dt_us to prevent animation fast-forward after pauses
    constexpr int64_t kMaxDeltaUs = 100000;  // 100ms max
    if (dt_us > kMaxDeltaUs) {
      dt_us = kMaxDeltaUs;
    }
    AnimationAction result = animation_->PlayNextFrame(dt_us);
    last_frame_time_ = now;

    // Handle result
    switch (result.status) {
      case AnimationAction::TRANSITION:
        HandleTransition(result);
        break;
      case AnimationAction::COMPLETE:
        if (!loop_) {
          SetAnimation(AnimationType::IDLE, true);
        }
        break;
      case AnimationAction::CONTINUE:
        break;
    }

    // Sleep
    WakeInterval interval = result.wake_interval;
    if (interval.us == 0) {
      interval = WakeInterval::Polling(ANIMATION_FRAME_INTERVAL_US);
    }

    if (interval != WakeInterval::Never()) {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(interval.us / 1000));
    } else {
      // Never wake - wait for external event
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
  }
  vTaskDelete(NULL);
}

void AnimationManager::StopTask() {
  if (!running_ || task_handle_ == nullptr) {
    return;  // Already stopped or never started
  }

  ESP_LOGI(TAG, "Stopping AnimationManager task");
  running_ = false;

  // Wake up the task so it can exit immediately
  xTaskNotifyGive(task_handle_);

  // Wait for task to actually finish (max 1 second)
  for (int i = 0; i < 100; i++) {
    eTaskState task_state = eTaskGetState(task_handle_);
    if (task_state == eDeleted || task_state == eInvalid) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // Check if task exited cleanly
  eTaskState final_state = eTaskGetState(task_handle_);
  if (final_state != eDeleted && final_state != eInvalid) {
    ESP_LOGW(TAG,
             "Animation task did not exit cleanly within 1 second (state: %d)",
             final_state);
  }

  task_handle_ = nullptr;
  ESP_LOGI(TAG, "AnimationManager task stopped");
}

AnimationType AnimationManager::GetCurrentAnimationType() const {
  if (animation_ == nullptr) {
    return AnimationType::NO_ANIMATIONS;
  }
  return animation_->Type();
}
