#include "animation.h"

#include <sys/stat.h>
#include <sys/types.h>

#include <algorithm>
#include <array>
#include <climits>
#include <cstdint>
#include <cstring>
#include <functional>
#include <map>
#include <vector>

#include "controller.h"
#include "data_types.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "ionbridge.h"
#include "portmacro.h"
#include "rpc.h"
#include "sdkconfig.h"

static const char *TAG = "Animation";

#define DISPLAY_DEFAULT_MODE CONFIG_DISPLAY_MODE
#define DISPLAY_DEFAULT_INTENSITY CONFIG_DISPLAY_INTENSITY
static constexpr int frame_rate = CONFIG_ANIMATION_FRAME_RATE;

static const Animation kBootAnimation = {
    .type = PERCENTAGE,
    .reversed = false,
    .intensity = MaxDisplayIntensity,
    .adjust_intensity = true,
    .segments = {{
        {0, 25, 500},
        {25, 50, 1000},
        {50, 75, 500},
        {75, 100, 1000},
    }},
};

static const Animation kBleAdvertisingAnimation{
    .type = PERCENTAGE,
    .reversed = false,
    .intensity = NormalDisplayIntensity,
    .adjust_intensity = false,
    .segments = {{
        {100, 100, 300},
        {0, 0, 300},
    }},
};

static const Animation kBleConnectedAnimation = {
    .type = INTENSITY,
    .reversed = false,
    .intensity = NormalDisplayIntensity,
    .adjust_intensity = false,
    .segments = {{{0, 200, 200}, {255, 0, 300}, {0, 0, 200}}},
};

static const Animation kDownloadStage1Animation = {
    .type = PERCENTAGE,
    .reversed = false,
    .intensity = MaxDisplayIntensity,
    .adjust_intensity = true,
    .segments = {{
        {0, 33, 1000},
    }},
};

static const Animation kDownloadStage2Animation = {
    .type = PERCENTAGE,
    .reversed = false,
    .intensity = MaxDisplayIntensity,
    .adjust_intensity = true,
    .segments = {{
        {0, 66, 1000},
    }},
};

static const Animation kDownloadStage3Animation = {
    .type = PERCENTAGE,
    .reversed = false,
    .intensity = MaxDisplayIntensity,
    .adjust_intensity = true,
    .segments = {{
        {0, 100, 1000},
    }},
};

static const Animation kUpdate3566Stage1Animation = {
    .type = PERCENTAGE,
    .reversed = false,
    .intensity = MaxDisplayIntensity,
    .adjust_intensity = true,
    .segments = {{
        {0, 20, 1000},
    }},
};

static const Animation kUpdate3566Stage2Animation = {
    .type = PERCENTAGE,
    .reversed = false,
    .intensity = MaxDisplayIntensity,
    .adjust_intensity = true,
    .segments = {{
        {0, 40, 1000},
    }},
};

static const Animation kUpdate3566Stage3Animation = {
    .type = PERCENTAGE,
    .reversed = false,
    .intensity = MaxDisplayIntensity,
    .adjust_intensity = true,
    .segments = {{
        {0, 60, 1000},
    }},
};

static const Animation kUpdate3566Stage4Animation = {
    .type = PERCENTAGE,
    .reversed = false,
    .intensity = MaxDisplayIntensity,
    .adjust_intensity = true,
    .segments = {{
        {0, 80, 1000},
    }},
};

static const Animation kUpdate3566Stage5Animation = {
    .type = PERCENTAGE,
    .reversed = false,
    .intensity = MaxDisplayIntensity,
    .adjust_intensity = true,
    .segments = {{
        {0, 100, 1000},
    }},
};

static const Animation kWifiConnectingAnimation = {
    .type = INTENSITY,
    .reversed = false,
    .intensity = NormalDisplayIntensity,
    .adjust_intensity = false,
    .segments = {{
        {0, 200, 800},
        {255, 0, 1000},
        {0, 0, 200},
    }},
};

static const std::array<const Animation *, 12> animations = {
    &kBootAnimation,
    &kBleAdvertisingAnimation,
    &kBleConnectedAnimation,
    &kDownloadStage1Animation,
    &kDownloadStage2Animation,
    &kDownloadStage3Animation,
    &kUpdate3566Stage1Animation,
    &kUpdate3566Stage2Animation,
    &kUpdate3566Stage3Animation,
    &kUpdate3566Stage4Animation,
    &kUpdate3566Stage5Animation,
    &kWifiConnectingAnimation,
};

static const std::map<AnimationId, std::vector<AnimationId>>
    AnimationTransitionRules = {
        {AnimationId::DOWNLOAD_STAGE1,
         {AnimationId::DOWNLOAD_STAGE2, AnimationId::DOWNLOAD_STAGE3}},
        {AnimationId::DOWNLOAD_STAGE2, {AnimationId::DOWNLOAD_STAGE3}},
        {AnimationId::DOWNLOAD_STAGE3,
         {AnimationId::UPDATE_3566_STAGE1, AnimationId::UPDATE_3566_STAGE2,
          AnimationId::UPDATE_3566_STAGE3, AnimationId::UPDATE_3566_STAGE4,
          AnimationId::UPDATE_3566_STAGE5}},
        {AnimationId::UPDATE_3566_STAGE1,
         {AnimationId::UPDATE_3566_STAGE2, AnimationId::UPDATE_3566_STAGE3,
          AnimationId::UPDATE_3566_STAGE4, AnimationId::UPDATE_3566_STAGE5}},
        {AnimationId::UPDATE_3566_STAGE2,
         {AnimationId::UPDATE_3566_STAGE3, AnimationId::UPDATE_3566_STAGE4,
          AnimationId::UPDATE_3566_STAGE5}},
        {AnimationId::UPDATE_3566_STAGE3,
         {AnimationId::UPDATE_3566_STAGE4, AnimationId::UPDATE_3566_STAGE5}},
        {AnimationId::UPDATE_3566_STAGE4, {AnimationId::UPDATE_3566_STAGE5}},
        {AnimationId::UPDATE_3566_STAGE5, {AnimationId::UPDATE_3566_STAGE5}},
};

AnimationController &AnimationController::GetInstance() {
  static AnimationController instance;
  return instance;
}

void AnimationController::PlaySegment(uint8_t start_percent,
                                      uint8_t end_percent, int duration_ms,
                                      AnimationType animation_type) {
  int total_frames = duration_ms * frame_rate / 1000;
  int frame_duration_ms = 1000 / frame_rate;
  std::function<esp_err_t(uint8_t)> set_display_func;
  double percent_per_frame = 0.0, current_percent = 0.0;

  percent_per_frame =
      static_cast<double>(end_percent - start_percent) / total_frames;

  current_percent = start_percent;
  // if animation_type is INTENSITY, set the display percentage to 100% before
  // starting to avoid flickering and reducing fpga command
  if (animation_type == AnimationType::INTENSITY) {
    ESP_ERROR_COMPLAIN(rpc::display::set_display_percentage(100),
                       "set_display_percentage");
    set_display_func = rpc::display::set_display_intensity;
  } else {
    set_display_func = rpc::display::set_display_percentage;
  }

  for (int frame = 0; frame < total_frames; frame++) {
    ESP_ERROR_COMPLAIN(set_display_func(current_percent),
                       "set_display failed: %d", animation_type);
    vTaskDelay(pdMS_TO_TICKS(frame_duration_ms));
    current_percent += percent_per_frame;
  }
  ESP_ERROR_COMPLAIN(set_display_func(end_percent),
                     "set_display end failed: %d", animation_type);
}

void AnimationController::PlayAnimation(const Animation *animation) {
  // Change display mode to manual to play animation frame by frame
  ESP_RETURN_VOID_ON_ERROR(rpc::display::set_display_mode(MANUAL), TAG,
                           "set_display_mode: %d", MANUAL);

  if (animation->reversed) {
    ESP_RETURN_VOID_ON_ERROR(
        rpc::display::set_display_flip_mode(DisplayFlipMode::FLIP), TAG,
        "set_display_flip_mode: FLIP");
  }
  if (animation->adjust_intensity) {
    ESP_RETURN_VOID_ON_ERROR(
        rpc::display::set_display_intensity(animation->intensity), TAG,
        "set_display_intensity");
  }

  for (int i = 0; i < animation->segments.size(); i++) {
    if (animation->segments[i].duration_ms == 0) {
      break;
    }
    PlaySegment(animation->segments[i].start_percent,
                animation->segments[i].end_percent,
                animation->segments[i].duration_ms, animation->type);
  }
}

void AnimationController::TaskLoopWrapper(void *arg) {
  AnimationController *aController = static_cast<AnimationController *>(arg);
  aController->TaskLoop();
}

void AnimationController::TaskLoop() {
  ESP_LOGI(TAG, "Animation task created");
  BaseType_t xResult;
  uint32_t event_val;
  AnimationEvent curr_event = {
      .event_type = AnimationEventType::STOP,
      .animation_id = AnimationId::NO_ANIMATIONS,
      .loop = false,
      .reversed = 0,
  };
  AnimationEvent event;

  while (true) {
    xResult =
        xTaskNotifyWait(ULONG_MAX, ULONG_MAX, &event_val, pdMS_TO_TICKS(20));
    if (xResult != pdPASS) {
      // no new event, check if current animation is finished
      if (curr_event.animation_id == AnimationId::NO_ANIMATIONS ||
          curr_event.event_type == AnimationEventType::STOP ||
          !curr_event.loop) {
        // no animation to perform currently, wait for next event
        continue;
      }
      // back to previous event
      event = curr_event;
    } else {
      memcpy(&event, &event_val, sizeof(event));
    }

    if (event.event_type == AnimationEventType::STOP) {
      // Stop current animation if it is playing
      if (curr_event.animation_id == event.animation_id) {
        StopAnimation(&curr_event);
      }
      continue;
    }
    StopAnimation(&curr_event);

    if (xResult == pdPASS &&
        !isTransitionAllowed(curr_event.animation_id, event.animation_id)) {
      continue;
    }

    curr_event = event;
    auto animation = animations[(uint8_t)event.animation_id];
    PlayAnimation(animation);

    if (!event.loop) {
      // Play the animation once
      StopAnimation(&curr_event);
      curr_event.animation_id = AnimationId::NO_ANIMATIONS;
      curr_event.event_type = AnimationEventType::STOP;
    }
  }
}

esp_err_t AnimationController::StartTask() {
  if (task_handle != nullptr) {
    return ESP_OK;
  }
  BaseType_t ret = xTaskCreate(TaskLoopWrapper, "animation", 2 * 1024, this, 1,
                               &task_handle);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create animation task");
    return ESP_FAIL;
  }

  return ESP_OK;
}

void AnimationController::StartAnimation(AnimationId animation_id, bool loop) {
  if (task_handle == nullptr) {
    ESP_LOGW(TAG, "Animation task not created");
    return;
  }
  // temporarily ignore ble and wifi animations
  if (animation_id == AnimationId::BLE_ADVERTISING ||
      animation_id == AnimationId::BLE_CONNECTED ||
      animation_id == AnimationId::WIFI_CONNECTING) {
    return;
  }
  ESP_LOGI(TAG, "Starting animation: %d, loop: %d", (uint8_t)animation_id,
           loop);
  AnimationEvent event = {
      .event_type = AnimationEventType::PLAY,
      .animation_id = animation_id,
      .loop = loop,
      .reversed = 0,
  };
  xTaskNotify(task_handle, *(uint32_t *)&event, eSetValueWithOverwrite);
}

void AnimationController::StopAnimation(AnimationId animation_id) {
  if (task_handle == nullptr) {
    return;
  }
  ESP_LOGI(TAG, "Stopping animation: %d", (uint8_t)animation_id);
  AnimationEvent event = {
      .event_type = AnimationEventType::STOP,
      .animation_id = animation_id,
      .loop = false,
      .reversed = 0,
  };
  xTaskNotify(task_handle, *(uint32_t *)&event, eSetValueWithOverwrite);
}

void AnimationController::StopAnimation(AnimationEvent *event) {
  event->animation_id = AnimationId::NO_ANIMATIONS;
  event->event_type = AnimationEventType::STOP;
  event->loop = false;

  DeviceController *controller = DeviceController::GetInstance();
  uint8_t display_mode = DISPLAY_DEFAULT_MODE,
          display_intensity = DISPLAY_DEFAULT_INTENSITY;
  if (controller != nullptr) {
    display_mode = controller->get_display_mode();
    display_intensity = controller->get_display_intensity();
    ESP_LOGD(TAG, "Restoring display mode: %d, intensity: %d", display_mode,
             display_intensity);
  }

  ESP_ERROR_COMPLAIN(rpc::display::set_display_mode(display_mode),
                     "set_display_mode: %d", display_mode);
  ESP_ERROR_COMPLAIN(rpc::display::set_display_intensity(display_intensity),
                     "set_display_intensity");
}

bool AnimationController::isTransitionAllowed(AnimationId current,
                                              AnimationId next) {
  // Check if there are explicit rules for the current animation.
  auto it = AnimationTransitionRules.find(current);
  if (it != AnimationTransitionRules.end()) {
    const auto &allowedTransitions = it->second;
    // Transition is allowed only if the next animation is in the allowed list.
    return std::find(allowedTransitions.begin(), allowedTransitions.end(),
                     next) != allowedTransitions.end();
  }
  // If no rules are defined, allow transition to any animation.
  return true;
}
