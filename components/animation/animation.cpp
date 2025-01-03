#include "animation.h"

#include <sys/stat.h>
#include <sys/types.h>

#include <array>
#include <climits>
#include <cstdint>

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
#include "utils.h"

#define EVENT_QUEUE_SIZE 5

static const char *TAG = "Animation";

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
    .segments = {{
        {0, 255, 200},
        {255, 000, 200},
    }},
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

AnimationController &AnimationController::GetInstance() {
  static AnimationController instance;
  return instance;
}

AnimationController::AnimationController()
    : animations({
          nullptr,
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
      }) {}

void AnimationController::perform_segment(uint8_t start_percent,
                                          uint8_t end_percent, int duration_ms,
                                          AnimationType animation_type) {
  int total_frames = duration_ms * frame_rate / 1000;
  int frame_duration_ms = 1000 / frame_rate;
  percent_per_frame =
      static_cast<double>(end_percent - start_percent) / total_frames;

  current_percent = start_percent;
  // if animation_type is INTENSITY, set the display percentage to 100% before
  // starting to avoid flickering and reducing fpga command
  if (animation_type == AnimationType::INTENSITY) {
    ESP_ERROR_COMPLAIN(rpc::display::set_display_percentage(100),
                       "set_display_percentage");
  }
  for (int frame = 0; frame < total_frames; frame++) {
    if (animation_type == AnimationType::INTENSITY) {
      ESP_ERROR_COMPLAIN(rpc::display::set_display_intensity(current_percent),
                         "set_display_intensity");
    } else {
      ESP_ERROR_COMPLAIN(rpc::display::set_display_percentage(current_percent),
                         "set_display_percentage");
    }
    vTaskDelay(pdMS_TO_TICKS(frame_duration_ms));
    current_percent += percent_per_frame;
  }
  if (animation_type == AnimationType::INTENSITY) {
    ESP_ERROR_COMPLAIN(rpc::display::set_display_intensity(end_percent),
                       "set_display_intensity");
  } else {
    ESP_ERROR_COMPLAIN(rpc::display::set_display_percentage(end_percent),
                       "set_display_percentage");
  }
}

void AnimationController::perform_animation(const Animation *animation) {
  if (curr_display_mode != MANUAL) {
    ESP_RETURN_VOID_ON_ERROR(rpc::display::set_display_mode(MANUAL), TAG,
                             "set_display_mode: %d", MANUAL);
    curr_display_mode = MANUAL;
  }

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
    perform_segment(animation->segments[i].start_percent,
                    animation->segments[i].end_percent,
                    animation->segments[i].duration_ms, animation->type);
  }
}

void AnimationController::TaskLoop(void *arg) {
  ESP_LOGI(TAG, "Animation task created");
  uint32_t event;
  BaseType_t xResult;
  AnimationEvent new_event = {
      .animation_id = AnimationId::NO_ANIMATION,
      .loop_mode = LoopMode::ONCE,
  };
  AnimationController &aController = AnimationController::GetInstance();

  while (true) {
    event = 0;
    xResult = xTaskNotifyWait(ULONG_MAX, ULONG_MAX, &event, pdMS_TO_TICKS(20));
    if (xResult == pdPASS) {
      memcpy(&new_event, &event, sizeof(AnimationEvent));
      aController.curr_animation_id = new_event.animation_id;
    }
    if (new_event.animation_id == AnimationId::NO_ANIMATION) {
      aController.stop_animation(AnimationId::NO_ANIMATION);
      continue;
    }
    if (!aController.allow_perform(new_event.animation_id)) {
      continue;
    }
    const Animation animation =
        *aController.animations[(uint8_t)new_event.animation_id];

    aController.perform_animation(&animation);
    if (new_event.loop_mode == LoopMode::ONCE) {
      new_event.animation_id = AnimationId::NO_ANIMATION;
      aController.back_to_prev_state();
    }
  }
}

esp_err_t AnimationController::start_task() {
  if (taskHandle_ != nullptr) {
    return ESP_OK;
  }
  BaseType_t ret =
      xTaskCreate(TaskLoop, "animation", 2 * 1024, this, 1, &taskHandle_);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create animation task");
    return ESP_FAIL;
  }

  return ESP_OK;
}

void AnimationController::set_animation(AnimationId animation_id,
                                        LoopMode loop_mode, bool force) {
  if (taskHandle_ == nullptr) {
    ESP_LOGW(TAG, "Animation task not created");
    return;
  }
  if (!force && is_high_priority_animation(curr_animation_id)) {
    if (!is_high_priority_animation(animation_id)) {
      return;
    }
  }
  ESP_LOGI(TAG, "Setting animation: %d, loop mode: %d", (uint8_t)animation_id,
           loop_mode);
  set_prev_state();
  AnimationEvent event = {
      .animation_id = animation_id,
      .loop_mode = loop_mode,
  };
  uint32_t event_;
  memcpy(&event_, &event, sizeof(AnimationEvent));
  xTaskNotify(taskHandle_, event_, eSetValueWithOverwrite);
}

void AnimationController::stop_animation(AnimationId animation_id) {
  if (taskHandle_ == nullptr ||
      curr_animation_id == AnimationId::NO_ANIMATION) {
    return;
  }
  ESP_LOGI(TAG, "Stopping animation: %d", (uint8_t)animation_id);
  if (animation_id != AnimationId::NO_ANIMATION &&
      animation_id != curr_animation_id) {
    return;
  }
  if (curr_animation_id == AnimationId::NO_ANIMATION) {
    return;
  }
  curr_animation_id = AnimationId::NO_ANIMATION;
  curr_loop_mode = LoopMode::ONCE;
  curr_display_mode = DISPLAY_DEFAULT_MODE;
  back_to_prev_state();
  xTaskNotify(taskHandle_, (uint32_t)AnimationId::NO_ANIMATION,
              eSetValueWithOverwrite);
  ESP_ERROR_COMPLAIN(rpc::display::set_display_mode(DISPLAY_DEFAULT_MODE),
                     "set_display_mode: %d", DISPLAY_DEFAULT_MODE);
  ESP_ERROR_COMPLAIN(
      rpc::display::set_display_intensity(DISPLAY_DEFAULT_INTENSITY),
      "set_display_intensity: %d", DISPLAY_DEFAULT_INTENSITY);
}

bool AnimationController::allow_perform(AnimationId animation_id) {
  bool allowed = false;
  if (AllowedAnimations[(uint8_t)curr_animation_id][0] ==
      (uint8_t)AnimationId::MAX_ANIMATIONS) {
    allowed = true;
  } else {
    for (auto val : AllowedAnimations[(uint8_t)curr_animation_id]) {
      if (val == (uint8_t)animation_id) {
        allowed = true;
        break;
      }
    }
  }
  if (!allowed) {
    ESP_LOGI(TAG,
             "Animation %d is not allowed to perform while %d is "
             "performing",
             (uint8_t)animation_id, (uint8_t)curr_animation_id);
  }
  return allowed;
}

void AnimationController::set_prev_state() {
  DeviceController *controller = DeviceController::GetInstance();
  if (controller) {
    prev_display_mode = controller->get_display_mode();
    prev_intensity = controller->get_display_intensity();
  }
}
void AnimationController::back_to_prev_state() {
  if (curr_display_mode != prev_display_mode) {
    ESP_ERROR_COMPLAIN(rpc::display::set_display_mode(prev_display_mode),
                       "set_display_mode: %d", prev_display_mode);
  }
  if (curr_intensity != prev_intensity) {
    ESP_ERROR_COMPLAIN(rpc::display::set_display_intensity(prev_intensity),
                       "set_display_intensity");
  }
  curr_display_mode = prev_display_mode;
  curr_intensity = prev_intensity;
}