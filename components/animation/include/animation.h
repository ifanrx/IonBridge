#ifndef H_ANIMATION_
#define H_ANIMATION_

#include <array>
#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "freertos/idf_additions.h"
#include "sdkconfig.h"

#define DISPLAY_DEFAULT_MODE CONFIG_DISPLAY_MODE
#define DISPLAY_DEFAULT_INTENSITY CONFIG_DISPLAY_INTENSITY

typedef struct {
  uint8_t start_percent;
  uint8_t end_percent;
  uint16_t duration_ms;
} AnimationSegment;

typedef enum {
  PERCENTAGE = 0x00,
  INTENSITY = 0x01,
} AnimationType;

typedef struct {
  AnimationType type;
  bool reversed;
  uint8_t intensity;
  bool adjust_intensity;  // Controls whether the animation should change
                          // intensity
  std::array<AnimationSegment, 5> segments;
} Animation;
typedef enum : uint8_t {
  ONCE = 0x00,
  FOREVER = 0x01,
} LoopMode;

enum class AnimationId : uint8_t {
  NO_ANIMATION = 0,
  BOOT_APP = 1,
  BLE_ADVERTISING = 2,
  BLE_CONNECTED = 3,
  DOWNLOAD_STAGE1 = 4,
  DOWNLOAD_STAGE2 = 5,
  DOWNLOAD_STAGE3 = 6,
  UPDATE_3566_STAGE1 = 7,
  UPDATE_3566_STAGE2 = 8,
  UPDATE_3566_STAGE3 = 9,
  UPDATE_3566_STAGE4 = 10,
  UPDATE_3566_STAGE5 = 11,
  WIFI_CONNECTING = 12,
  MAX_ANIMATIONS = 13,
};

typedef struct __attribute__((packed)) {
  AnimationId animation_id;
  LoopMode loop_mode;
} AnimationEvent;

// allowedanimation [1: [1,2,3], 2:[1,2]]
const std::array<std::array<uint8_t, 13>, 13> AllowedAnimations = {{
    {(uint8_t)AnimationId::MAX_ANIMATIONS},  // NO_ANIMATION
    {(uint8_t)AnimationId::MAX_ANIMATIONS},  // BOOT_APP
    {(uint8_t)AnimationId::MAX_ANIMATIONS},  // BLE_ADVERTISING
    {(uint8_t)AnimationId::MAX_ANIMATIONS},  // BLE_CONNECTED
    {(uint8_t)AnimationId::DOWNLOAD_STAGE2,
     (uint8_t)AnimationId::DOWNLOAD_STAGE3},  // DOWNLOAD_STAGE1
    {(uint8_t)AnimationId::DOWNLOAD_STAGE3},  // DOWNLOAD_STAGE2
    {(uint8_t)AnimationId::UPDATE_3566_STAGE1,
     (uint8_t)AnimationId::UPDATE_3566_STAGE2,
     (uint8_t)AnimationId::UPDATE_3566_STAGE3,
     (uint8_t)AnimationId::UPDATE_3566_STAGE4,
     (uint8_t)AnimationId::UPDATE_3566_STAGE5},  // DOWNLOAD_STAGE3
    {(uint8_t)AnimationId::UPDATE_3566_STAGE2,
     (uint8_t)AnimationId::UPDATE_3566_STAGE3,
     (uint8_t)AnimationId::UPDATE_3566_STAGE4,
     (uint8_t)AnimationId::UPDATE_3566_STAGE5},  // UPDATE_3566_STAGE1
    {(uint8_t)AnimationId::UPDATE_3566_STAGE3,
     (uint8_t)AnimationId::UPDATE_3566_STAGE4,
     (uint8_t)AnimationId::UPDATE_3566_STAGE5},  // UPDATE_3566_STAGE2
    {(uint8_t)AnimationId::UPDATE_3566_STAGE4,
     (uint8_t)AnimationId::UPDATE_3566_STAGE5},  // UPDATE_3566_STAGE3
    {(uint8_t)AnimationId::UPDATE_3566_STAGE5},  // UPDATE_3566_STAGE4
    {(uint8_t)AnimationId::MAX_ANIMATIONS},      // UPDATE_3566_STAGE5
    {(uint8_t)AnimationId::MAX_ANIMATIONS},      // WIFI_CONNECTING
}};

// Non-member operator+
inline AnimationId operator+(AnimationId lhs, uint8_t rhs) {
  uint8_t res = static_cast<uint8_t>(lhs) + rhs;
  if (res >= static_cast<uint8_t>(AnimationId::MAX_ANIMATIONS)) {
    return AnimationId::NO_ANIMATION;
  }
  return static_cast<AnimationId>(res);
}

class AnimationController {
  static constexpr int frame_rate = CONFIG_ANIMATION_FRAME_RATE;
  const std::array<const Animation *, 13> animations;

  // Task management
  TaskHandle_t taskHandle_;

  static void TaskLoop(void *arg);  // Task loop

  AnimationId curr_animation_id = AnimationId::NO_ANIMATION,
              prev_animation_id = AnimationId::NO_ANIMATION;
  LoopMode curr_loop_mode = LoopMode::ONCE, prev_loop_mode = LoopMode::ONCE;
  uint8_t curr_intensity = DISPLAY_DEFAULT_INTENSITY,
          curr_display_mode = DISPLAY_DEFAULT_MODE,
          prev_display_mode = DISPLAY_DEFAULT_MODE,
          prev_intensity = DISPLAY_DEFAULT_INTENSITY;
  double percent_per_frame = 0.0, current_percent = 0.0;

  // singleton
 public:
  static AnimationController &GetInstance();
  AnimationController(const AnimationController &) = delete;
  AnimationController &operator=(const AnimationController &) = delete;

  esp_err_t start_task();
  void set_animation(AnimationId animation_id, LoopMode loop_mode,
                     bool force = false);
  void set_animation(uint8_t animation_id, LoopMode loop_mode,
                     bool force = false) {
    if (animation_id >= (uint8_t)AnimationId::MAX_ANIMATIONS) return;
    return set_animation((AnimationId)animation_id, loop_mode, force);
  }
  void stop_animation(AnimationId animation_id);
  void stop_animation(uint8_t animation_id) {
    if (animation_id >= (uint8_t)AnimationId::MAX_ANIMATIONS) return;
    return stop_animation((AnimationId)animation_id);
  }
  void perform_animation(const Animation *animation);
  void perform_segment(uint8_t start_percent, uint8_t end_percent,
                       int duration_ms, AnimationType type);
  void perform_previous_animation() {
    set_animation(prev_animation_id, prev_loop_mode, true);
  }
  bool is_high_priority_animation(AnimationId animation_id) {
    uint8_t val = (uint8_t)animation_id;
    return val >= (uint8_t)AnimationId::DOWNLOAD_STAGE1 &&
           val <= (uint8_t)AnimationId::UPDATE_3566_STAGE5;
  }
  bool allow_perform(AnimationId animation_id);
  void set_prev_state();
  void back_to_prev_state();

 private:
  AnimationController();
  ~AnimationController() = default;
};

#endif  // H_ANIMATION_
