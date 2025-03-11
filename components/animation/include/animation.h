#ifndef H_ANIMATION_
#define H_ANIMATION_

#include <array>
#include <cstdint>

#include "esp_err.h"
#include "freertos/idf_additions.h"

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

enum class AnimationId : uint8_t {
  BOOT_APP = 0,
  BLE_ADVERTISING = 1,
  BLE_CONNECTED = 2,
  DOWNLOAD_STAGE1 = 3,
  DOWNLOAD_STAGE2 = 4,
  DOWNLOAD_STAGE3 = 5,
  UPDATE_3566_STAGE1 = 6,
  UPDATE_3566_STAGE2 = 7,
  UPDATE_3566_STAGE3 = 8,
  UPDATE_3566_STAGE4 = 9,
  UPDATE_3566_STAGE5 = 10,
  WIFI_CONNECTING = 11,
  NO_ANIMATIONS = 12,
};

enum class AnimationEventType : uint8_t {
  PLAY = 0,
  STOP = 1,
};

// size of the struct is 4 bytes, fits in the notification value: uint32_t
typedef struct __attribute__((packed)) {
  AnimationEventType event_type;
  AnimationId animation_id;
  bool loop;
  uint8_t reversed;
} AnimationEvent;

// Non-member operator+
inline AnimationId operator+(AnimationId lhs, uint8_t rhs) {
  uint8_t res = static_cast<uint8_t>(lhs) + rhs;
  if (res >= static_cast<uint8_t>(AnimationId::NO_ANIMATIONS)) {
    return AnimationId::NO_ANIMATIONS;
  }
  return static_cast<AnimationId>(res);
}

class AnimationController {
  AnimationController() = default;
  ~AnimationController() = default;

  // Task management
  TaskHandle_t task_handle;
  static void TaskLoopWrapper(void *arg);
  void TaskLoop();

  // singleton
 public:
  static AnimationController &GetInstance();
  AnimationController(const AnimationController &) = delete;
  AnimationController &operator=(const AnimationController &) = delete;

  esp_err_t StartTask();
  void StartAnimation(AnimationId animation_id, bool loop);
  void StopAnimation(AnimationId animation_id);
  void StopAnimation(AnimationEvent *event);
  void PlayAnimation(const Animation *animation);
  void PlaySegment(uint8_t start_percent, uint8_t end_percent, int duration_ms,
                   AnimationType type);
  bool isTransitionAllowed(AnimationId current, AnimationId next);
};

#endif  // H_ANIMATION_
