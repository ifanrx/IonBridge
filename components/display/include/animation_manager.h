#ifndef H_ANIMATION_MANAGER_
#define H_ANIMATION_MANAGER_

#include "display_animation.h"

class AnimationManager {
 public:
  AnimationManager() = default;
  ~AnimationManager() = default;

  static AnimationManager &GetInstance();
  esp_err_t StartTask();
  void SetNextAnimation(AnimationType animation_type, bool loop);

 private:
  TaskHandle_t task_handle_;
  BaseAnimation *animation_ = nullptr;
  bool loop_ = false;
  AnimationType next_animation_type_ = AnimationType::NO_ANIMATIONS;
  bool next_animation_loop_;

  static void TaskLoopWrapper(void *arg);
  void TaskLoop();
  void SetAnimation(AnimationType animation_type, bool loop);
  void SetAnimation(BaseAnimation *animation, bool loop);
  void SwitchAnimation();
  bool isTransitionAllowed(AnimationType from, AnimationType to);
};

#endif /* H_ANIMATION_MANAGER_ */
