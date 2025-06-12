#include "animation_manager.h"

#include <algorithm>
#include <map>
#include <vector>

#include "display_animation.h"
#include "esp_err.h"
#include "esp_log.h"
#include "utils.h"

static const char *TAG = "AnimationManager";

static const std::map<AnimationType, std::vector<AnimationType>>
    AnimationTransitionRules = {
        {AnimationType::DOWNLOAD_STAGE1,
         {AnimationType::DOWNLOAD_STAGE2, AnimationType::DOWNLOAD_STAGE3,
          AnimationType::IDLE_ANIMATION}},
        {AnimationType::DOWNLOAD_STAGE2,
         {AnimationType::DOWNLOAD_STAGE3, AnimationType::IDLE_ANIMATION}},
        {AnimationType::DOWNLOAD_STAGE3,
         {AnimationType::UPDATE_3566_STAGE1, AnimationType::UPDATE_3566_STAGE2,
          AnimationType::UPDATE_3566_STAGE3, AnimationType::UPDATE_3566_STAGE4,
          AnimationType::UPDATE_3566_STAGE5}},
        {AnimationType::UPDATE_3566_STAGE1,
         {AnimationType::UPDATE_3566_STAGE2, AnimationType::UPDATE_3566_STAGE3,
          AnimationType::UPDATE_3566_STAGE4,
          AnimationType::UPDATE_3566_STAGE5}},
        {AnimationType::UPDATE_3566_STAGE2,
         {AnimationType::UPDATE_3566_STAGE3, AnimationType::UPDATE_3566_STAGE4,
          AnimationType::UPDATE_3566_STAGE5}},
        {AnimationType::UPDATE_3566_STAGE3,
         {AnimationType::UPDATE_3566_STAGE4,
          AnimationType::UPDATE_3566_STAGE5}},
        {AnimationType::UPDATE_3566_STAGE4,
         {AnimationType::UPDATE_3566_STAGE5}},
        {
            AnimationType::UPDATE_3566_STAGE5,
            {AnimationType::UPDATE_3566_STAGE5, AnimationType::IDLE_ANIMATION},
        },
        {
            AnimationType::DEVICE_INSERTED,
            {AnimationType::IDLE_ANIMATION},
        },
        {
            AnimationType::DEVICE_REMOVED,
            {AnimationType::IDLE_ANIMATION},
        },
        {
            AnimationType::POWER_DISPLAY,
            {AnimationType::IDLE_ANIMATION},
        },
        {
            AnimationType::SCREEN_OFF,
            {AnimationType::IDLE_ANIMATION},
        },
};

AnimationManager &AnimationManager::GetInstance() {
  static AnimationManager instance;
  return instance;
}

esp_err_t AnimationManager::StartTask() {
  animation_ = &NoAnimation::GetInstance();
  BaseType_t ret = xTaskCreate(TaskLoopWrapper, "animation", 2 * 1024, this,
                               tskIDLE_PRIORITY + 1, &task_handle_);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create display task");
    return ESP_FAIL;
  }

  return ESP_OK;
}

void AnimationManager::SetNextAnimation(AnimationType animation_type,
                                        bool loop) {
  next_animation_type_ = animation_type;
  next_animation_loop_ = loop;
}

void AnimationManager::TaskLoopWrapper(void *arg) {
  AnimationManager *aManager = static_cast<AnimationManager *>(arg);
  aManager->TaskLoop();
}

void AnimationManager::SetAnimation(AnimationType animation_type, bool loop) {
  switch (animation_type) {
    case AnimationType::BOOT_APP:
      SetAnimation(&BootAppAnimation::GetInstance(), loop);
      break;
    case AnimationType::BLE_ADVERTISING:
      SetAnimation(&BleAdvertisingAnimation::GetInstance(), loop);
      break;
    case AnimationType::BLE_CONNECTED:
      SetAnimation(&BleConnectedAnimation::GetInstance(), loop);
      break;
    case AnimationType::DOWNLOAD_STAGE1:
      SetAnimation(&DownloadStage1Animation::GetInstance(), loop);
      break;
    case AnimationType::DOWNLOAD_STAGE2:
      SetAnimation(&DownloadStage2Animation::GetInstance(), loop);
      break;
    case AnimationType::DOWNLOAD_STAGE3:
      SetAnimation(&DownloadStage3Animation::GetInstance(), loop);
      break;
    case AnimationType::UPDATE_3566_STAGE1:
      SetAnimation(&Update3566Stage1Animation::GetInstance(), loop);
      break;
    case AnimationType::UPDATE_3566_STAGE2:
      SetAnimation(&Update3566Stage2Animation::GetInstance(), loop);
      break;
    case AnimationType::UPDATE_3566_STAGE3:
      SetAnimation(&Update3566Stage3Animation::GetInstance(), loop);
      break;
    case AnimationType::UPDATE_3566_STAGE4:
      SetAnimation(&Update3566Stage4Animation::GetInstance(), loop);
      break;
    case AnimationType::UPDATE_3566_STAGE5:
      SetAnimation(&Update3566Stage5Animation::GetInstance(), loop);
      break;
    case AnimationType::WIFI_CONNECTING:
      SetAnimation(&WifiConnectingAnimation::GetInstance(), loop);
      break;
    case AnimationType::IDLE_ANIMATION:
      SetAnimation(&IdleAnimation::GetInstance(), loop);
      break;
    case AnimationType::DEVICE_INSERTED:
      SetAnimation(&DeviceInsertedAnimation::GetInstance(), loop);
      break;
    case AnimationType::DEVICE_REMOVED:
      SetAnimation(&DeviceRemovedAnimation::GetInstance(), loop);
      break;
    case AnimationType::POWER_DISPLAY:
      SetAnimation(&PowerDisplayAnimation::GetInstance(), loop);
      break;
    case AnimationType::SCREEN_OFF:
      SetAnimation(&ScreenOffAnimation::GetInstance(), loop);
      break;
    default:
      ESP_LOGW(TAG, "Unknown animation type: %d",
               static_cast<int>(animation_type));
      SetAnimation(&NoAnimation::GetInstance(), loop);
      break;
  }
}

void AnimationManager::SetAnimation(BaseAnimation *next_animation, bool loop) {
  ESP_LOGI(TAG, "Display animation changed: %s => %s",
           animation_->Type().toString(), next_animation->Type().toString());
  animation_->Exiting();
  animation_ = next_animation;
  loop_ = loop;
  animation_->Entering();
}

void AnimationManager::SwitchAnimation() {
  AnimationType next_animation_type = next_animation_type_;
  bool next_animation_loop = next_animation_loop_;
  next_animation_type_ = AnimationType::NO_ANIMATIONS;
  next_animation_loop_ = false;
  if (next_animation_type == AnimationType::NO_ANIMATIONS ||
      animation_->Type() == next_animation_type) {
    return;
  }

  if (!isTransitionAllowed(animation_->Type(), next_animation_type)) {
    ESP_LOGW(TAG, "Transition from %s to %s is not allowed",
             animation_->Type().toString(), next_animation_type.toString());
    return;
  }
  SetAnimation(next_animation_type, next_animation_loop);
}

void AnimationManager::TaskLoop() {
  while (true) {
    SwitchAnimation();
    if (animation_->Play() && !loop_) {
      // Animation has finished playing
      SetAnimation(AnimationType::IDLE_ANIMATION, true);
    };
    DELAY_MS(100);
  }
}

bool AnimationManager::isTransitionAllowed(AnimationType from,
                                           AnimationType to) {
  auto it = AnimationTransitionRules.find(from);
  if (it == AnimationTransitionRules.end()) {
    return true;
  }
  const auto &allowedTransitions = it->second;
  return std::find(allowedTransitions.begin(), allowedTransitions.end(), to) !=
         allowedTransitions.end();
}
