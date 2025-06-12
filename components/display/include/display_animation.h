#ifndef H_DISPLAY_ANIMATION_
#define H_DISPLAY_ANIMATION_

#include <array>
#include <cstdint>
#include <string>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/idf_additions.h"
#include "singleton.h"

#if defined(CONFIG_DISPLAY_DRIVER_LVGL)
#include "lvgl.h"
#endif

#define ANIMATION_MAX_FILE_COUNT 10

class AnimationType {
 public:
  enum Value : uint8_t {
    BOOT_APP,
    BLE_ADVERTISING,
    BLE_CONNECTED,
    DOWNLOAD_STAGE1,
    DOWNLOAD_STAGE2,
    DOWNLOAD_STAGE3,
    UPDATE_3566_STAGE1,
    UPDATE_3566_STAGE2,
    UPDATE_3566_STAGE3,
    UPDATE_3566_STAGE4,
    UPDATE_3566_STAGE5,
    WIFI_CONNECTING,
    IDLE_ANIMATION,
    DEVICE_INSERTED,
    DEVICE_REMOVED,
    POWER_DISPLAY,
    SCREEN_OFF,

    NO_ANIMATIONS = 0xFF,
  };

  AnimationType() = default;
  constexpr AnimationType(Value animation) : value(animation) {}

  // Allow switch and comparisons
  constexpr operator Value() const { return value; }

  // Prevent usage: if(animation)
  explicit operator bool() const = delete;

  // Add equality operators
  constexpr bool operator==(AnimationType other) const {
    return value == other.value;
  }
  constexpr bool operator==(Value other) const { return value == other; }
  constexpr bool operator!=(AnimationType other) const {
    return value != other.value;
  }

  // Add value getter method
  constexpr Value getValue() const { return value; }

  // Moved operator+ into the class
  constexpr AnimationType operator+(uint8_t rhs) const {
    uint8_t res = static_cast<uint8_t>(value) + rhs;
    if (res >= static_cast<uint8_t>(NO_ANIMATIONS)) {
      return AnimationType(NO_ANIMATIONS);
    }
    return AnimationType(static_cast<Value>(res));
  }

  // Convert the animation type to a string representation
  const char *toString() const;

 private:
  Value value;
};

void play_bar_animation(AnimationType type);

class BaseAnimation {
 protected:
  AnimationType type_;

  BaseAnimation() = default;
  ~BaseAnimation() = default;

  explicit BaseAnimation(AnimationType animation_type)
      : type_(animation_type) {}

 public:
  virtual void Entering() {};
  virtual bool Play() { return true; };
  virtual void Exiting() {};

  AnimationType Type() const { return type_; }
};

class BootAppAnimation : public BaseAnimation,
                         public Singleton<BootAppAnimation> {
 public:
  bool Play() override;
#if defined(CONFIG_DISPLAY_DRIVER_LVGL)
  void Entering() override;
  void Exiting() override;
#endif

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<BootAppAnimation>;
  // Private constructor to prevent external instantiation
  BootAppAnimation() : BaseAnimation(AnimationType::BOOT_APP) {}

#if defined(CONFIG_DISPLAY_DRIVER_LVGL)
  static void CompleteCallback(void *arg);
  void Complete() { completed_ = true; };

  lv_obj_t *animimg_ = nullptr;
  char *files_[ANIMATION_MAX_FILE_COUNT];
  int file_count_ = 0;
  bool completed_ = false;
#endif
};

template <AnimationType::Value T>
class GenericAnimation : public BaseAnimation,
                         public Singleton<GenericAnimation<T>> {
 public:
  bool Play() override {
    play_bar_animation(type_);
    return true;
  }

 private:
  friend class Singleton<GenericAnimation<T>>;
  GenericAnimation() : BaseAnimation(T) {}
};
using BleAdvertisingAnimation =
    GenericAnimation<AnimationType::BLE_ADVERTISING>;
using BleConnectedAnimation = GenericAnimation<AnimationType::BLE_CONNECTED>;
using DownloadStage1Animation =
    GenericAnimation<AnimationType::DOWNLOAD_STAGE1>;
using DownloadStage2Animation =
    GenericAnimation<AnimationType::DOWNLOAD_STAGE2>;
using DownloadStage3Animation =
    GenericAnimation<AnimationType::DOWNLOAD_STAGE3>;
using Update3566Stage1Animation =
    GenericAnimation<AnimationType::UPDATE_3566_STAGE1>;
using Update3566Stage2Animation =
    GenericAnimation<AnimationType::UPDATE_3566_STAGE2>;
using Update3566Stage3Animation =
    GenericAnimation<AnimationType::UPDATE_3566_STAGE3>;
using Update3566Stage4Animation =
    GenericAnimation<AnimationType::UPDATE_3566_STAGE4>;
using Update3566Stage5Animation =
    GenericAnimation<AnimationType::UPDATE_3566_STAGE5>;
using WifiConnectingAnimation =
    GenericAnimation<AnimationType::WIFI_CONNECTING>;

class IdleAnimation : public BaseAnimation, public Singleton<IdleAnimation> {
 public:
  void Entering() override;
  bool Play() override { return false; };
#if defined(CONFIG_DISPLAY_DRIVER_LVGL)
  void Exiting() override;
#endif

 private:
  friend class Singleton<IdleAnimation>;
  IdleAnimation() : BaseAnimation(AnimationType::IDLE_ANIMATION) {}

#if defined(CONFIG_DISPLAY_DRIVER_LVGL)
  lv_obj_t *animimg_ = nullptr;
  char *files_[ANIMATION_MAX_FILE_COUNT];
  int file_count_ = 0;
#endif
};

class DeviceInsertedAnimation : public BaseAnimation,
                                public Singleton<DeviceInsertedAnimation> {
 public:
  bool Play() override;
  void Exiting() override;

 private:
  friend class Singleton<DeviceInsertedAnimation>;
  DeviceInsertedAnimation() : BaseAnimation(AnimationType::DEVICE_INSERTED) {}

#if defined(CONFIG_DISPLAY_DRIVER_LVGL)
  lv_obj_t *label_ = nullptr;
#endif
};

class DeviceRemovedAnimation : public BaseAnimation,
                               public Singleton<DeviceRemovedAnimation> {
 public:
  bool Play() override;
  void Exiting() override;

 private:
  friend class Singleton<DeviceRemovedAnimation>;
  DeviceRemovedAnimation() : BaseAnimation(AnimationType::DEVICE_REMOVED) {}

#if defined(CONFIG_DISPLAY_DRIVER_LVGL)
  lv_obj_t *label_ = nullptr;
#endif
};

class PowerDisplayAnimation : public BaseAnimation,
                              public Singleton<PowerDisplayAnimation> {
 public:
  bool Play() override;
  void Exiting() override;

 private:
  friend class Singleton<PowerDisplayAnimation>;
  PowerDisplayAnimation() : BaseAnimation(AnimationType::POWER_DISPLAY) {}

#if defined(CONFIG_DISPLAY_DRIVER_LVGL)
  lv_obj_t *label_ = nullptr;
#endif
};

class ScreenOffAnimation : public BaseAnimation,
                           public Singleton<ScreenOffAnimation> {
 public:
  void Entering() override;
  bool Play() override { return false; };
  void Exiting() override;

 private:
  friend class Singleton<ScreenOffAnimation>;
  ScreenOffAnimation() : BaseAnimation(AnimationType::SCREEN_OFF) {}
};

class NoAnimation : public BaseAnimation, public Singleton<NoAnimation> {
 public:
  bool Play() override { return false; };

 private:
  friend class Singleton<NoAnimation>;
  NoAnimation() : BaseAnimation(AnimationType::NO_ANIMATIONS) {}
};

#endif /* H_DISPLAY_ANIMATION_ */
