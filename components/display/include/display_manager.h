#ifndef H_DISPLAY_MANAGER_
#define H_DISPLAY_MANAGER_

#include <cstdint>

#include "display_animation.h"
#include "esp_err.h"
#include "freertos/idf_additions.h"

typedef struct {
  union {
    struct {
      // Change to the underlying type and provide conversion functions
      AnimationType::Value
          animation_type_value;  // Using the enum value directly
      bool loop;
      uint8_t reserved[2];
    };
    uint32_t raw;
  };
} DisplayEvent;

class DisplayManager {
 public:
  static DisplayManager &GetInstance();

  esp_err_t Init();
  esp_err_t StartTask();
  void Notify(DisplayEvent event, bool overwrite = true);
  void SetAnimation(AnimationType animation_type, bool loop,
                    bool overwrite = true);

  esp_err_t SetDisplayPercentage(uint8_t percentage);
  esp_err_t SetDisplayMode(uint8_t mode);
  esp_err_t SetDisplayIntensity(uint8_t intensity);
  esp_err_t SetDisplayMode() { return SetDisplayMode(display_mode_); }
  esp_err_t SetDisplayIntensity() {
    return SetDisplayIntensity(display_intensity_);
  }
  esp_err_t SetDisplayFlipMode(uint8_t mode);
  uint8_t GetDisplayPercentage() const { return display_percentage_; }
  uint8_t GetDisplayMode() const { return display_mode_; }
  uint8_t GetDisplayIntensity() const { return display_intensity_; }
  uint8_t GetDisplayFlipMode() const { return display_flip_mode_; }

  esp_err_t DisplayOn() { return SetDisplayMode(); }
  esp_err_t DisplayOff();

 private:
  DisplayManager();
  ~DisplayManager() = default;

  uint8_t display_percentage_, display_mode_, display_intensity_,
      display_flip_mode_;

  // Task management
  TaskHandle_t task_handle_;

  static void TaskLoopWrapper(void *arg);
  void TaskLoop();

  // singleton
  DisplayManager(const DisplayManager &) = delete;
  DisplayManager &operator=(const DisplayManager &) = delete;
};

#endif /* H_DISPLAY_MANAGER_ */
