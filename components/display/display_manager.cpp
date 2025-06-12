#include "display_manager.h"

#include "animation_manager.h"
#include "display_animation.h"
#include "display_types.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_LINUX
#undef ULONG_MAX
#define ULONG_MAX 0xFFFFFFFF
#else
#include <climits>
#endif

#if defined(CONFIG_DISPLAY_DRIVER_FPGA)
#include "rpc.h"
#elif defined(CONFIG_DISPLAY_DRIVER_LVGL)
#include "bsp/bsp_generic.h"
#include "bsp/display.h"
#include "esp_lv_decoder.h"

#define display_backlight_on bsp_display_backlight_off
#define display_backlight_off bsp_display_backlight_on
#endif

static const char *TAG = "DisplayManager";

DisplayManager &DisplayManager::GetInstance() {
  static DisplayManager instance;
  return instance;
}

DisplayManager::DisplayManager()
    : display_percentage_(0),
      display_mode_(0),
      display_intensity_(0),
      display_flip_mode_(0),
      task_handle_(nullptr) {
#if defined(CONFIG_DISPLAY_DRIVER_FPGA)
  display_mode_ = CONFIG_DISPLAY_MODE;
  display_intensity_ = CONFIG_DISPLAY_INTENSITY;
#endif
}

esp_err_t DisplayManager::Init() {
#if defined(CONFIG_DISPLAY_DRIVER_FPGA) || defined(CONFIG_DISPLAY_DRIVER_NONE)
  return ESP_OK;
#elif defined(CONFIG_DISPLAY_DRIVER_LVGL)
  ESP_RETURN_ON_FALSE(bsp_display_start() != NULL, ESP_FAIL, TAG,
                      "bsp_display_start");
  ESP_RETURN_ON_ERROR(display_backlight_on(), TAG,
                      "Failed to turn on display backlight");
  lv_disp_set_rotation(NULL, LV_DISP_ROT_NONE);
  esp_lv_decoder_handle_t decoder_handle = NULL;
  esp_lv_decoder_init(&decoder_handle);  // Initialize this after lvgl starts
  return ESP_OK;
#else
#error "No display driver selected"
#endif
}

esp_err_t DisplayManager::StartTask() {
#ifdef CONFIG_DISPLAY_DRIVER_NONE
  return ESP_OK;
#else
  if (task_handle_ != nullptr) {
    return ESP_OK;
  }
  ESP_RETURN_ON_ERROR(AnimationManager::GetInstance().StartTask(), TAG,
                      "start animation manager task");
  BaseType_t ret = xTaskCreate(TaskLoopWrapper, "display", 1.5 * 1024, this,
                               tskIDLE_PRIORITY + 1, &task_handle_);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create display task");
    task_handle_ = nullptr;
    return ESP_FAIL;
  }

  return ESP_OK;
#endif
}

void DisplayManager::Notify(DisplayEvent event, bool overwrite) {
#ifdef CONFIG_DISPLAY_DRIVER_NONE
  return;
#else
  if (task_handle_ == nullptr) {
    ESP_LOGW(TAG, "Display task not created");
    return;
  }
  ESP_LOGD(TAG, "Notifying display task: %d, overwrite: %d",
           (uint8_t)event.animation_type_value, overwrite);
  xTaskNotify(task_handle_, event.raw,
              overwrite ? eSetValueWithOverwrite : eSetValueWithoutOverwrite);
#endif
}

void DisplayManager::SetAnimation(AnimationType animation_type, bool loop,
                                  bool overwrite) {
  // temporarily ignore ble and wifi animations
  if (animation_type == AnimationType::BLE_ADVERTISING ||
      animation_type == AnimationType::BLE_CONNECTED ||
      animation_type == AnimationType::WIFI_CONNECTING) {
    return;
  }
  Notify({.animation_type_value = animation_type.getValue(),
          .loop = loop,
          .reserved = {}},
         overwrite);
}

esp_err_t DisplayManager::SetDisplayPercentage(uint8_t percentage) {
#if defined(CONFIG_DISPLAY_DRIVER_FPGA)
  esp_err_t err = rpc::fpga::set_display_percentage(percentage);
  if (err == ESP_OK) {
    display_percentage_ = percentage;
  }
  return err;
#elif defined(CONFIG_DISPLAY_DRIVER_NONE)
  // No display driver, do nothing
  return ESP_OK;
#else
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t DisplayManager::SetDisplayMode(uint8_t mode) {
#if defined(CONFIG_DISPLAY_DRIVER_FPGA)
  esp_err_t err = rpc::fpga::set_display_mode(mode);
  if (err == ESP_OK && mode != DisplayMode::OFF) {
    display_mode_ = mode;
  }
  return err;
#elif defined(CONFIG_DISPLAY_DRIVER_LVGL)
  switch (mode) {
    case DisplayMode::OFF:
      return display_backlight_off();
    default:
      display_mode_ = mode;
      return display_backlight_on();
  }
  return ESP_OK;
#elif defined(CONFIG_DISPLAY_DRIVER_NONE)
  // No display driver, do nothing
  return ESP_OK;
#else
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t DisplayManager::SetDisplayIntensity(uint8_t intensity) {
#if defined(CONFIG_DISPLAY_DRIVER_FPGA)
  esp_err_t err = rpc::fpga::set_display_intensity(intensity);
  if (err == ESP_OK) {
    display_intensity_ = intensity;
  }
  return err;
#elif defined(CONFIG_DISPLAY_DRIVER_LVGL)
  return bsp_display_brightness_set(intensity);
#elif defined(CONFIG_DISPLAY_DRIVER_NONE)
  // No display driver, do nothing
  return ESP_OK;
#else
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t DisplayManager::SetDisplayFlipMode(uint8_t mode) {
#if defined(CONFIG_DISPLAY_DRIVER_FPGA)
  esp_err_t err = rpc::fpga::set_display_flip_mode(mode);
  if (err == ESP_OK) {
    display_flip_mode_ = mode;
  }
  return err;
#elif defined(CONFIG_DISPLAY_DRIVER_LVGL)
  switch (mode) {
    case DisplayFlipMode::NORMAL:
      lv_disp_set_rotation(NULL, LV_DISP_ROT_NONE);
      break;
    case DisplayFlipMode::FLIP:
      lv_disp_set_rotation(NULL, LV_DISP_ROT_180);
      break;
    default:
      ESP_LOGW(TAG, "Invalid flip mode: %d", mode);
      return ESP_ERR_INVALID_ARG;
  }
  return ESP_OK;
#elif defined(CONFIG_DISPLAY_DRIVER_NONE)
  // No display driver, do nothing
  return ESP_OK;
#else
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t DisplayManager::DisplayOff() {
  return SetDisplayMode(DisplayMode::OFF);
}

void DisplayManager::TaskLoopWrapper(void *arg) {
  DisplayManager *aManager = static_cast<DisplayManager *>(arg);
  aManager->TaskLoop();
}

void DisplayManager::TaskLoop() {
  ESP_LOGI(TAG, "DisplayManager task created");

  while (true) {
    DisplayEvent event;
    BaseType_t received =
        xTaskNotifyWait(ULONG_MAX, ULONG_MAX, &event.raw, pdMS_TO_TICKS(20));
    if (received == pdTRUE) {
      AnimationType type = AnimationType(event.animation_type_value);
      ESP_LOGD(TAG, "Set next animation: %s, loop: %d", type.toString(),
               event.loop);
      AnimationManager::GetInstance().SetNextAnimation(type, event.loop);
    }
  }
}
