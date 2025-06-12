#include "display_animation.h"

#include <dirent.h>

#include <cstddef>

#include "display_manager.h"
#include "display_types.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "utils.h"

#if defined(CONFIG_DISPLAY_DRIVER_FPGA)
#include <functional>
#include <map>

#include "ionbridge.h"

static constexpr int frame_rate = CONFIG_ANIMATION_FRAME_RATE;
#elif defined(CONFIG_DISPLAY_DRIVER_LVGL)
#include "bsp/bsp_generic.h"
#include "misc/lv_anim.h"
#include "nvs_namespace.h"

#define DISPLAY_UNLOCK_GOTO_ON_NULL(obj, goto_tag, log_tag, format, ...) \
  if (unlikely(obj == NULL)) {                                           \
    ESP_LOGE(log_tag, format, ##__VA_ARGS__);                            \
    bsp_display_unlock();                                                \
    goto goto_tag;                                                       \
  }

#define DISPLAY_LOCK_TIMEOUT_MS CONFIG_DISPLAY_LOCK_TIMEOUT_MS
#endif

static const char *TAG = "DisplayAnimation";

const char *AnimationType::toString() const {
  switch (value) {
    case AnimationType::BOOT_APP:
      return "BOOT_APP";
    case AnimationType::BLE_ADVERTISING:
      return "BLE_ADVERTISING";
    case AnimationType::BLE_CONNECTED:
      return "BLE_CONNECTED";
    case AnimationType::DOWNLOAD_STAGE1:
      return "DOWNLOAD_STAGE1";
    case AnimationType::DOWNLOAD_STAGE2:
      return "DOWNLOAD_STAGE2";
    case AnimationType::DOWNLOAD_STAGE3:
      return "DOWNLOAD_STAGE3";
    case AnimationType::UPDATE_3566_STAGE1:
      return "UPDATE_3566_STAGE1";
    case AnimationType::UPDATE_3566_STAGE2:
      return "UPDATE_3566_STAGE2";
    case AnimationType::UPDATE_3566_STAGE3:
      return "UPDATE_3566_STAGE3";
    case AnimationType::UPDATE_3566_STAGE4:
      return "UPDATE_3566_STAGE4";
    case AnimationType::UPDATE_3566_STAGE5:
      return "UPDATE_3566_STAGE5";
    case AnimationType::WIFI_CONNECTING:
      return "WIFI_CONNECTING";
    case AnimationType::IDLE_ANIMATION:
      return "IDLE_ANIMATION";
    case AnimationType::DEVICE_INSERTED:
      return "DEVICE_INSERTED";
    case AnimationType::DEVICE_REMOVED:
      return "DEVICE_REMOVED";
    case AnimationType::POWER_DISPLAY:
      return "POWER_DISPLAY";
    case AnimationType::SCREEN_OFF:
      return "SCREEN_OFF";
    case AnimationType::NO_ANIMATIONS:
      return "NO_ANIMATIONS";
    default:
      return "UNKNOWN";
  }
}

#if defined(CONFIG_DISPLAY_DRIVER_FPGA)
typedef struct {
  uint8_t start_percent;
  uint8_t end_percent;
  uint16_t duration_ms;
} BarAnimationSegment;

typedef enum {
  PERCENTAGE = 0x00,
  INTENSITY = 0x01,
} BarAnimationType;

typedef struct {
  BarAnimationType type;
  bool reversed;
  uint8_t intensity;
  bool adjust_intensity;  // Controls whether the animation should change
                          // intensity
  std::array<BarAnimationSegment, 5> segments;
} BarAnimation;

static const BarAnimation kBootAnimation = {
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

static const BarAnimation kBleAdvertisingAnimation{
    .type = PERCENTAGE,
    .reversed = false,
    .intensity = NormalDisplayIntensity,
    .adjust_intensity = false,
    .segments = {{
        {100, 100, 300},
        {0, 0, 300},
    }},
};

static const BarAnimation kBleConnectedAnimation = {
    .type = INTENSITY,
    .reversed = false,
    .intensity = NormalDisplayIntensity,
    .adjust_intensity = false,
    .segments = {{{0, 200, 200}, {255, 0, 300}, {0, 0, 200}}},
};

static const BarAnimation kDownloadStage1Animation = {
    .type = PERCENTAGE,
    .reversed = false,
    .intensity = MaxDisplayIntensity,
    .adjust_intensity = true,
    .segments = {{
        {0, 33, 1000},
    }},
};

static const BarAnimation kDownloadStage2Animation = {
    .type = PERCENTAGE,
    .reversed = false,
    .intensity = MaxDisplayIntensity,
    .adjust_intensity = true,
    .segments = {{
        {0, 66, 1000},
    }},
};

static const BarAnimation kDownloadStage3Animation = {
    .type = PERCENTAGE,
    .reversed = false,
    .intensity = MaxDisplayIntensity,
    .adjust_intensity = true,
    .segments = {{
        {0, 100, 1000},
    }},
};

static const BarAnimation kUpdate3566Stage1Animation = {
    .type = PERCENTAGE,
    .reversed = false,
    .intensity = MaxDisplayIntensity,
    .adjust_intensity = true,
    .segments = {{
        {0, 20, 1000},
    }},
};

static const BarAnimation kUpdate3566Stage2Animation = {
    .type = PERCENTAGE,
    .reversed = false,
    .intensity = MaxDisplayIntensity,
    .adjust_intensity = true,
    .segments = {{
        {0, 40, 1000},
    }},
};

static const BarAnimation kUpdate3566Stage3Animation = {
    .type = PERCENTAGE,
    .reversed = false,
    .intensity = MaxDisplayIntensity,
    .adjust_intensity = true,
    .segments = {{
        {0, 60, 1000},
    }},
};

static const BarAnimation kUpdate3566Stage4Animation = {
    .type = PERCENTAGE,
    .reversed = false,
    .intensity = MaxDisplayIntensity,
    .adjust_intensity = true,
    .segments = {{
        {0, 80, 1000},
    }},
};

static const BarAnimation kUpdate3566Stage5Animation = {
    .type = PERCENTAGE,
    .reversed = false,
    .intensity = MaxDisplayIntensity,
    .adjust_intensity = true,
    .segments = {{
        {0, 100, 1000},
    }},
};

static const BarAnimation kWifiConnectingAnimation = {
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

static const std::map<AnimationType, const BarAnimation *> kBarAnimations = {
    {AnimationType::BOOT_APP, &kBootAnimation},
    {AnimationType::BLE_ADVERTISING, &kBleAdvertisingAnimation},
    {AnimationType::BLE_CONNECTED, &kBleConnectedAnimation},
    {AnimationType::DOWNLOAD_STAGE1, &kDownloadStage1Animation},
    {AnimationType::DOWNLOAD_STAGE2, &kDownloadStage2Animation},
    {AnimationType::DOWNLOAD_STAGE3, &kDownloadStage3Animation},
    {AnimationType::UPDATE_3566_STAGE1, &kUpdate3566Stage1Animation},
    {AnimationType::UPDATE_3566_STAGE2, &kUpdate3566Stage2Animation},
    {AnimationType::UPDATE_3566_STAGE3, &kUpdate3566Stage3Animation},
    {AnimationType::UPDATE_3566_STAGE4, &kUpdate3566Stage4Animation},
    {AnimationType::UPDATE_3566_STAGE5, &kUpdate3566Stage5Animation},
    {AnimationType::WIFI_CONNECTING, &kWifiConnectingAnimation},
};

static esp_err_t set_display_intensity(uint8_t intensity) {
  DisplayManager &display_manager = DisplayManager::GetInstance();
  return display_manager.SetDisplayIntensity(intensity);
}

static esp_err_t set_display_percentage(uint8_t percentage) {
  DisplayManager &display_manager = DisplayManager::GetInstance();
  return display_manager.SetDisplayPercentage(percentage);
}

void play_segment(uint8_t start_percent, uint8_t end_percent, int duration_ms,
                  BarAnimationType animation_type) {
  int total_frames = duration_ms * frame_rate / 1000;
  int frame_duration_ms = 1000 / frame_rate;
  std::function<esp_err_t(uint8_t)> set_display_func;
  double percent_per_frame = 0.0, current_percent = 0.0;
  DisplayManager &display_manager = DisplayManager::GetInstance();

  percent_per_frame =
      static_cast<double>(end_percent - start_percent) / total_frames;

  current_percent = start_percent;
  // if animation_type is INTENSITY, set the display percentage to 100%
  //   before starting to avoid flickering and reducing fpga command
  if (animation_type == BarAnimationType::INTENSITY) {
    ESP_ERROR_COMPLAIN(display_manager.SetDisplayPercentage(100),
                       "set_display_percentage");
    set_display_func = set_display_intensity;
  } else {
    set_display_func = set_display_percentage;
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

void play_bar_animation(AnimationType animation_type) {
  auto it = kBarAnimations.find(animation_type);
  if (it == kBarAnimations.end()) {
    ESP_LOGE(TAG, "BarAnimation not supported for type: %s",
             animation_type.toString());
    return;
  }

  const BarAnimation *animation = it->second;
  DisplayManager &display_manager = DisplayManager::GetInstance();

  ESP_RETURN_VOID_ON_ERROR(display_manager.SetDisplayMode(DisplayMode::MANUAL),
                           TAG, "Failed to set display mode");

  // Initialize display based on animation type
  if (animation->type == PERCENTAGE) {
    ESP_ERROR_COMPLAIN(display_manager.SetDisplayPercentage(0),
                       "Failed to set display percentage");
  } else if (animation->type == INTENSITY) {
    ESP_ERROR_COMPLAIN(display_manager.SetDisplayIntensity(0),
                       "Failed to set display intensity");
  }

  // Play each animation segment
  for (const BarAnimationSegment &segment : animation->segments) {
    if (segment.duration_ms == 0) {
      break;
    }

    play_segment(segment.start_percent, segment.end_percent,
                 segment.duration_ms, animation->type);
  }
}
#elif defined(CONFIG_DISPLAY_DRIVER_LVGL)
void play_bar_animation(AnimationType animation_type) {}
#endif

#if defined(CONFIG_DISPLAY_DRIVER_FPGA)
bool BootAppAnimation::Play() {
  play_bar_animation(type_);
  return true;
}
#elif defined(CONFIG_DISPLAY_DRIVER_LVGL)

constexpr int kFileBufferSize = 64;
static char file_buffer_array[ANIMATION_MAX_FILE_COUNT][kFileBufferSize] = {};

static int get_files(const char *directory, char *files[], int max_files,
                     int bufsize) {
  __attribute__((unused)) esp_err_t ret = ESP_OK;
  int file_count = 0;
  DIR *dir = opendir(directory);
  if (dir == NULL) {
    ESP_LOGE(TAG, "Failed to open directory: %s", directory);
    return -1;
  }
  struct dirent *entry;
  while ((entry = readdir(dir)) != NULL) {
    if (file_count >= max_files) {
      break;
    }
    if (entry->d_type == DT_REG) {
      int n = snprintf(files[file_count], bufsize, "A:%s/%s", directory,
                       entry->d_name);
      if (n < 0 || n >= bufsize) {
        ESP_LOGE(TAG, "File path too long: %s/%s", directory, entry->d_name);
        continue;
      }
      file_count++;
    }
  }
  closedir(dir);
  return file_count;
}

void BootAppAnimation::Entering() {
  completed_ = true;
  const char *directory = CONFIG_ANIMATION_IMAGE_DIR "/boot";
  for (int i = 0; i < ANIMATION_MAX_FILE_COUNT; i++) {
    files_[i] = file_buffer_array[i];
  }
  file_count_ = get_files(directory, (char **)files_, ANIMATION_MAX_FILE_COUNT,
                          kFileBufferSize);
  ESP_RETURN_VOID_ON_FALSE(file_count_ > 0, ESP_FAIL, TAG,
                           "Failed to get files from directory: %s", directory);
  ESP_RETURN_VOID_ON_FALSE(bsp_display_lock(DISPLAY_LOCK_TIMEOUT_MS),
                           ESP_FAILED, TAG, "bsp_display_lock");
  animimg_ = lv_animimg_create(lv_scr_act());
  lv_anim_t *anim;
  DISPLAY_UNLOCK_GOTO_ON_NULL(animimg_, END, TAG, "lv_animimg_create");
  lv_obj_center(animimg_);
  lv_animimg_set_src(animimg_, (const void **)files_, file_count_);
  lv_animimg_set_duration(animimg_, CONFIG_BOOT_ANIMATION_PLAY_DURATION);
  lv_animimg_set_repeat_count(animimg_, 1);
  anim = &((lv_animimg_t *)animimg_)->anim;
  lv_anim_set_ready_cb(anim, (lv_anim_deleted_cb_t)CompleteCallback);
  lv_animimg_start(animimg_);
  bsp_display_unlock();
  completed_ = false;
END:
  return;
}

#define LVGL_OBJ_DEL(obj) \
  if ((obj) != nullptr) { \
    bsp_display_lock(0);  \
    lv_obj_del(obj);      \
    (obj) = nullptr;      \
    bsp_display_unlock(); \
  }

void BootAppAnimation::Exiting() {
  file_count_ = 0;
  LVGL_OBJ_DEL(animimg_);
}

bool BootAppAnimation::Play() { return completed_; }
void BootAppAnimation::CompleteCallback(void *arg) {
  BootAppAnimation::GetInstance().Complete();
}

#endif

#if defined(CONFIG_DISPLAY_DRIVER_FPGA)
void IdleAnimation::Entering() {
  DisplayManager &display_manager = DisplayManager::GetInstance();
  ESP_ERROR_COMPLAIN(display_manager.SetDisplayMode(DisplayMode::POWER_METER),
                     "SetDisplayMode");
  ESP_ERROR_COMPLAIN(display_manager.SetDisplayIntensity(128),
                     "SetDisplayIntensity");
  ESP_ERROR_COMPLAIN(display_manager.SetDisplayPercentage(0),
                     "SetDisplayPercentage");
}

#elif defined(CONFIG_DISPLAY_DRIVER_LVGL)
void IdleAnimation::Entering() {
  // Get animation directory from NVS or use default
  char directory[32] = {0};
  size_t size = sizeof(directory) - 1;  // Ensure space for null terminator

  esp_err_t err =
      DisplayNVSGet(directory, &size, NVSKey::DISPLAY_IDLE_ANIMATION);
  if (err != ESP_OK) {
    if (err != ESP_ERR_NVS_NOT_FOUND) {
      ESP_LOGW(TAG,
               "Failed to get idle animation directory: 0x%04x, using default",
               err);
    }
    strlcpy(directory, CONFIG_ANIMATION_IMAGE_DIR "/idle", sizeof(directory));
  }
  for (int i = 0; i < ANIMATION_MAX_FILE_COUNT; i++) {
    files_[i] = file_buffer_array[i];
  }
  file_count_ = get_files(directory, (char **)files_, ANIMATION_MAX_FILE_COUNT,
                          kFileBufferSize);
  if (file_count_ <= 0) {
    ESP_LOGE(TAG, "No animation images found in directory: %s", directory);
    return;
  }
  ESP_RETURN_VOID_ON_FALSE(bsp_display_lock(DISPLAY_LOCK_TIMEOUT_MS),
                           ESP_FAILED, TAG, "bsp_display_lock");
  esp_err_t ret __attribute__((unused)) = ESP_OK;
  animimg_ = lv_animimg_create(lv_scr_act());
  ESP_GOTO_ON_FALSE(animimg_ != NULL, ESP_FAIL, END, TAG, "lv_animimg_create");
  lv_obj_center(animimg_);
  lv_animimg_set_src(animimg_, (const void **)files_, file_count_);
  lv_animimg_set_duration(animimg_, CONFIG_IDLE_ANIMATION_PLAY_DURATION);
  lv_animimg_set_repeat_count(animimg_, LV_ANIM_REPEAT_INFINITE);
  lv_animimg_start(animimg_);
END:
  bsp_display_unlock();
}
void IdleAnimation::Exiting() {
  file_count_ = 0;
  LVGL_OBJ_DEL(animimg_);
}

#endif

bool DeviceInsertedAnimation::Play() {
#if defined(CONFIG_DISPLAY_DRIVER_LVGL)
  esp_err_t ret __attribute__((unused)) = ESP_OK;
  ESP_GOTO_ON_FALSE(bsp_display_lock(DISPLAY_LOCK_TIMEOUT_MS), ESP_FAIL, END,
                    TAG, "bsp_display_lock");
  label_ = lv_label_create(lv_scr_act());
  DISPLAY_UNLOCK_GOTO_ON_NULL(label_, END, TAG, "lv_label_create");
  lv_label_set_text(label_, "Inserted");
  lv_obj_align(label_, LV_ALIGN_CENTER, 0, 0);
  bsp_display_unlock();
  DELAY_MS(3000);
END:
#endif
  return true;
}

void DeviceInsertedAnimation::Exiting() {
#if defined(CONFIG_DISPLAY_DRIVER_LVGL)
  LVGL_OBJ_DEL(label_);
#endif
}

bool DeviceRemovedAnimation::Play() {
#if defined(CONFIG_DISPLAY_DRIVER_LVGL)
  esp_err_t ret __attribute__((unused)) = ESP_OK;
  ESP_GOTO_ON_FALSE(bsp_display_lock(DISPLAY_LOCK_TIMEOUT_MS), ESP_FAIL, END,
                    TAG, "bsp_display_lock");
  label_ = lv_label_create(lv_scr_act());
  DISPLAY_UNLOCK_GOTO_ON_NULL(label_, END, TAG, "lv_label_create");
  lv_label_set_text(label_, "Removed");
  lv_obj_align(label_, LV_ALIGN_CENTER, 0, 0);
  bsp_display_unlock();
  DELAY_MS(3000);
END:
#endif
  return true;
}

void DeviceRemovedAnimation::Exiting() {
#if defined(CONFIG_DISPLAY_DRIVER_LVGL)
  LVGL_OBJ_DEL(label_);
#endif
}

bool PowerDisplayAnimation::Play() {
#if defined(CONFIG_DISPLAY_DRIVER_LVGL)
  esp_err_t ret __attribute__((unused)) = ESP_OK;
  ESP_GOTO_ON_FALSE(bsp_display_lock(DISPLAY_LOCK_TIMEOUT_MS), ESP_FAIL, END,
                    TAG, "bsp_display_lock");
  label_ = lv_label_create(lv_scr_act());
  DISPLAY_UNLOCK_GOTO_ON_NULL(label_, END, TAG, "lv_label_create");
  lv_label_set_text(label_, "160W");
  lv_obj_align(label_, LV_ALIGN_CENTER, 0, 0);
  bsp_display_unlock();
  DELAY_MS(3000);
END:
#endif
  return true;
}

void PowerDisplayAnimation::Exiting() {
#if defined(CONFIG_DISPLAY_DRIVER_LVGL)
  LVGL_OBJ_DEL(label_);
#endif
}

void ScreenOffAnimation::Entering() {
  DisplayManager &display_manager = DisplayManager::GetInstance();
  display_manager.SetDisplayMode(DisplayMode::OFF);
}

void ScreenOffAnimation::Exiting() {
  DisplayManager &display_manager = DisplayManager::GetInstance();
  display_manager.SetDisplayMode(DisplayMode::POWER_METER);
}
