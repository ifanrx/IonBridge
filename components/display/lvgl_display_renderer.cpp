#include "lvgl_display_renderer.h"

#ifdef CONFIG_DISPLAY_DRIVER_LVGL

#include <dirent.h>

#include <algorithm>
#include <array>
#include <string>

#include "animation_manager.h"
#include "bsp/bsp_generic.h"
#include "data_types.pb.h"
#include "display_animation.h"
#include "display_types.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_lv_decoder.h"
#include "font/lv_font_loader.h"
#include "sdkconfig.h"

static const char* TAG = "LVGLRenderer";

// LVGL filesystem driver prefix for animation assets
#define LVGL_FS_PREFIX "A:"

// Display lock timeout
static const uint32_t DISPLAY_LOCK_TIMEOUT_MS = 1000;

static constexpr uint32_t kIdleMeteorFrameCount = 8;
static constexpr uint32_t kIdleMeteorPeriodMs = 200;

static uint8_t GetIdlePowerTier(IAnimationDataSource* data_source) {
  constexpr uint16_t kMediumThreshold = 10;
  constexpr uint16_t kHighThreshold = 50;

  if (!data_source) {
    return 0;
  }

  uint16_t total = data_source->GetTotalWatts();

  if (total >= kHighThreshold) return 2;
  if (total >= kMediumThreshold) return 1;
  return 0;
}

/**
 * @brief RAII wrapper for LVGL display lock
 *
 * Automatically acquires display lock on construction and releases on
 * destruction. This prevents deadlocks from early returns or exceptions.
 *
 * Example usage:
 *   DisplayLock lock;
 *   if (lock) {
 *     lv_obj_clean(lv_scr_act());  // Lock held
 *   }  // Lock automatically released
 */
class DisplayLock {
 public:
  DisplayLock() : locked_(bsp_display_lock(DISPLAY_LOCK_TIMEOUT_MS)) {
    if (!locked_) {
      ESP_LOGE(TAG, "Failed to acquire display lock");
    }
  }

  ~DisplayLock() {
    if (locked_) {
      bsp_display_unlock();
    }
  }

  // Delete copy/move to prevent double-unlock
  DisplayLock(const DisplayLock&) = delete;
  DisplayLock& operator=(const DisplayLock&) = delete;
  DisplayLock(DisplayLock&&) = delete;
  DisplayLock& operator=(DisplayLock&&) = delete;

  // Check if lock was acquired successfully
  explicit operator bool() const { return locked_; }

 private:
  bool locked_;
};

// Use common protocol name mapping from display_renderer_utils
using display_renderer_utils::GetFCProtocolName;

LVGLRenderer::LVGLRenderer(LVGLDisplayDriver* driver)
    : driver_(driver), current_content_(nullptr) {}

esp_err_t LVGLRenderer::Init() {
  ESP_LOGI(TAG, "Initializing LVGL renderer");
  ESP_RETURN_ON_ERROR(InitFonts(), TAG, "Failed to init fonts");
  ESP_RETURN_ON_ERROR(InitDecoder(), TAG, "Failed to init decoder");
  ESP_RETURN_ON_ERROR(InitScreenStyle(), TAG, "Failed to init screen style");
  InitStyles();
  return ESP_OK;
}

esp_err_t LVGLRenderer::Deinit() {
  ESP_LOGI(TAG, "Deinitializing LVGL renderer");

  // Clean up fonts
  for (size_t i = 0; i < FONT_KIND_MAX; i++) {
    if (fonts_[i]) {
      lv_font_free(fonts_[i]);
      fonts_[i] = NULL;
    }
  }

  return ESP_OK;
}

void LVGLRenderer::InitStyles() {
  lv_font_t* small_font = GetFont(FONT_KIND_SMALL);
  lv_font_t* middle_font = GetFont(FONT_KIND_MIDDLE);
  lv_font_t* large_font = GetFont(FONT_KIND_LARGE);

  lv_style_t* style_default = &styles_[STYLE_KIND_DEFAULT];
  lv_style_init(style_default);
  lv_style_set_text_font(style_default, small_font);
  lv_style_set_text_color(style_default, lv_color_white());
  lv_style_set_line_color(style_default, lv_color_white());
  lv_style_set_line_width(style_default, 2);
  lv_style_set_line_rounded(style_default, true);

  lv_style_t* style_dashed_line = &styles_[STYLE_KIND_DASHED_LINE];
  lv_style_init(style_dashed_line);
  lv_style_set_line_color(style_dashed_line, lv_color_white());
  lv_style_set_line_width(style_dashed_line, 2);
  lv_style_set_line_dash_width(style_dashed_line, 2);
  lv_style_set_line_dash_gap(style_dashed_line, 4);
  lv_style_set_line_rounded(style_dashed_line, true);

  lv_style_t* style_middle_font = &styles_[STYLE_KIND_MIDDLE_FONT];
  lv_style_init(style_middle_font);
  lv_style_set_text_font(style_middle_font, middle_font);
  lv_style_set_text_color(style_middle_font, lv_color_white());

  lv_style_t* style_large_font = &styles_[STYLE_KIND_LARGE_FONT];
  lv_style_init(style_large_font);
  lv_style_set_text_font(style_large_font, large_font);
  lv_style_set_text_color(style_large_font, lv_color_white());

  lv_style_t* style_small_font_and_color =
      &styles_[STYLE_KIND_SMALL_FONT_AND_COLOR];
  lv_style_init(style_small_font_and_color);
  lv_style_set_text_font(style_small_font_and_color, small_font);
  lv_style_set_text_color(style_small_font_and_color, lv_color_hex(0xFF623B));

  lv_style_t* style_small_font_and_align_right =
      &styles_[STYLE_KIND_SMALL_FONT_AND_ALIGN_RIGHT];
  lv_style_init(style_small_font_and_align_right);
  lv_style_set_text_font(style_small_font_and_align_right, small_font);
  lv_style_set_text_color(style_small_font_and_align_right, lv_color_white());
  lv_style_set_text_align(style_small_font_and_align_right,
                          LV_TEXT_ALIGN_RIGHT);
}

void LVGLRenderer::Clear() {
  DisplayLock lock;
  if (!lock) {
    ESP_LOGE(TAG, "Display lock timeout in %s", __func__);
    return;
  }

  lv_obj_clean(lv_scr_act());
  current_content_ = nullptr;

  // Reset animation state flags
  boot_animation_started_ = false;
  wifi_animation_started_ = false;
  idle_showing_pin_ = false;
  idle_last_wifi_connected_ = false;
  idle_last_type_ = UINT8_MAX;
  idle_last_power_tier_ = UINT8_MAX;
}

void LVGLRenderer::ShowText(const char* text, const TextStyle& style) {
  if (!text) {
    ESP_LOGW(TAG, "ShowText called with NULL text");
    return;
  }

  DisplayLock lock;
  if (!lock) {
    ESP_LOGE(TAG, "Display lock timeout in %s", __func__);
    return;
  }

  // Create label
  lv_obj_t* label = lv_label_create(lv_scr_act());
  lv_label_set_text(label, text);

  // Apply font
  lv_font_t* font = GetFontForSize(style.size);
  if (font) {
    lv_obj_set_style_text_font(label, font, 0);
  }

  // Apply alignment
  lv_align_t align = GetLVGLAlign(style.align);
  lv_obj_align(label, align, 0, 0);

  current_content_ = label;
}

void LVGLRenderer::ShowPercentage(uint8_t percentage, ProgressStyle style) {
  if (percentage > display_renderer_utils::kMaxPercentage) {
    percentage = display_renderer_utils::kMaxPercentage;
  }

#ifdef CONFIG_LV_USE_BAR
  DisplayLock lock;
  if (!lock) {
    ESP_LOGE(TAG, "Display lock timeout in %s", __func__);
    return;
  }

  // Create progress bar
  lv_obj_t* bar = lv_bar_create(lv_scr_act());
  lv_obj_set_size(bar, 200, 20);
  lv_bar_set_value(bar, percentage, LV_ANIM_ON);
  lv_obj_center(bar);

  // Apply color based on style
  lv_color_t color = GetColorForStyle(style);
  lv_obj_set_style_bg_color(bar, color, LV_PART_INDICATOR);

  current_content_ = bar;
#else
  // Fallback: show percentage as text when bar widget unavailable
  ESP_LOGW(TAG, "Bar widget not available, showing percentage as text: %d%%",
           percentage);
  TextStyle text_style;
  text_style.size = TextStyle::Size::LARGE;
  text_style.align = Alignment::CENTER;
  char buf[8];
  snprintf(buf, sizeof(buf), "%d%%", percentage);
  ShowText(buf, text_style);
#endif

  // Adjust hardware brightness
  uint8_t intensity = display_renderer_utils::GetIntensityForStyle(style);
  driver_->SetIntensity(intensity);
}

void LVGLRenderer::ShowIcon(IconType icon) {
  const char* icon_path = GetIconPath(icon);
  if (!icon_path) {
    ESP_LOGW(TAG, "No icon path for icon type");
    return;
  }

  DisplayLock lock;
  if (!lock) {
    ESP_LOGE(TAG, "Display lock timeout in %s", __func__);
    return;
  }

  // Create image object
  lv_obj_t* img = lv_img_create(lv_scr_act());
  lv_img_set_src(img, icon_path);
  lv_obj_center(img);

  current_content_ = img;
}

void LVGLRenderer::ShowImage(const char* path, Alignment align) {
  if (!path) {
    ESP_LOGW(TAG, "Image path is null");
    return;
  }

  // Add LVGL filesystem prefix if not already present
  std::string full_path;
  if (strncmp(path, LVGL_FS_PREFIX, strlen(LVGL_FS_PREFIX)) != 0) {
    full_path = LVGL_FS_PREFIX;
    full_path += path;
  } else {
    full_path = path;
  }

  DisplayLock lock;
  if (!lock) {
    ESP_LOGE(TAG, "Display lock timeout in %s", __func__);
    return;
  }

  // Create image object
  lv_obj_t* img = lv_img_create(lv_scr_act());
  if (!img) {
    ESP_LOGE(TAG, "Failed to create image object");
    return;  // Lock auto-released
  }

  lv_img_set_src(img, full_path.c_str());

  // Apply alignment
  lv_align_t lv_align = GetLVGLAlign(align);
  lv_obj_align(img, lv_align, 0, 0);

  current_content_ = img;
}

void LVGLRenderer::ShowAnimatedImage(const char* dir_path, uint32_t frame_count,
                                     uint32_t period_ms) {
  if (!dir_path) {
    ESP_LOGW(TAG, "Directory path is null");
    return;
  }

  // Reset frame count (no heap allocation/deallocation)
  anim_frame_count_ = 0;

  // Scan directory for image files
  DIR* dir = opendir(dir_path);
  if (!dir) {
    ESP_LOGE(TAG, "Failed to open directory: %s", dir_path);
    return;
  }

  struct dirent* entry;
  while ((entry = readdir(dir)) != NULL && anim_frame_count_ < kMaxAnimFrames) {
    if (entry->d_type == DT_REG) {
      // Build path string: LVGL_FS_PREFIX + dir_path + "/" + filename
      anim_frame_paths_[anim_frame_count_] =
          std::string(LVGL_FS_PREFIX) + dir_path + "/" + entry->d_name;
      anim_frame_count_++;
    }
  }

  // Check if we hit the limit while more frames remain
  if (entry != NULL) {  // readdir() didn't reach end
    // Count remaining frames for better diagnostics
    size_t remaining = 0;
    while ((entry = readdir(dir)) != NULL) {
      if (entry->d_type == DT_REG) remaining++;
    }
    ESP_LOGW(TAG,
             "Animation truncated: loaded %zu frames, %zu more frames ignored "
             "(max: %zu)",
             anim_frame_count_, remaining, kMaxAnimFrames);
  }

  closedir(dir);

  if (anim_frame_count_ == 0) {
    ESP_LOGW(TAG, "No image files found in directory: %s", dir_path);
    return;
  }

  // Sort file paths alphabetically (only the used portion)
  std::sort(anim_frame_paths_.begin(),
            anim_frame_paths_.begin() + anim_frame_count_);

  // Build pointer array for LVGL (pointers to our owned strings)
  for (size_t i = 0; i < anim_frame_count_; i++) {
    anim_frame_ptrs_[i] = anim_frame_paths_[i].c_str();
  }

  DisplayLock lock;
  if (!lock) {
    ESP_LOGE(TAG, "Display lock timeout in %s", __func__);
    return;
  }

  // Create animated image widget
  lv_obj_t* animimg = lv_animimg_create(lv_scr_act());
  if (!animimg) {
    ESP_LOGE(TAG, "Failed to create animated image object");
    return;  // Lock auto-released
  }

  lv_obj_center(animimg);
  lv_animimg_set_src(animimg, (const void**)anim_frame_ptrs_.data(),
                     anim_frame_count_);
  lv_animimg_set_duration(animimg, period_ms);
  lv_animimg_set_repeat_count(animimg, LV_ANIM_REPEAT_INFINITE);
  lv_animimg_start(animimg);

  current_content_ = animimg;

  ESP_LOGI(TAG, "Started animated image with %zu frames from %s",
           anim_frame_count_, dir_path);
}

void LVGLRenderer::ShowAdvertisingState() {
  // BLE advertising: show disconnected/advertising icon
  Clear();
  ShowIcon(IconType::DISCONNECTED);
}

void LVGLRenderer::ShowConnectedState() {
  // BLE connected: show connected icon
  Clear();
  ShowIcon(IconType::CONNECTED);
}

void LVGLRenderer::ShowBootState() {
  // Boot animation: show animated image sequence
  const char* directory = CONFIG_ANIMATION_IMAGE_DIR "/boot";
  ShowAnimatedImage(directory, 0, CONFIG_BOOT_ANIMATION_PLAY_DURATION);
}

void LVGLRenderer::ShowBootProgress(float progress) {
  // LVGL handles animation autonomously via lv_animimg
  // Just ensure boot animation is started on first call
  if (!boot_animation_started_) {
    ShowBootState();
    boot_animation_started_ = true;
  }
  // Ignore progress - LVGL timer handles frame advancement
  (void)progress;
}

void LVGLRenderer::ShowWifiConnecting(float progress) {
  // LVGL handles animation autonomously
  if (!wifi_animation_started_) {
    const char* directory = CONFIG_ANIMATION_IMAGE_DIR "/wifi_connecting";
    ShowAnimatedImage(directory, 0,
                      CONFIG_WIFI_CONNECTING_ANIMATION_PLAY_DURATION);
    wifi_animation_started_ = true;
  }
  (void)progress;
}

void LVGLRenderer::ShowOTAProgress(float progress, bool is_download_phase) {
  (void)is_download_phase;  // LVGL only shows progress bar
  uint8_t percent = static_cast<uint8_t>(progress * 100.0f);
  if (percent > 100) percent = 100;
  ShowPercentage(percent, ProgressStyle::NORMAL);
}

void LVGLRenderer::ShowPowerChart(const PowerDataPoint* data, size_t count) {
  if (!data || count == 0) {
    ESP_LOGW(TAG, "ShowPowerChart called with no data");
    return;
  }

#ifdef CONFIG_LV_USE_CHART
  DisplayLock lock;
  if (!lock) {
    ESP_LOGE(TAG, "Display lock timeout in %s", __func__);
    return;
  }

  // Create chart
  lv_obj_t* chart = lv_chart_create(lv_scr_act());
  lv_obj_set_size(chart, 200, 100);
  lv_obj_center(chart);

  // Configure chart
  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(chart, count);

  // Add series
  lv_chart_series_t* series = lv_chart_add_series(
      chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);

  // Populate data
  for (size_t i = 0; i < count; i++) {
    // Scale power to chart range (0-100)
    uint16_t value =
        (data[i].power_mw * display_renderer_utils::kMaxPercentage) /
        display_renderer_utils::kMaxPowerMw;
    if (value > display_renderer_utils::kMaxPercentage)
      value = display_renderer_utils::kMaxPercentage;
    lv_chart_set_next_value(chart, series, value);
  }

  // Refresh chart
  lv_chart_refresh(chart);

  current_content_ = chart;

#else
  // Fallback: show total power as text when chart widget unavailable
  ESP_LOGW(TAG, "Chart widget not available");
  uint32_t total_mw = 0;
  for (size_t i = 0; i < count; i++) {
    total_mw += data[i].power_mw;
  }
  TextStyle text_style;
  text_style.size = TextStyle::Size::LARGE;
  text_style.align = Alignment::CENTER;
  char buf[16];
  snprintf(buf, sizeof(buf), "%luW", total_mw / 1000);
  ShowText(buf, text_style);
#endif
}

void LVGLRenderer::ShowInfoScreen(const char* title, const char* content) {
  if (!title || !content) {
    ESP_LOGW(TAG, "ShowInfoScreen called with NULL parameters");
    return;
  }

  DisplayLock lock;
  if (!lock) {
    ESP_LOGE(TAG, "Display lock timeout in %s", __func__);
    return;
  }

  // Title label (top-left, small font)
  lv_obj_t* title_label = lv_label_create(lv_scr_act());
  lv_label_set_text(title_label, title);
  lv_obj_set_style_text_font(title_label, GetFont(FONT_KIND_SMALL), 0);
  lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 0, 10);

  // Content label (below title, larger font, multi-line)
  lv_obj_t* content_label = lv_label_create(lv_scr_act());
  lv_label_set_text(content_label, content);
  lv_obj_set_style_text_font(content_label, GetFont(FONT_KIND_MIDDLE), 0);
  lv_label_set_long_mode(content_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(content_label, LV_PCT(90));
  lv_obj_align(content_label, LV_ALIGN_TOP_LEFT, 0, 66);

  current_content_ = content_label;
}

void LVGLRenderer::ShowTestPattern(DisplayTestScreen screen) {
  DisplayLock lock;
  if (!lock) {
    ESP_LOGE(TAG, "Display lock timeout in %s", __func__);
    return;
  }

  lv_obj_t* scr = lv_scr_act();
  lv_obj_clean(scr);

  lv_font_t* font_large = GetFont(FONT_KIND_LARGE);

  switch (screen) {
    case DisplayTestScreen::TEST_SCREEN_RED:
      lv_obj_set_style_bg_color(scr, lv_palette_main(LV_PALETTE_RED), 0);
      break;
    case DisplayTestScreen::TEST_SCREEN_GREEN:
      lv_obj_set_style_bg_color(scr, lv_palette_main(LV_PALETTE_GREEN), 0);
      break;
    case DisplayTestScreen::TEST_SCREEN_BLUE:
      lv_obj_set_style_bg_color(scr, lv_palette_main(LV_PALETTE_BLUE), 0);
      break;
    case DisplayTestScreen::TEST_SCREEN_WHITE:
      lv_obj_set_style_bg_color(scr, lv_color_white(), 0);
      break;
    case DisplayTestScreen::TEST_SCREEN_BLACK:
      lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
      break;
    case DisplayTestScreen::TEST_SCREEN_GREY:
      lv_obj_set_style_bg_color(scr, lv_palette_main(LV_PALETTE_GREY), 0);
      break;
    case DisplayTestScreen::TEST_SCREEN_OFF:
      // Black screen - backlight control should be done via
      // GetDriver()->SetMode()
      lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
      break;
    case DisplayTestScreen::TEST_SCREEN_LOW_LIGHT: {
      lv_obj_set_style_bg_color(scr, lv_color_white(), 0);
      lv_obj_t* label = lv_label_create(scr);
      if (font_large) lv_obj_set_style_text_font(label, font_large, 0);
      lv_label_set_text(label, "LOW LIGHT");
      lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
      break;
    }
    case DisplayTestScreen::TEST_SCREEN_HIGH_LIGHT: {
      lv_obj_set_style_bg_color(scr, lv_color_white(), 0);
      lv_obj_t* label = lv_label_create(scr);
      if (font_large) lv_obj_set_style_text_font(label, font_large, 0);
      lv_label_set_text(label, "HIGH LIGHT");
      lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
      break;
    }
    case DisplayTestScreen::TEST_SCREEN_DEFAULT: {
      lv_obj_set_style_bg_color(scr, lv_color_white(), 0);
      lv_obj_t* label = lv_label_create(scr);
      if (font_large) lv_obj_set_style_text_font(label, font_large, 0);
      lv_label_set_text(label, "DISPLAY TEST");
      lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
      break;
    }
    case DisplayTestScreen::TEST_SCREEN_1: {
      lv_obj_set_style_bg_color(scr, lv_color_white(), 0);
      lv_obj_t* label = lv_label_create(scr);
      if (font_large) lv_obj_set_style_text_font(label, font_large, 0);
      lv_label_set_text(label, "TEST SCREEN 1");
      lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
      break;
    }
    case DisplayTestScreen::TEST_SCREEN_2: {
      lv_obj_set_style_bg_color(scr, lv_color_white(), 0);
      lv_obj_t* label = lv_label_create(scr);
      if (font_large) lv_obj_set_style_text_font(label, font_large, 0);
      lv_label_set_text(label, "TEST SCREEN 2");
      lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
      break;
    }
    case DisplayTestScreen::TEST_SCREEN_3: {
      lv_obj_set_style_bg_color(scr, lv_color_white(), 0);
      lv_obj_t* label = lv_label_create(scr);
      if (font_large) lv_obj_set_style_text_font(label, font_large, 0);
      lv_label_set_text(label, "TEST SCREEN 3");
      lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
      break;
    }
    default:
      lv_obj_set_style_bg_color(scr, lv_color_white(), 0);
      break;
  }
}

void LVGLRenderer::ShowChargingTotal(
    const std::vector<PortChargingInfo>& ports) {
  DisplayLock lock;
  if (!lock) {
    ESP_LOGE(TAG, "Display lock timeout in %s", __func__);
    return;
  }

  lv_obj_clean(lv_scr_act());

  lv_font_t* font_small = GetFont(FONT_KIND_SMALL);
  lv_font_t* font_large = GetFont(FONT_KIND_LARGE);

  // Calculate total power and extract port IDs
  uint16_t total_power_w = 0;
  std::string port_text;
  for (size_t i = 0; i < ports.size(); i++) {
    total_power_w += ports[i].power_w;

    // Build port list (e.g., "C1\nC2\nC3")
    port_text += "C" + std::to_string(ports[i].port_id + 1);
    if (i != ports.size() - 1) {
      port_text += "\n";
    }
  }

  // Create port labels
  lv_obj_t* port_label = lv_label_create(lv_scr_act());
  if (font_small) lv_obj_set_style_text_font(port_label, font_small, 0);
  lv_obj_set_style_text_color(port_label, lv_color_white(), 0);
  lv_label_set_text(port_label, port_text.c_str());

  // Position port label
  if (ports.size() == 1) {
    lv_obj_align(port_label, LV_ALIGN_TOP_LEFT, 0, 0);
  } else {
    lv_obj_align(port_label, LV_ALIGN_TOP_RIGHT, 0, 0);
  }

  // Create power value label
  lv_obj_t* power_label = lv_label_create(lv_scr_act());
  if (font_large) lv_obj_set_style_text_font(power_label, font_large, 0);
  lv_obj_set_style_text_color(power_label, lv_color_white(), 0);
  lv_label_set_text_fmt(power_label, "%d", total_power_w);
  lv_obj_align(power_label, LV_ALIGN_BOTTOM_LEFT, 0, 0);

  // Create "W" unit label
  lv_obj_t* w_label = lv_label_create(lv_scr_act());
  if (font_small) lv_obj_set_style_text_font(w_label, font_small, 0);
  lv_obj_set_style_text_color(w_label, lv_color_white(), 0);
  lv_obj_set_style_text_align(w_label, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_text(w_label, "W");
  lv_obj_align_to(w_label, power_label, LV_ALIGN_OUT_RIGHT_BOTTOM, 0, 0);

  // For single port: show detailed info (voltage/current/protocol)
  if (ports.size() == 1) {
    const PortChargingInfo& port_info = ports[0];
    lv_obj_t* details_label = lv_label_create(lv_scr_act());
    if (font_small) lv_obj_set_style_text_font(details_label, font_small, 0);
    lv_obj_set_style_text_color(details_label, lv_color_white(), 0);
    lv_obj_set_style_text_align(details_label, LV_TEXT_ALIGN_RIGHT, 0);

    const char* protocol = GetFCProtocolName(port_info.fc_protocol);
    lv_label_set_text_fmt(details_label, "%dA/%dV\n%s",
                          port_info.current_ma / 1000,
                          port_info.voltage_mv / 1000, protocol);
    lv_obj_align(details_label, LV_ALIGN_TOP_RIGHT, 0, 0);
  } else {
    // Multi-port: show "TOTAL:" label
    lv_obj_t* total_label = lv_label_create(lv_scr_act());
    if (font_small) lv_obj_set_style_text_font(total_label, font_small, 0);
    lv_obj_set_style_text_color(total_label, lv_color_white(), 0);
    lv_obj_set_style_text_align(total_label, LV_TEXT_ALIGN_RIGHT, 0);
    lv_label_set_text(total_label, "TOTAL:");
    lv_obj_align_to(total_label, power_label, LV_ALIGN_OUT_TOP_LEFT, 0, -8);
  }
}

void LVGLRenderer::ShowChargingPorts(const std::vector<PortChargingInfo>& ports,
                                     bool show_power) {
  if (ports.size() < 2) {
    ESP_LOGW(TAG, "ShowChargingPorts requires at least 2 ports");
    return;
  }

  DisplayLock lock;
  if (!lock) {
    ESP_LOGE(TAG, "Display lock timeout in %s", __func__);
    return;
  }

  lv_obj_clean(lv_scr_act());

  lv_font_t* font_small = GetFont(FONT_KIND_SMALL);
  lv_font_t* font_middle = GetFont(FONT_KIND_MIDDLE);

  // Calculate layout
  int offset = CONFIG_BSP_DISPLAY_HEIGHT / ports.size();

  // Create port info for each port
  for (size_t i = 0; i < ports.size(); i++) {
    const PortChargingInfo& port = ports[i];

    // Port label (e.g., "C1")
    lv_obj_t* port_label = lv_label_create(lv_scr_act());
    if (font_small) lv_obj_set_style_text_font(port_label, font_small, 0);
    lv_obj_set_style_text_color(port_label, lv_color_white(), 0);
    lv_label_set_text_fmt(port_label, "C%d", port.port_id + 1);
    lv_obj_align(port_label, LV_ALIGN_TOP_LEFT, 0, offset * i + 6);

    if (show_power) {
      // Show power in watts
      lv_obj_t* w_label = lv_label_create(lv_scr_act());
      if (font_small) lv_obj_set_style_text_font(w_label, font_small, 0);
      lv_obj_set_style_text_color(w_label, lv_color_white(), 0);
      lv_obj_set_style_text_align(w_label, LV_TEXT_ALIGN_RIGHT, 0);
      lv_label_set_text(w_label, "W");
      lv_obj_align(w_label, LV_ALIGN_TOP_RIGHT, 0, offset * i + 19);

      lv_obj_t* power_label = lv_label_create(lv_scr_act());
      if (font_middle) lv_obj_set_style_text_font(power_label, font_middle, 0);
      lv_obj_set_style_text_color(power_label, lv_color_white(), 0);
      lv_label_set_text_fmt(power_label, "%d", port.power_w);
      lv_obj_align_to(power_label, w_label, LV_ALIGN_OUT_LEFT_BOTTOM, 0, 0);
    } else {
      // Show voltage/current/protocol
      const char* protocol = GetFCProtocolName(port.fc_protocol);
      lv_obj_t* details_label = lv_label_create(lv_scr_act());
      if (font_small) lv_obj_set_style_text_font(details_label, font_small, 0);
      lv_obj_set_style_text_color(details_label, lv_color_white(), 0);
      lv_obj_set_style_text_align(details_label, LV_TEXT_ALIGN_RIGHT, 0);
      lv_label_set_text_fmt(details_label, "%dA/%dV\n%s",
                            port.current_ma / 1000, port.voltage_mv / 1000,
                            protocol);
      lv_obj_align(details_label, LV_ALIGN_TOP_RIGHT, 0, offset * i + 6);
    }
  }

  // Draw separator lines
  static lv_point_t line_points[4][2];  // Max 4 lines for 3 ports
  lv_style_t dashed_style;
  lv_style_init(&dashed_style);
  lv_style_set_line_color(&dashed_style, lv_color_white());
  lv_style_set_line_width(&dashed_style, 1);
  lv_style_set_line_dash_width(&dashed_style, 4);
  lv_style_set_line_dash_gap(&dashed_style, 4);

  if (ports.size() == 2) {
    // 3 lines for 2 ports (top, middle, bottom)
    line_points[0][0] = {0, 0};
    line_points[0][1] = {CONFIG_BSP_DISPLAY_WIDTH, 0};
    line_points[1][0] = {0, CONFIG_BSP_DISPLAY_HEIGHT / 2};
    line_points[1][1] = {CONFIG_BSP_DISPLAY_WIDTH,
                         CONFIG_BSP_DISPLAY_HEIGHT / 2};
    line_points[2][0] = {0, CONFIG_BSP_DISPLAY_HEIGHT};
    line_points[2][1] = {CONFIG_BSP_DISPLAY_WIDTH, CONFIG_BSP_DISPLAY_HEIGHT};

    for (int i = 0; i < 3; i++) {
      lv_obj_t* line = lv_line_create(lv_scr_act());
      lv_line_set_points(line, line_points[i], 2);
      lv_obj_add_style(line, &dashed_style, 0);
    }
  } else {
    // 2 lines for 3+ ports (1/3 and 2/3 positions)
    line_points[0][0] = {0, CONFIG_BSP_DISPLAY_HEIGHT / 3};
    line_points[0][1] = {CONFIG_BSP_DISPLAY_WIDTH,
                         CONFIG_BSP_DISPLAY_HEIGHT / 3};
    line_points[1][0] = {0, CONFIG_BSP_DISPLAY_HEIGHT / 3 * 2};
    line_points[1][1] = {CONFIG_BSP_DISPLAY_WIDTH,
                         CONFIG_BSP_DISPLAY_HEIGHT / 3 * 2};

    for (int i = 0; i < 2; i++) {
      lv_obj_t* line = lv_line_create(lv_scr_act());
      lv_line_set_points(line, line_points[i], 2);
      lv_obj_add_style(line, &dashed_style, 0);
    }
  }
}

WakeInterval LVGLRenderer::ShowChargingStatus(IAnimationDataSource* data_source,
                                              int64_t dt_us) {
  (void)dt_us;
  const auto& ports_data = data_source->GetPortsData();
  ShowChargingTotal(ports_data);
  return WakeInterval::Polling(2000000);  // 2s update interval
}

WakeInterval LVGLRenderer::ShowIdleState(IAnimationDataSource* data_source,
                                         int64_t dt_us) {
  (void)dt_us;
  if (!data_source) {
    return WakeInterval::CheckInterval();
  }

  bool wifi_connected = data_source->IsWifiConnected();
  uint8_t idle_type = data_source->GetIdleAnimationType();

  if (!wifi_connected) {
    if (!idle_showing_pin_) {
      Clear();
      const auto& pin = data_source->GetPin();
      char code[32];
      snprintf(code, sizeof(code), "PIN: %d%d%d%d", pin[0], pin[1], pin[2],
               pin[3]);
      TextStyle style;
      style.size = TextStyle::Size::LARGE;
      style.align = Alignment::CENTER;
      ShowText(code, style);
      idle_showing_pin_ = true;
    }

    idle_last_wifi_connected_ = wifi_connected;
    idle_last_type_ = idle_type;
    idle_last_power_tier_ = GetIdlePowerTier(data_source);
    return WakeInterval::CheckInterval();
  }

  if (idle_showing_pin_ || idle_last_type_ != idle_type ||
      idle_last_wifi_connected_ != wifi_connected) {
    idle_showing_pin_ = false;
    if (idle_type == pb_IdleAnimationType_IDLE_NONE) {
      driver_->SetMode(DisplayMode::OFF);
    } else if (idle_type == pb_IdleAnimationType_IDLE_METEOR) {
      static const char* kDirs[] = {"idle_slow", "idle_medium", "idle_fast"};
      uint8_t tier = GetIdlePowerTier(data_source);
      const char* dir = kDirs[(tier < 3) ? tier : 0];
      const char* directory = CONFIG_ANIMATION_IMAGE_DIR;
      std::string full_dir = std::string(directory) + "/" + dir;
      ShowAnimatedImage(full_dir.c_str(), 0,
                        kIdleMeteorFrameCount * kIdleMeteorPeriodMs);
      idle_last_power_tier_ = tier;
    } else {
      driver_->SetMode(DisplayMode::OFF);
    }
  } else if (idle_type == pb_IdleAnimationType_IDLE_METEOR) {
    uint8_t tier = GetIdlePowerTier(data_source);
    if (tier != idle_last_power_tier_) {
      static const char* kDirs[] = {"idle_slow", "idle_medium", "idle_fast"};
      const char* dir = kDirs[(tier < 3) ? tier : 0];
      const char* directory = CONFIG_ANIMATION_IMAGE_DIR;
      std::string full_dir = std::string(directory) + "/" + dir;
      ShowAnimatedImage(full_dir.c_str(), 0,
                        kIdleMeteorFrameCount * kIdleMeteorPeriodMs);
      idle_last_power_tier_ = tier;
    }
  }

  idle_last_wifi_connected_ = wifi_connected;
  idle_last_type_ = idle_type;
  return WakeInterval::CheckInterval();
}

IDisplayDriver* LVGLRenderer::GetDriver() { return driver_; }

// Helper methods

lv_font_t* LVGLRenderer::GetFontForSize(TextStyle::Size size) {
  switch (size) {
    case TextStyle::Size::SMALL:
      return GetFont(FONT_KIND_SMALL);
    case TextStyle::Size::MEDIUM:
      return GetFont(FONT_KIND_MIDDLE);
    case TextStyle::Size::LARGE:
      return GetFont(FONT_KIND_LARGE);
    default:
      return GetFont(FONT_KIND_MIDDLE);
  }
}

lv_align_t LVGLRenderer::GetLVGLAlign(Alignment align) {
  switch (align) {
    case Alignment::LEFT:
      return LV_ALIGN_LEFT_MID;
    case Alignment::CENTER:
      return LV_ALIGN_CENTER;
    case Alignment::RIGHT:
      return LV_ALIGN_RIGHT_MID;
    default:
      return LV_ALIGN_CENTER;
  }
}

lv_color_t LVGLRenderer::GetColorForStyle(ProgressStyle style) {
  switch (style) {
    case ProgressStyle::SUCCESS:
      return lv_palette_main(LV_PALETTE_GREEN);
    case ProgressStyle::WARNING:
      return lv_palette_main(LV_PALETTE_YELLOW);
    case ProgressStyle::ERROR:
      return lv_palette_main(LV_PALETTE_RED);
    case ProgressStyle::DIM:
      return lv_palette_main(LV_PALETTE_GREY);
    case ProgressStyle::NORMAL:
    default:
      return lv_palette_main(LV_PALETTE_BLUE);
  }
}

const char* LVGLRenderer::GetIconPath(IconType icon) {
  // Icon paths for LVGL asset storage
  switch (icon) {
    case IconType::CHARGING:
      return LVGL_FS_PREFIX "/icons/charging.bin";
    case IconType::IDLE:
      return LVGL_FS_PREFIX "/icons/idle.bin";
    case IconType::CONNECTED:
      return LVGL_FS_PREFIX "/icons/connected.bin";
    case IconType::DISCONNECTED:
      return LVGL_FS_PREFIX "/icons/disconnected.bin";
    case IconType::WARNING:
      return LVGL_FS_PREFIX "/icons/warning.bin";
    case IconType::ERROR:
      return LVGL_FS_PREFIX "/icons/error.bin";
    case IconType::CHECK:
      return LVGL_FS_PREFIX "/icons/check.bin";
    case IconType::POWER:
      return LVGL_FS_PREFIX "/icons/power.bin";
    default:
      return nullptr;
  }
}

// ============================================================================
// Resource Management
// ============================================================================

esp_err_t LVGLRenderer::InitFonts() {
  const char* font_files[] = {
      [FONT_KIND_SMALL] = LVGL_FS_PREFIX CONFIG_ANIMATION_FONT_DIR "/small.bin",
      [FONT_KIND_MIDDLE] =
          LVGL_FS_PREFIX CONFIG_ANIMATION_FONT_DIR "/middle.bin",
      [FONT_KIND_LARGE] = LVGL_FS_PREFIX CONFIG_ANIMATION_FONT_DIR "/large.bin",
  };

  for (size_t i = 0; i < sizeof(font_files) / sizeof(font_files[0]); i++) {
    fonts_[i] = lv_font_load(font_files[i]);
    if (fonts_[i] == NULL) {
      ESP_LOGE(TAG, "Failed to load font: %s", font_files[i]);
      goto ERROR;
    }
  }
  return ESP_OK;

ERROR:
  for (size_t i = 0; i < FONT_KIND_MAX; i++) {
    if (fonts_[i]) {
      lv_font_free(fonts_[i]);
      fonts_[i] = NULL;
    }
  }
  return ESP_FAIL;
}

esp_err_t LVGLRenderer::InitDecoder() {
  esp_lv_decoder_handle_t decoder_handle = NULL;
  esp_lv_decoder_init(&decoder_handle);
  return ESP_OK;
}

esp_err_t LVGLRenderer::InitScreenStyle() {
  lv_disp_set_bg_color(NULL, lv_color_black());
  lv_disp_set_bg_opa(NULL, LV_OPA_COVER);

  static lv_style_t style;
  lv_style_init(&style);
  lv_style_set_bg_color(&style, lv_color_black());
  lv_style_set_bg_opa(&style, LV_OPA_COVER);
  lv_obj_add_style(lv_scr_act(), &style, 0);
  lv_obj_clear_flag(lv_scr_act(), LV_OBJ_FLAG_SCROLLABLE);

  return ESP_OK;
}

lv_font_t* LVGLRenderer::GetFont(FontKind kind) {
  if (kind >= FONT_KIND_MAX) {
    return NULL;
  }
  return fonts_[kind];
}

// ============================================================================
// LVGLTestCapabilities Implementation
// ============================================================================

esp_err_t LVGLTestCapabilities::SetScreen(uint8_t screen) {
  auto& anim = DisplayTestAnimation::GetInstance();
  anim.auto_mode_ = false;
  anim.loop_once_ = false;
  anim.completed_ = false;
  anim.ShowTestPattern(static_cast<DisplayTestScreen>(screen));
  return ESP_OK;
}

esp_err_t LVGLTestCapabilities::SetAutoMode(uint8_t duration_secs,
                                            bool loop_once) {
  auto& anim = DisplayTestAnimation::GetInstance();

  // Check if already in the same mode with same settings
  if (anim.auto_mode_ && duration_secs * 1000 == anim.auto_duration_ms &&
      anim.loop_once_ == loop_once && !anim.completed_) {
    return ESP_OK;  // Already configured
  }

  anim.auto_mode_ = true;
  anim.loop_once_ = loop_once;
  anim.completed_ = false;
  anim.auto_duration_ms = duration_secs * 1000;
  anim.elapsed_us_ = 0;

  // Reset iterator and show first screen
  ResetTestSequence();
  anim.ShowTestPattern(GetNextTestScreen());

  // Wake task ONLY here - transitioning from manual to auto
  AnimationManager::GetInstance().NotifyUpdate();
  return ESP_OK;
}

// Static constexpr definition for ODR-use
constexpr DisplayTestScreen LVGLTestCapabilities::kSequence[];

DisplayTestScreen LVGLTestCapabilities::GetNextTestScreen() {
  DisplayTestScreen screen = kSequence[current_index_];
  current_index_ = (current_index_ + 1) % std::size(kSequence);
  return screen;
}

void LVGLTestCapabilities::ResetTestSequence() { current_index_ = 0; }

size_t LVGLTestCapabilities::GetTestScreenCount() const {
  return std::size(kSequence);
}

bool LVGLTestCapabilities::IsAtStart() const { return current_index_ == 0; }

#endif  // CONFIG_DISPLAY_DRIVER_LVGL
