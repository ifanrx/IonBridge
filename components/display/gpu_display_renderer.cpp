#include "gpu_display_renderer.h"

#ifdef CONFIG_DISPLAY_DRIVER_GPU

#include <cinttypes>
#include <cstring>

#include "animation_manager.h"
#include "data_types.pb.h"
#include "display_animation.h"
#include "display_types.h"
#include "esp_check.h"
#include "esp_log.h"
#include "gpu_data_types.h"
#include "sdkconfig.h"
#include "strategy.h"

static const char* TAG = "GPURenderer";

// Display dimensions
static const uint8_t PANEL_WIDTH = 96;
static const uint8_t PANEL_HEIGHT = 32;
static constexpr size_t FRAMEBUFFER_SIZE = (PANEL_WIDTH * PANEL_HEIGHT) / 8;

// Font dimensions indexed by GPUFontSizeIndex (SMALL=0, LARGE=1)
static constexpr uint8_t FONT_WIDTHS[] = {10,
                                          15};  // small: 10x16, large: 15x24
static constexpr uint8_t FONT_HEIGHTS[] = {16, 24};

// Idle animation timing (meteor frames)
static constexpr uint32_t kIdleMeteorFrameCount = 8;
static constexpr int64_t kIdleMeteorPeriodUs = 200 * 1000;

// Mode icon path prefix and filenames
static constexpr const char* kModeIconPrefix =
    CONFIG_ANIMATION_IMAGE_DIR "/icons/mode_";

static const char* GetIdlePowerBasedAnimDir(IAnimationDataSource* data_source) {
  constexpr uint8_t kMediumThreshold = 10;
  constexpr uint8_t kHighThreshold = 50;
  static const char* kDirs[] = {"idle_slow", "idle_medium", "idle_fast"};

  if (!data_source) {
    return kDirs[0];
  }

  uint16_t total = data_source->GetTotalWatts();

  if (total >= kHighThreshold) return kDirs[2];
  if (total >= kMediumThreshold) return kDirs[1];
  return kDirs[0];
}

GPURenderer::GPURenderer(GPUDisplayDriver* driver) : driver_(driver) {}

esp_err_t GPURenderer::Init() {
  ESP_LOGI(TAG, "Initializing GPU renderer");
  return ESP_OK;
}

esp_err_t GPURenderer::Deinit() {
  ESP_LOGI(TAG, "Deinitializing GPU renderer");
  return ESP_OK;
}

void GPURenderer::Clear() {
  // Reset animation state
  current_boot_frame_ = UINT32_MAX;
  current_wifi_frame_ = UINT32_MAX;
  current_greeting_frame_ = UINT32_MAX;
  current_advertising_frame_ = UINT32_MAX;
  current_connected_frame_ = UINT32_MAX;
  ota_first_call_ = true;
  idle_last_type_ = UINT8_MAX;
  idle_frame_ = 0;
  idle_frame_elapsed_us_ = 0;

  // Fill screen with black (0x00) - PRE_COMPOSITE_MODE_CLEAR doesn't work
  uint8_t black_buffer[FRAMEBUFFER_SIZE] = {0};
  UploadFramebuffer(black_buffer, PANEL_WIDTH, PANEL_HEIGHT, BITBLT_MODE_COPY,
                    false, true);
}

void GPURenderer::ShowText(const char* text, const TextStyle& style) {
  if (!text) {
    ESP_LOGW(TAG, "ShowText called with NULL text");
    return;
  }

  DrawTextRequest request = {};

  // Select font size
  GPUFontSizeIndex font_size = GetGPUFontSize(style.size);
  request.font_size = font_size;

  // Calculate text length (max 12 characters)
  size_t len = strlen(text);
  request.length = (len > 12) ? 12 : len;
  memcpy(request.text, text, request.length);

  // Set rotation (0° = horizontal left-to-right)
  request.rotation = GPU_TEXT_ROTATION_0;

  // Calculate position based on alignment
  request.x = CalculateXForAlign(text, font_size, style.align);
  request.y =
      (PANEL_HEIGHT - FONT_HEIGHTS[font_size]) / 2;  // Center vertically

  // Set compositing modes
  request.bitblt_mode = BITBLT_MODE_COPY;
  request.pre_composite_mode = PRE_COMPOSITE_MODE_CLEAR;
  request.post_composite_mode = POST_COMPOSITE_MODE_COMMIT;

  // Send command via driver
  esp_err_t ret = driver_->DrawText(request);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to draw text: %s", esp_err_to_name(ret));
  }
}

void GPURenderer::ShowPercentage(uint8_t percentage, ProgressStyle style) {
  if (percentage > display_renderer_utils::kMaxPercentage) {
    percentage = display_renderer_utils::kMaxPercentage;
  }

  // Use GPU's built-in vertical VU meter animation
  SetAnimationDataRequest data = {};
  data.data.port_power_usage[0] = percentage;

  // Set animation index
  driver_->SetAnimationIndex(GPU_ANIMATION_VERTICAL_POWER_VUMETER,
                             PRE_COMPOSITE_MODE_CLEAR,
                             POST_COMPOSITE_MODE_COMMIT);

  // Send animation data
  driver_->SetAnimationData(data);

  // Adjust hardware brightness
  uint8_t intensity = display_renderer_utils::GetIntensityForStyle(style);
  driver_->SetIntensity(intensity);
}

void GPURenderer::ShowIcon(IconType icon) {
  // Icons would need to be uploaded as framebuffer data
  // For now, show a simple representation using text or patterns
  switch (icon) {
    case IconType::CHARGING: {
      TextStyle style;
      style.size = TextStyle::Size::LARGE;
      style.align = Alignment::CENTER;
      ShowText("CHG", style);
      break;
    }
    case IconType::IDLE: {
      TextStyle style;
      style.size = TextStyle::Size::LARGE;
      style.align = Alignment::CENTER;
      ShowText("IDLE", style);
      break;
    }
    case IconType::ERROR: {
      TextStyle style;
      style.size = TextStyle::Size::LARGE;
      style.align = Alignment::CENTER;
      ShowText("ERR", style);
      break;
    }
    case IconType::WARNING: {
      TextStyle style;
      style.size = TextStyle::Size::LARGE;
      style.align = Alignment::CENTER;
      ShowText("WARN", style);
      break;
    }
    default:
      ESP_LOGW(TAG, "Icon type %d not fully implemented", (int)icon);
      Clear();
      break;
  }
}

void GPURenderer::ShowImage(const char* path, Alignment align) {
  if (!path) {
    ESP_LOGW(TAG, "ShowImage called with NULL path");
    return;
  }

  // Switch GPU to framebuffer mode
  driver_->SetAnimationIndex(GPU_ANIMATION_FRAMEBUFFER,
                             PRE_COMPOSITE_MODE_CLEAR,
                             POST_COMPOSITE_MODE_WAIT);

  // Load and upload image (384-byte raw bitmap)
  esp_err_t ret = LoadAndShowImage(path, BITBLT_MODE_COPY, true, true);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to show image: %s (%s)", path, esp_err_to_name(ret));
    // Show placeholder on error
    TextStyle style;
    style.size = TextStyle::Size::SMALL;
    style.align = align;
    ShowText("IMG?", style);
  }
}

void GPURenderer::ShowAnimatedImage(const char* dir_path, uint32_t frame_count,
                                    uint32_t period_ms) {
  if (!dir_path) {
    ESP_LOGW(TAG, "ShowAnimatedImage: invalid parameters");
    return;
  }

  (void)frame_count;  // No longer used for autonomous playback
  (void)period_ms;    // Animation timing now controlled by caller

  // Switch GPU to framebuffer mode
  driver_->SetAnimationIndex(GPU_ANIMATION_FRAMEBUFFER,
                             PRE_COMPOSITE_MODE_CLEAR,
                             POST_COMPOSITE_MODE_WAIT);

  // Load and show first frame only (caller will drive frame advancement)
  char frame_path[64];
  snprintf(frame_path, sizeof(frame_path), "%s/frame_0.bin", dir_path);

  uint8_t frame_buffer[384];
  esp_err_t ret = LoadRawBitmap(frame_path, frame_buffer, sizeof(frame_buffer));
  if (ret == ESP_OK) {
    ret = UploadFramebuffer(frame_buffer, PANEL_WIDTH, PANEL_HEIGHT,
                            BITBLT_MODE_COPY, false, true);
  }

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to load first frame from %s", dir_path);
    TextStyle style;
    style.size = TextStyle::Size::SMALL;
    style.align = Alignment::CENTER;
    ShowText("ANIM?", style);
  }
}

void GPURenderer::ShowAdvertisingState() {
  // BLE advertising: show disconnected/advertising icon
  Clear();
  // ShowIcon(IconType::DISCONNECTED);
}

void GPURenderer::ShowConnectedState() {
  // BLE connected: show connected icon
  Clear();
  // ShowIcon(IconType::CONNECTED);
}

void GPURenderer::ShowBootState() {
  // Legacy: delegates to ShowBootProgress for initial state
  ShowBootProgress(0.0f);
}

GOLDensityLevel GPURenderer::GetPowerBasedDensityLevel() {
  constexpr uint16_t kMediumThreshold = 10;  // 10W
  constexpr uint16_t kHighThreshold = 50;    // 50W

  uint16_t total = data_source_.GetTotalWatts();

  if (total >= kHighThreshold) return GOL_DENSITY_HIGH;
  if (total >= kMediumThreshold) return GOL_DENSITY_MEDIUM;
  return GOL_DENSITY_LOW;
}

void GPURenderer::ShowBootProgress(float progress) {
  const char* directory = CONFIG_ANIMATION_IMAGE_DIR "/boot";

  // Count frames on first call (UINT32_MAX means already checked, no frames)
  if (boot_frame_count_ == 0) {
    // Count frames by trying to open them
    for (uint32_t i = 0; i < 29; i++) {
      char frame_path[64];
      snprintf(frame_path, sizeof(frame_path), "%s/frame_%" PRIu32 ".bin",
               directory, i);
      FILE* f = fopen(frame_path, "rb");
      if (!f) {
        if (i == 0) {
          ESP_LOGW(TAG, "Failed to open %s: errno=%d (%s)", frame_path, errno,
                   strerror(errno));
        }
        break;  // boot_frame_count_ already set from previous iteration
      }
      fclose(f);
      boot_frame_count_ = i + 1;  // Update count on each successful open
    }

    if (boot_frame_count_ == 0) {
      ESP_LOGW(TAG, "No boot animation frames found in %s", directory);
      boot_frame_count_ = UINT32_MAX;  // Mark as checked
      return;
    }

    // Pre-cache frames if small enough
    if (boot_frame_count_ <= kMaxCachedFrames) {
      ESP_LOGI(TAG, "Pre-caching %" PRIu32 " boot frames", boot_frame_count_);
      bool cache_success = true;
      for (uint32_t i = 0; i < boot_frame_count_; i++) {
        char frame_path[64];
        snprintf(frame_path, sizeof(frame_path), "%s/frame_%" PRIu32 ".bin",
                 directory, i);
        if (LoadRawBitmap(frame_path, frame_cache_[i],
                          sizeof(frame_cache_[i])) != ESP_OK) {
          cache_success = false;
          break;
        }
      }
      frames_cached_ = cache_success;
    }

    // Switch GPU to framebuffer mode
    driver_->SetAnimationIndex(GPU_ANIMATION_FRAMEBUFFER,
                               PRE_COMPOSITE_MODE_CLEAR,
                               POST_COMPOSITE_MODE_WAIT);
  }

  // No frames available
  if (boot_frame_count_ == UINT32_MAX) return;

  // Calculate frame from progress
  uint32_t frame =
      static_cast<uint32_t>(progress * static_cast<float>(boot_frame_count_));
  if (frame >= boot_frame_count_) frame = boot_frame_count_ - 1;

  // Skip if same frame
  if (frame == current_boot_frame_) return;
  current_boot_frame_ = frame;

  // Upload frame
  esp_err_t ret;
  if (frames_cached_) {
    ret = UploadFramebuffer(frame_cache_[frame], PANEL_WIDTH, PANEL_HEIGHT,
                            BITBLT_MODE_COPY, false, true);
  } else {
    char frame_path[64];
    snprintf(frame_path, sizeof(frame_path), "%s/frame_%" PRIu32 ".bin",
             directory, frame);
    uint8_t frame_buffer[384];
    ret = LoadRawBitmap(frame_path, frame_buffer, sizeof(frame_buffer));
    if (ret == ESP_OK) {
      ret = UploadFramebuffer(frame_buffer, PANEL_WIDTH, PANEL_HEIGHT,
                              BITBLT_MODE_COPY, false, true);
    }
  }

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to show boot frame %" PRIu32, frame);
  }
}

void GPURenderer::ShowWifiConnecting(float progress) {
  const char* directory = CONFIG_ANIMATION_IMAGE_DIR "/wifi_connecting";

  // Count frames on first call
  if (wifi_frame_count_ == 0) {
    for (uint32_t i = 0; i < 8; i++) {
      char frame_path[64];
      snprintf(frame_path, sizeof(frame_path), "%s/frame_%" PRIu32 ".bin",
               directory, i);
      FILE* f = fopen(frame_path, "rb");
      if (!f) {
        wifi_frame_count_ = i;
        break;
      }
      fclose(f);
    }

    if (wifi_frame_count_ == 0) {
      ESP_LOGW(TAG, "No wifi animation frames found in %s", directory);
      wifi_frame_count_ = UINT32_MAX;  // Mark as checked
    } else {
      // Switch GPU to framebuffer mode
      driver_->SetAnimationIndex(GPU_ANIMATION_FRAMEBUFFER,
                                 PRE_COMPOSITE_MODE_CLEAR,
                                 POST_COMPOSITE_MODE_WAIT);
    }
  }

  // No frames - skip rendering
  if (wifi_frame_count_ == UINT32_MAX) {
    return;
  }

  // Calculate frame from progress (looping)
  uint32_t frame =
      static_cast<uint32_t>(progress * static_cast<float>(wifi_frame_count_)) %
      wifi_frame_count_;

  // Skip if same frame
  if (frame == current_wifi_frame_) return;
  current_wifi_frame_ = frame;

  // Load and upload frame
  char frame_path[64];
  snprintf(frame_path, sizeof(frame_path), "%s/frame_%" PRIu32 ".bin",
           directory, frame);

  uint8_t frame_buffer[384];
  esp_err_t ret = LoadRawBitmap(frame_path, frame_buffer, sizeof(frame_buffer));
  if (ret == ESP_OK) {
    ret = UploadFramebuffer(frame_buffer, PANEL_WIDTH, PANEL_HEIGHT,
                            BITBLT_MODE_COPY, false, true);
  }

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to show wifi frame %" PRIu32, frame);
  }
}

void GPURenderer::ShowGreetingProgress(float progress) {
  const char* directory = CONFIG_ANIMATION_IMAGE_DIR "/greeting";

  // Count frames on first call
  if (greeting_frame_count_ == 0) {
    for (uint32_t i = 0; i < 13; i++) {
      char frame_path[64];
      snprintf(frame_path, sizeof(frame_path), "%s/frame_%" PRIu32 ".bin",
               directory, i);
      FILE* f = fopen(frame_path, "rb");
      if (!f) {
        greeting_frame_count_ = i;
        break;
      }
      fclose(f);
    }

    if (greeting_frame_count_ == 0) {
      ESP_LOGW(TAG, "No greeting frames found in %s", directory);
      greeting_frame_count_ = UINT32_MAX;  // Mark as checked
    }

    driver_->SetAnimationIndex(GPU_ANIMATION_FRAMEBUFFER,
                               PRE_COMPOSITE_MODE_CLEAR,
                               POST_COMPOSITE_MODE_WAIT);
  }

  if (greeting_frame_count_ == UINT32_MAX) {
    return;
  }

  uint32_t frame = static_cast<uint32_t>(
                       progress * static_cast<float>(greeting_frame_count_)) %
                   greeting_frame_count_;

  if (frame == current_greeting_frame_) return;
  current_greeting_frame_ = frame;

  char frame_path[64];
  snprintf(frame_path, sizeof(frame_path), "%s/frame_%" PRIu32 ".bin",
           directory, frame);

  uint8_t frame_buffer[384];
  esp_err_t ret = LoadRawBitmap(frame_path, frame_buffer, sizeof(frame_buffer));
  if (ret == ESP_OK) {
    ret = UploadFramebuffer(frame_buffer, PANEL_WIDTH, PANEL_HEIGHT,
                            BITBLT_MODE_COPY, false, true);
  }

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to show greeting frame %" PRIu32, frame);
  }
}

void GPURenderer::ShowAdvertisingProgress(float progress) {
  const char* directory = CONFIG_ANIMATION_IMAGE_DIR "/advertising";

  // advertising animation (fallback limit)
  if (advertising_frame_count_ == 0) {
    for (uint32_t i = 0; i < 13; i++) {
      char frame_path[64];
      snprintf(frame_path, sizeof(frame_path), "%s/frame_%" PRIu32 ".bin",
               directory, i);
      FILE* f = fopen(frame_path, "rb");
      if (!f) {
        advertising_frame_count_ = i;
        break;
      }
      fclose(f);
    }

    if (advertising_frame_count_ == 0) {
      ESP_LOGW(TAG, "No advertising frames found in %s", directory);
      advertising_frame_count_ = UINT32_MAX;  // Mark as checked
    }

    driver_->SetAnimationIndex(GPU_ANIMATION_FRAMEBUFFER,
                               PRE_COMPOSITE_MODE_CLEAR,
                               POST_COMPOSITE_MODE_WAIT);
  }

  if (advertising_frame_count_ == UINT32_MAX) {
    return;  // No animation resources, don't display anything
  }

  uint32_t frame =
      static_cast<uint32_t>(progress *
                            static_cast<float>(advertising_frame_count_)) %
      advertising_frame_count_;

  if (frame == current_advertising_frame_) return;
  current_advertising_frame_ = frame;

  char frame_path[64];
  snprintf(frame_path, sizeof(frame_path), "%s/frame_%" PRIu32 ".bin",
           directory, frame);

  uint8_t frame_buffer[384];
  esp_err_t ret = LoadRawBitmap(frame_path, frame_buffer, sizeof(frame_buffer));
  if (ret == ESP_OK) {
    ret = UploadFramebuffer(frame_buffer, PANEL_WIDTH, PANEL_HEIGHT,
                            BITBLT_MODE_COPY, false, true);
  }

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to show advertising frame %" PRIu32, frame);
  }
}

void GPURenderer::ShowConnectedProgress(float progress) {
  const char* directory = CONFIG_ANIMATION_IMAGE_DIR "/connected";

  // connected animation (fallback limit)
  if (connected_frame_count_ == 0) {
    for (uint32_t i = 0; i < 13; i++) {
      char frame_path[64];
      snprintf(frame_path, sizeof(frame_path), "%s/frame_%" PRIu32 ".bin",
               directory, i);
      FILE* f = fopen(frame_path, "rb");
      if (!f) {
        connected_frame_count_ = i;
        break;
      }
      fclose(f);
    }

    if (connected_frame_count_ == 0) {
      ESP_LOGW(TAG, "No connected frames found in %s", directory);
      connected_frame_count_ = UINT32_MAX;  // Mark as checked
    }

    driver_->SetAnimationIndex(GPU_ANIMATION_FRAMEBUFFER,
                               PRE_COMPOSITE_MODE_CLEAR,
                               POST_COMPOSITE_MODE_WAIT);
  }

  if (connected_frame_count_ == UINT32_MAX) {
    return;  // No animation resources, don't display anything
  }

  uint32_t frame = static_cast<uint32_t>(
                       progress * static_cast<float>(connected_frame_count_)) %
                   connected_frame_count_;

  if (frame == current_connected_frame_) return;
  current_connected_frame_ = frame;

  char frame_path[64];
  snprintf(frame_path, sizeof(frame_path), "%s/frame_%" PRIu32 ".bin",
           directory, frame);

  uint8_t frame_buffer[384];
  esp_err_t ret = LoadRawBitmap(frame_path, frame_buffer, sizeof(frame_buffer));
  if (ret == ESP_OK) {
    ret = UploadFramebuffer(frame_buffer, PANEL_WIDTH, PANEL_HEIGHT,
                            BITBLT_MODE_COPY, false, true);
  }

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to show connected frame %" PRIu32, frame);
  }
}

void GPURenderer::ShowOTAProgress(float progress, bool is_download_phase) {
  (void)progress;  // GPU shows static image, not progress bar

  // Only reload image when phase changes or on first call
  if (!ota_first_call_ && ota_last_phase_ == is_download_phase) {
    return;  // Same phase, no need to reload
  }
  ota_first_call_ = false;
  ota_last_phase_ = is_download_phase;

  // Show appropriate OTA image based on phase
  const char* image_path =
      is_download_phase ? IMAGE_PATH("ota/download") : IMAGE_PATH("ota/update");

  ESP_LOGI(TAG, "OTA phase: %s, showing image: %s",
           is_download_phase ? "download" : "update", image_path);

  // Switch GPU to framebuffer mode and show image
  driver_->SetAnimationIndex(GPU_ANIMATION_FRAMEBUFFER,
                             PRE_COMPOSITE_MODE_CLEAR,
                             POST_COMPOSITE_MODE_WAIT);

  esp_err_t ret = LoadAndShowImage(image_path, BITBLT_MODE_COPY, true, true);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to show OTA image: %s", image_path);
    // Fallback to text
    TextStyle style;
    style.size = TextStyle::Size::SMALL;
    style.align = Alignment::CENTER;
    ShowText(is_download_phase ? "OTA DL" : "OTA UPD", style);
  }
}

void GPURenderer::ShowPowerChart(const PowerDataPoint* data, size_t count) {
  if (!data || count == 0) {
    ESP_LOGW(TAG, "ShowPowerChart called with no data");
    return;
  }

  // Use horizontal VU meter to show power history
  // Map most recent data points to VU meter levels
  SetAnimationDataRequest anim_data = {};

  // Take up to 8 most recent data points
  size_t start_idx = (count > 8) ? (count - 8) : 0;
  for (size_t i = 0; i < 8 && (start_idx + i) < count; i++) {
    // Scale power to 0-100%
    uint32_t power_mw = data[start_idx + i].power_mw;
    uint8_t percentage = (power_mw * display_renderer_utils::kMaxPercentage) /
                         display_renderer_utils::kMaxPowerMw;
    if (percentage > display_renderer_utils::kMaxPercentage)
      percentage = display_renderer_utils::kMaxPercentage;

    anim_data.data.port_power_usage[i] = percentage;
  }

  // Use horizontal VU meter per port to show history
  driver_->SetAnimationIndex(GPU_ANIMATION_HORIZONTAL_POWER_VUMETER_PER_PORT,
                             PRE_COMPOSITE_MODE_CLEAR,
                             POST_COMPOSITE_MODE_COMMIT);

  driver_->SetAnimationData(anim_data);
}

void GPURenderer::ShowInfoScreen(const char* title, const char* content) {
  if (!title || !content) {
    ESP_LOGW(TAG, "ShowInfoScreen called with NULL parameters");
    return;
  }

  // Clear screen first
  driver_->SetAnimationIndex(GPU_ANIMATION_FRAMEBUFFER,
                             PRE_COMPOSITE_MODE_CLEAR,
                             POST_COMPOSITE_MODE_WAIT);

  // Draw title at top (small font)
  DrawTextRequest title_req = {};
  title_req.x = 0;
  title_req.y = 0;
  title_req.rotation = GPU_TEXT_ROTATION_0;
  title_req.font_size = GPU_FONT_SIZE_SMALL;
  title_req.length = strlen(title) > 12 ? 12 : strlen(title);
  memcpy(title_req.text, title, title_req.length);
  title_req.bitblt_mode = BITBLT_MODE_COPY;
  title_req.pre_composite_mode = PRE_COMPOSITE_MODE_KEEP;
  title_req.post_composite_mode = POST_COMPOSITE_MODE_WAIT;

  driver_->DrawText(title_req);

  // Draw content below title (multi-line support via newline split)
  DrawTextRequest content_req = {};
  content_req.x = 0;
  content_req.y = 16;  // Below title
  content_req.font_size = GPU_FONT_SIZE_SMALL;
  content_req.rotation = GPU_TEXT_ROTATION_0;
  content_req.length = strlen(content) > 12 ? 12 : strlen(content);
  memcpy(content_req.text, content, content_req.length);
  content_req.bitblt_mode = BITBLT_MODE_COPY;
  content_req.pre_composite_mode = PRE_COMPOSITE_MODE_KEEP;
  content_req.post_composite_mode = POST_COMPOSITE_MODE_COMMIT;

  driver_->DrawText(content_req);
}

void GPURenderer::ShowTestPattern(DisplayTestScreen screen) {
  uint8_t buffer[FRAMEBUFFER_SIZE];

  // Handle solid fill screens
  if (screen == DisplayTestScreen::TEST_SCREEN_WHITE) {
    memset(buffer, 0xFF, FRAMEBUFFER_SIZE);
    UploadFramebuffer(buffer, PANEL_WIDTH, PANEL_HEIGHT, BITBLT_MODE_COPY,
                      false, true);
    return;
  }
  if (screen == DisplayTestScreen::TEST_SCREEN_BLACK) {
    memset(buffer, 0x00, FRAMEBUFFER_SIZE);
    UploadFramebuffer(buffer, PANEL_WIDTH, PANEL_HEIGHT, BITBLT_MODE_COPY,
                      false, true);
    return;
  }
  if (screen == DisplayTestScreen::TEST_SCREEN_OFF) {
    // Display will be turned off via SetMode(OFF) in animation layer
    return;
  }

  // Clear screen for text-based screens
  memset(buffer, 0x00, FRAMEBUFFER_SIZE);
  UploadFramebuffer(buffer, PANEL_WIDTH, PANEL_HEIGHT, BITBLT_MODE_COPY, false,
                    true);

  // Map test screen to text
  const char* text = "";
  switch (screen) {
    case DisplayTestScreen::TEST_SCREEN_LOW_LIGHT:
      text = "LOW";
      break;
    case DisplayTestScreen::TEST_SCREEN_HIGH_LIGHT:
      text = "HIGH";
      break;
    case DisplayTestScreen::TEST_SCREEN_DEFAULT:
      text = "TEST";
      break;
    default:
      return;
  }

  // Draw text centered
  DrawTextRequest req = {};
  req.x = 24;
  req.y = 10;
  req.rotation = GPU_TEXT_ROTATION_0;
  req.font_size = GPU_FONT_SIZE_SMALL;
  req.length = strlen(text);
  memcpy(req.text, text, req.length);
  req.bitblt_mode = BITBLT_MODE_COPY;
  req.pre_composite_mode = PRE_COMPOSITE_MODE_KEEP;
  req.post_composite_mode = POST_COMPOSITE_MODE_COMMIT;

  driver_->DrawText(req);
}

// Layout definitions for ShowChargingTotal
static constexpr ChargingTotalLayout kPortraitLayout = {
    .rotation = GPU_TEXT_ROTATION_270,
    .label = {7, 0, GPU_FONT_SIZE_SMALL},      // Top-right corner
    .ones = {72, 0, GPU_FONT_SIZE_LARGE},      // Bottom, flush right
    .tens = {72, 17, GPU_FONT_SIZE_LARGE},     // Above ones, 2px gap
    .hundreds = {42, 17, GPU_FONT_SIZE_LARGE}  // Stacked above tens, 6px gap
};

static constexpr ChargingTotalLayout kLandscapeLayout = {
    .rotation = GPU_TEXT_ROTATION_0,
    .label = {0, 0, GPU_FONT_SIZE_SMALL},    // TODO: adjust for landscape
    .ones = {0, 0, GPU_FONT_SIZE_LARGE},     // TODO: adjust for landscape
    .tens = {0, 0, GPU_FONT_SIZE_LARGE},     // TODO: adjust for landscape
    .hundreds = {0, 0, GPU_FONT_SIZE_LARGE}  // TODO: adjust for landscape
};

WakeInterval GPURenderer::ShowChargingStatus(IAnimationDataSource* data_source,
                                             int64_t dt_us) {
  (void)dt_us;
  const auto& ports_data = data_source->GetPortsData();
  ShowChargingTotal(ports_data);
  return WakeInterval::Polling(2000000);  // 2s update interval
}

WakeInterval GPURenderer::ShowIdleState(IAnimationDataSource* data_source,
                                        int64_t dt_us) {
  if (!data_source) {
    return WakeInterval::CheckInterval();
  }
  uint8_t idle_type = data_source->GetIdleAnimationType();

  if (idle_type != idle_last_type_) {
    idle_last_type_ = idle_type;
    idle_frame_ = 0;
    idle_frame_elapsed_us_ = 0;

    if (idle_type == pb_IdleAnimationType_IDLE_NONE) {
      driver_->SetMode(DisplayMode::OFF);
      return WakeInterval::CheckInterval();
    }

    if (idle_type == pb_IdleAnimationType_IDLE_METEOR) {
      driver_->SetMode(DisplayMode::MANUAL);
      char path[80];
      snprintf(path, sizeof(path), CONFIG_ANIMATION_IMAGE_DIR "/%s/frame_0.bin",
               GetIdlePowerBasedAnimDir(data_source));
      ShowImage(path, Alignment::CENTER);
      return WakeInterval::Polling(kIdleMeteorPeriodUs);
    }

    driver_->SetAnimationIndex(GPU_ANIMATION_GAME_OF_LIFE,
                               PRE_COMPOSITE_MODE_CLEAR,
                               POST_COMPOSITE_MODE_COMMIT);

    SetAnimationDataRequest data = {};
    data.data.gol_density = GetPowerBasedDensityLevel();
    driver_->SetAnimationData(data);
    return WakeInterval::CheckInterval();
  }

  if (idle_type == pb_IdleAnimationType_IDLE_METEOR) {
    idle_frame_elapsed_us_ += dt_us;
    if (idle_frame_elapsed_us_ >= kIdleMeteorPeriodUs) {
      uint32_t advance =
          static_cast<uint32_t>(idle_frame_elapsed_us_ / kIdleMeteorPeriodUs);
      idle_frame_ = (idle_frame_ + advance) % kIdleMeteorFrameCount;
      idle_frame_elapsed_us_ -= advance * kIdleMeteorPeriodUs;

      char path[80];
      snprintf(path, sizeof(path),
               CONFIG_ANIMATION_IMAGE_DIR "/%s/frame_%lu.bin",
               GetIdlePowerBasedAnimDir(data_source),
               static_cast<unsigned long>(idle_frame_));
      ShowImage(path, Alignment::CENTER);
    }
    int64_t remaining = kIdleMeteorPeriodUs - idle_frame_elapsed_us_;
    return WakeInterval::Polling(remaining > 0 ? remaining
                                               : kIdleMeteorPeriodUs);
  }

  return WakeInterval::CheckInterval();
}

void GPURenderer::ShowChargingTotal(
    const std::vector<PortChargingInfo>& ports) {
  // Select layout based on screen orientation (currently portrait only)
  const auto& layout = kPortraitLayout;

  // Calculate total power (ceiling to watts)
  uint32_t total_mw = 0;
  for (const auto& port : ports) {
    total_mw += static_cast<uint32_t>(port.voltage_mv) * port.current_ma / 1000;
  }
  uint16_t watts = static_cast<uint16_t>((total_mw + 999) / 1000);

  // Helper to draw a character at a given position
  auto drawChar = [this, &layout](const CharPosition& pos, char ch,
                                  PreCompositeMode pre,
                                  PostCompositeMode post) {
    DrawTextRequest req = {};
    req.x = pos.x;
    req.y = pos.y;
    req.rotation = layout.rotation;
    req.font_size = pos.font;
    req.length = 1;
    req.text[0] = ch;
    req.bitblt_mode = BITBLT_MODE_OR;
    req.pre_composite_mode = pre;
    req.post_composite_mode = post;
    driver_->DrawText(req);
  };

  // Determine digit count and last character position
  size_t digit_count = (watts >= 100) ? 3 : (watts >= 10) ? 2 : 1;
  bool has_tens = (digit_count >= 2);
  bool has_hundreds = (digit_count == 3);

  // Draw "T" label (clears screen)
  drawChar(layout.label, 'T', PRE_COMPOSITE_MODE_CLEAR,
           POST_COMPOSITE_MODE_WAIT);

  // Draw ones digit
  drawChar(layout.ones, '0' + (watts % 10), PRE_COMPOSITE_MODE_KEEP,
           (!has_tens) ? POST_COMPOSITE_MODE_COMMIT : POST_COMPOSITE_MODE_WAIT);

  // Draw tens digit
  if (has_tens) {
    drawChar(layout.tens, '0' + ((watts / 10) % 10), PRE_COMPOSITE_MODE_KEEP,
             (!has_hundreds) ? POST_COMPOSITE_MODE_COMMIT
                             : POST_COMPOSITE_MODE_WAIT);
  }

  // Draw hundreds digit (stacked above tens)
  if (has_hundreds) {
    drawChar(layout.hundreds, '1', PRE_COMPOSITE_MODE_KEEP,
             POST_COMPOSITE_MODE_COMMIT);
  }

  driver_->SetIntensity(display_renderer_utils::kMaxPercentage);
  ESP_LOGD(TAG, "ShowChargingTotal: %zu ports, %dW", ports.size(), watts);
}

IDisplayDriver* GPURenderer::GetDriver() { return driver_; }

// Helper methods

GPUFontSizeIndex GPURenderer::GetGPUFontSize(TextStyle::Size size) {
  switch (size) {
    case TextStyle::Size::SMALL:
      return GPU_FONT_SIZE_SMALL;
    case TextStyle::Size::LARGE:
      return GPU_FONT_SIZE_LARGE;
    case TextStyle::Size::MEDIUM:
    default:
      return GPU_FONT_SIZE_SMALL;  // Default to smaller font
  }
}

uint8_t GPURenderer::CalculateXForAlign(const char* text,
                                        GPUFontSizeIndex font_size,
                                        Alignment align) {
  if (!text) return 0;

  size_t len = strlen(text);
  if (len > 12) len = 12;

  // Calculate text width using font dimensions
  uint8_t char_width = FONT_WIDTHS[font_size];
  uint8_t text_width = len * char_width;

  switch (align) {
    case Alignment::LEFT:
      return 0;
    case Alignment::CENTER:
      return (PANEL_WIDTH - text_width) / 2;
    case Alignment::RIGHT:
      return PANEL_WIDTH - text_width;
    default:
      return 0;
  }
}

// Image loading helpers (moved from image_utils)

esp_err_t GPURenderer::LoadRawBitmap(const char* path, uint8_t* buffer,
                                     size_t buffer_size) {
  ESP_RETURN_ON_FALSE(path != nullptr && buffer != nullptr, ESP_ERR_INVALID_ARG,
                      TAG, "Invalid arguments");
  ESP_RETURN_ON_FALSE(buffer_size >= FRAMEBUFFER_SIZE, ESP_ERR_INVALID_SIZE,
                      TAG, "Buffer too small (need %d bytes)",
                      FRAMEBUFFER_SIZE);

  FILE* file = fopen(path, "rb");
  ESP_RETURN_ON_FALSE(file != nullptr, ESP_ERR_NOT_FOUND, TAG,
                      "Failed to open file: %s", path);

  fseek(file, 0, SEEK_END);
  size_t file_size = ftell(file);
  fseek(file, 0, SEEK_SET);

  if (file_size != FRAMEBUFFER_SIZE) {
    ESP_LOGW(TAG, "File size mismatch: %zu bytes (expected %d)", file_size,
             FRAMEBUFFER_SIZE);
    fclose(file);
    return ESP_ERR_INVALID_SIZE;
  }

  size_t bytes_read = fread(buffer, 1, FRAMEBUFFER_SIZE, file);
  fclose(file);

  ESP_RETURN_ON_FALSE(bytes_read == FRAMEBUFFER_SIZE, ESP_FAIL, TAG,
                      "Read only %zu/%d bytes", bytes_read, FRAMEBUFFER_SIZE);

  ESP_LOGD(TAG, "Loaded bitmap from %s (%d bytes)", path, FRAMEBUFFER_SIZE);
  return ESP_OK;
}

esp_err_t GPURenderer::UploadFramebuffer(const uint8_t* bitmap, uint8_t width,
                                         uint8_t height, BitBLTMode bitblt_mode,
                                         bool clear_before, bool commit_after) {
  ESP_RETURN_ON_FALSE(bitmap != nullptr, ESP_ERR_INVALID_ARG, TAG,
                      "Bitmap is null");
  ESP_RETURN_ON_FALSE(width == PANEL_WIDTH && height == PANEL_HEIGHT,
                      ESP_ERR_INVALID_SIZE, TAG,
                      "Only %dx%d images supported (got %dx%d)", PANEL_WIDTH,
                      PANEL_HEIGHT, width, height);

  esp_err_t ret;

  // Region 1: Top 25 rows (96×25 = 300 bytes = 5 chunks of 60 bytes)
  uint8_t top_height = 25;
  size_t top_bytes = (PANEL_WIDTH * top_height) / 8;  // 300 bytes
  uint8_t top_chunks = (top_bytes + RPC_CHUNK_SIZE - 1) / RPC_CHUNK_SIZE;  // 5

  PrepareFramebufferRequest prep_req = {
      .compressed = 0,
      .chunk_count = top_chunks,
      .x = 0,
      .y = 0,
      .width = PANEL_WIDTH,
      .height = top_height,
      .bitblt_mode = bitblt_mode,
      .pre_composite_mode =
          clear_before ? PRE_COMPOSITE_MODE_CLEAR : PRE_COMPOSITE_MODE_KEEP,
      .post_composite_mode = POST_COMPOSITE_MODE_WAIT,  // Don't commit yet
  };

  ret = driver_->PrepareFramebuffer(prep_req);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to prepare top region");

  // Upload top region chunks
  for (uint8_t i = 0; i < top_chunks; i++) {
    SetFramebufferRequest chunk_req = {};
    chunk_req.chunk_index = i;

    size_t offset = i * RPC_CHUNK_SIZE;
    size_t chunk_size =
        (i == top_chunks - 1) ? (top_bytes - offset) : RPC_CHUNK_SIZE;

    memcpy(chunk_req.chunk_data, bitmap + offset, chunk_size);

    // Zero-fill partial last chunk if needed
    if (chunk_size < RPC_CHUNK_SIZE) {
      memset(chunk_req.chunk_data + chunk_size, 0, RPC_CHUNK_SIZE - chunk_size);
    }

    ret = driver_->SetFramebufferChunk(chunk_req);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to upload top chunk %d/%d", i + 1,
                        top_chunks);
  }

  ESP_LOGD(TAG, "Uploaded top region (%d bytes in %d chunks)", top_bytes,
           top_chunks);

  // Region 2: Bottom 7 rows (96×7 = 84 bytes = 2 chunks, last partial)
  uint8_t bottom_height = PANEL_HEIGHT - top_height;        // 7
  size_t bottom_bytes = (PANEL_WIDTH * bottom_height) / 8;  // 84 bytes
  uint8_t bottom_chunks =
      (bottom_bytes + RPC_CHUNK_SIZE - 1) / RPC_CHUNK_SIZE;  // 2

  prep_req.chunk_count = bottom_chunks;
  prep_req.y = top_height;
  prep_req.height = bottom_height;
  prep_req.pre_composite_mode = PRE_COMPOSITE_MODE_KEEP;  // Don't clear
  prep_req.post_composite_mode =
      commit_after ? POST_COMPOSITE_MODE_COMMIT : POST_COMPOSITE_MODE_WAIT;

  ret = driver_->PrepareFramebuffer(prep_req);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to prepare bottom region");

  // Upload bottom region chunks
  for (uint8_t i = 0; i < bottom_chunks; i++) {
    SetFramebufferRequest chunk_req = {};
    chunk_req.chunk_index = i;

    size_t offset = top_bytes + (i * RPC_CHUNK_SIZE);
    size_t chunk_size = (i == bottom_chunks - 1)
                            ? (bottom_bytes - (i * RPC_CHUNK_SIZE))
                            : RPC_CHUNK_SIZE;

    memcpy(chunk_req.chunk_data, bitmap + offset, chunk_size);

    // Zero-fill partial last chunk
    if (chunk_size < RPC_CHUNK_SIZE) {
      memset(chunk_req.chunk_data + chunk_size, 0, RPC_CHUNK_SIZE - chunk_size);
    }

    ret = driver_->SetFramebufferChunk(chunk_req);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to upload bottom chunk %d/%d", i + 1,
                        bottom_chunks);
  }

  ESP_LOGD(TAG, "Uploaded bottom region (%d bytes in %d chunks)", bottom_bytes,
           bottom_chunks);
  ESP_LOGD(TAG, "Framebuffer upload complete (%dx%d, %d total bytes)", width,
           height, FRAMEBUFFER_SIZE);

  return ESP_OK;
}

esp_err_t GPURenderer::LoadAndShowImage(const char* path,
                                        BitBLTMode bitblt_mode,
                                        bool clear_before, bool commit_after) {
  uint8_t framebuffer[FRAMEBUFFER_SIZE];

  ESP_RETURN_ON_ERROR(LoadRawBitmap(path, framebuffer, FRAMEBUFFER_SIZE), TAG,
                      "Failed to load bitmap");

  ESP_RETURN_ON_ERROR(
      UploadFramebuffer(framebuffer, PANEL_WIDTH, PANEL_HEIGHT, bitblt_mode,
                        clear_before, commit_after),
      TAG, "Failed to upload framebuffer");

  return ESP_OK;
}

// ============================================================================
// GPUTestCapabilities Implementation
// ============================================================================

esp_err_t GPUTestCapabilities::SetScreen(uint8_t screen) {
  auto& anim = DisplayTestAnimation::GetInstance();
  anim.auto_mode_ = false;
  anim.loop_once_ = false;
  anim.completed_ = false;

  anim.ShowTestPattern(static_cast<DisplayTestScreen>(screen));
  return ESP_OK;
}

esp_err_t GPUTestCapabilities::SetAutoMode(uint8_t duration_secs,
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
constexpr DisplayTestScreen GPUTestCapabilities::kSequence[];

DisplayTestScreen GPUTestCapabilities::GetNextTestScreen() {
  DisplayTestScreen screen = kSequence[current_index_];
  current_index_ = (current_index_ + 1) % std::size(kSequence);
  return screen;
}

void GPUTestCapabilities::ResetTestSequence() { current_index_ = 0; }

size_t GPUTestCapabilities::GetTestScreenCount() const {
  return std::size(kSequence);
}

bool GPUTestCapabilities::IsAtStart() const { return current_index_ == 0; }

// ============================================================================
// Portrait Mode Helper Functions
// ============================================================================

void GPURenderer::LogicalToPhysical(int lx, int ly, int8_t& px, int8_t& py) {
  // Portrait (32x96) -> Landscape (96x32)
  // User views screen rotated 90° clockwise from physical orientation
  // Verified: physical (1,1) appears at user's right-upper area
  // Conversion: px = ly, py = 31 - lx
  px = static_cast<int8_t>(ly);
  py = static_cast<int8_t>(31 - lx);
}

esp_err_t GPURenderer::UploadSmallBitmap(const uint8_t* bitmap, int8_t x,
                                         int8_t y, uint8_t width,
                                         uint8_t height, PreCompositeMode pre,
                                         PostCompositeMode post) {
  size_t total_bytes = (width * height + 7) / 8;
  uint8_t chunk_count = (total_bytes + RPC_CHUNK_SIZE - 1) / RPC_CHUNK_SIZE;

  if (chunk_count > RPC_CHUNK_MAX_COUNT) {
    ESP_LOGE(TAG, "Bitmap too large: %zu bytes", total_bytes);
    return ESP_ERR_INVALID_SIZE;
  }

  PrepareFramebufferRequest prep = {};
  prep.compressed = 0;
  prep.chunk_count = chunk_count;
  prep.x = x;
  prep.y = y;
  prep.width = width;
  prep.height = height;
  prep.bitblt_mode = BITBLT_MODE_COPY;
  prep.pre_composite_mode = pre;
  prep.post_composite_mode = post;

  esp_err_t ret = driver_->PrepareFramebuffer(prep);
  ESP_RETURN_ON_ERROR(ret, TAG, "PrepareFramebuffer failed");

  for (uint8_t i = 0; i < chunk_count; i++) {
    SetFramebufferRequest chunk = {};
    chunk.chunk_index = i;

    size_t offset = i * RPC_CHUNK_SIZE;
    size_t chunk_size =
        (i == chunk_count - 1) ? (total_bytes - offset) : RPC_CHUNK_SIZE;

    memcpy(chunk.chunk_data, bitmap + offset, chunk_size);
    if (chunk_size < RPC_CHUNK_SIZE) {
      memset(chunk.chunk_data + chunk_size, 0, RPC_CHUNK_SIZE - chunk_size);
    }

    ret = driver_->SetFramebufferChunk(chunk);
    ESP_RETURN_ON_ERROR(ret, TAG, "SetFramebufferChunk failed");
  }

  return ESP_OK;
}

#endif  // CONFIG_DISPLAY_DRIVER_GPU
