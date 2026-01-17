#ifndef H_GPU_DISPLAY_RENDERER_
#define H_GPU_DISPLAY_RENDERER_

#include "display_renderer.h"
#include "gpu_display_driver.h"

#ifdef CONFIG_DISPLAY_DRIVER_GPU

#include "animation_data_source.h"
#include "gpu_data_types.h"

/**
 * @brief Character position for charging total display layout
 */
struct CharPosition {
  int8_t x;
  int8_t y;
  GPUFontSizeIndex font;
};

/**
 * @brief Layout configuration for ShowChargingTotal
 *
 * Supports different layouts for portrait (270°) and landscape (0°) modes.
 * Each layout defines positions for the "T" label and up to 3 power digits.
 */
struct ChargingTotalLayout {
  GPUTextRotation rotation;
  CharPosition label;     // "T" indicator
  CharPosition ones;      // Ones digit (rightmost)
  CharPosition tens;      // Tens digit
  CharPosition hundreds;  // Hundreds digit (stacked above tens for 100-199W)
};

// Forward declaration
class GPURenderer;

/**
 * @brief GPU-specific test capabilities implementation
 *
 * Provides manual screen selection and auto-cycling test modes
 * for QA validation of GPU display hardware.
 *
 * Monochrome display: only brightness/contrast tests are meaningful.
 * Colors (RED, GREEN, BLUE) are excluded.
 */
class GPUTestCapabilities : public IDisplayTestCapabilities {
 public:
  explicit GPUTestCapabilities(GPURenderer* parent) : parent_(parent) {}

  esp_err_t SetScreen(uint8_t screen) override;
  esp_err_t SetAutoMode(uint8_t duration_secs, bool loop_once) override;

  // Stateful iterator for renderer-specific test sequence
  DisplayTestScreen GetNextTestScreen() override;
  void ResetTestSequence() override;
  size_t GetTestScreenCount() const override;
  bool IsAtStart() const override;

 private:
  GPURenderer* parent_;
  size_t current_index_ = 0;

  // Monochrome display: meaningful tests only (no colors, no grey)
  static constexpr DisplayTestScreen kSequence[] = {
      DisplayTestScreen::TEST_SCREEN_DEFAULT,     // "TEST" text
      DisplayTestScreen::TEST_SCREEN_LOW_LIGHT,   // 10% brightness
      DisplayTestScreen::TEST_SCREEN_HIGH_LIGHT,  // 100% brightness
      DisplayTestScreen::TEST_SCREEN_BLACK,       // Full black
      DisplayTestScreen::TEST_SCREEN_WHITE,       // Full white
      DisplayTestScreen::TEST_SCREEN_OFF,         // Backlight off
  };
};

/**
 * @brief GPU-based display renderer
 *
 * Renders using a dedicated GPU chip over UART RPC. The GPU provides
 * hardware-accelerated graphics primitives, built-in animations (VU meters,
 * Game of Life), text rendering (6x12 and 8x16 fonts), and framebuffer
 * compositing.
 *
 * Display: 96x32 monochrome OLED
 *
 * Capabilities:
 * - Text rendering (up to 12 characters, horizontal/vertical)
 * - Power VU meters (single port and multi-port)
 * - Custom framebuffer upload (for icons and complex graphics)
 * - Hardware test patterns
 * - BitBLT compositing modes
 *
 * The GPU chip handles all rendering, so this renderer sends RPC commands
 * rather than drawing pixels directly.
 */
class GPURenderer : public IDisplayRenderer {
 public:
  explicit GPURenderer(GPUDisplayDriver* driver);

  // =========================================================================
  // IDisplayRenderer implementation
  // =========================================================================

  void Clear() override;
  void ShowPercentage(uint8_t percentage, ProgressStyle style) override;

  // Intent-based state display
  void ShowAdvertisingState() override;
  void ShowConnectedState() override;
  void ShowBootState() override;
  // Progress-based animation methods
  void ShowBootProgress(float progress) override;
  void ShowWifiConnecting(float progress) override;
  void ShowGreetingProgress(float progress) override;
  void ShowOTAProgress(float progress, bool is_download_phase) override;

  // Complex rendering operations
  void ShowPowerChart(const PowerDataPoint* data, size_t count) override;

  // Information display
  void ShowInfoScreen(const char* title, const char* content) override;

  // Charging display (intent-based)
  WakeInterval ShowChargingStatus(IAnimationDataSource* data_source,
                                  int64_t dt_us) override;
  WakeInterval ShowIdleState(IAnimationDataSource* data_source,
                             int64_t dt_us) override;

  // Display test patterns
  void ShowTestPattern(DisplayTestScreen screen) override;

  IDisplayDriver* GetDriver() override;

  // Extended rendering (text, images, complex layouts)
  void ShowText(const char* text, const TextStyle& style) override;
  void ShowImage(const char* path, Alignment align) override;
  void ShowAnimatedImage(const char* dir_path, uint32_t frame_count,
                         uint32_t period_ms) override;

  // Progress-based state animations (for BLE advertising/connected)
  void ShowAdvertisingProgress(float progress);
  void ShowConnectedProgress(float progress);

  // Lifecycle methods
  esp_err_t Init() override;
  esp_err_t Deinit() override;

  // =========================================================================
  // Test capabilities
  // =========================================================================

  IDisplayTestCapabilities* GetTestCapabilities() override {
    return &test_capabilities_;
  }

 private:
  friend class GPUTestCapabilities;
  GPUTestCapabilities test_capabilities_{this};
  GPUDisplayDriver* driver_;

  // Progress-based animation state (minimal, no autonomous playback)
  uint32_t boot_frame_count_ = 0;
  uint32_t current_boot_frame_ = UINT32_MAX;
  uint32_t wifi_frame_count_ = 0;
  uint32_t current_wifi_frame_ = UINT32_MAX;
  uint32_t greeting_frame_count_ = 0;
  uint32_t current_greeting_frame_ = UINT32_MAX;
  uint32_t advertising_frame_count_ = 0;
  uint32_t current_advertising_frame_ = UINT32_MAX;
  uint32_t connected_frame_count_ = 0;
  uint32_t current_connected_frame_ = UINT32_MAX;

  // OTA animation state
  bool ota_last_phase_ = true;  // true = download, false = update
  bool ota_first_call_ = true;

  // Idle animation state
  uint8_t idle_last_type_ = UINT8_MAX;
  uint32_t idle_frame_ = 0;
  int64_t idle_frame_elapsed_us_ = 0;

  // Frame cache for boot animation (avoid filesystem I/O during playback)
  static constexpr size_t kMaxCachedFrames = 5;
  uint8_t frame_cache_[kMaxCachedFrames][384] = {};
  bool frames_cached_ = false;

  // Helper methods
  GPUFontSizeIndex GetGPUFontSize(TextStyle::Size size);
  uint8_t CalculateXForAlign(const char* text, GPUFontSizeIndex font_size,
                             Alignment align);
  void ShowIcon(IconType icon);  // Internal helper for
                                 // ShowAdvertisingState/ShowConnectedState
  esp_err_t ShowDataMatrixImage(const char* path);
  void ShowChargingTotal(const std::vector<PortChargingInfo>& ports);

  // Image loading helpers (moved from image_utils)
  esp_err_t LoadRawBitmap(const char* path, uint8_t* buffer,
                          size_t buffer_size);
  esp_err_t UploadFramebuffer(const uint8_t* bitmap, uint8_t width,
                              uint8_t height, BitBLTMode bitblt_mode,
                              bool clear_before, bool commit_after);
  esp_err_t LoadAndShowImage(const char* path, BitBLTMode bitblt_mode,
                             bool clear_before, bool commit_after);

  // Coordinate conversion: logical (32x96) -> physical (96x32)
  static void LogicalToPhysical(int lx, int ly, int8_t& px, int8_t& py);
  // Upload small region bitmap
  esp_err_t UploadSmallBitmap(const uint8_t* bitmap, int8_t x, int8_t y,
                              uint8_t width, uint8_t height,
                              PreCompositeMode pre, PostCompositeMode post);

  // Get GOL density level based on current power consumption
  GOLDensityLevel GetPowerBasedDensityLevel();
  AnimationDataSource data_source_;  // For querying power data
};

#endif  // CONFIG_DISPLAY_DRIVER_GPU

#endif  // H_GPU_DISPLAY_RENDERER_
