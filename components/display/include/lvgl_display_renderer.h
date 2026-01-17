#ifndef H_LVGL_DISPLAY_RENDERER_
#define H_LVGL_DISPLAY_RENDERER_

#include <array>
#include <string>
#include <vector>

#include "display_renderer.h"
#include "lvgl_display_driver.h"

#ifdef CONFIG_DISPLAY_DRIVER_LVGL

#include "bsp/display.h"
#include "lvgl.h"

// Forward declaration
class LVGLRenderer;

/**
 * @brief LVGL-specific test capabilities implementation
 *
 * Provides manual screen selection and auto-cycling test modes
 * for QA validation of LVGL display hardware.
 *
 * Color display: includes color tests and grey.
 */
class LVGLTestCapabilities : public IDisplayTestCapabilities {
 public:
  explicit LVGLTestCapabilities(LVGLRenderer* parent) : parent_(parent) {}

  esp_err_t SetScreen(uint8_t screen) override;
  esp_err_t SetAutoMode(uint8_t duration_secs, bool loop_once) override;

  // Stateful iterator for renderer-specific test sequence
  DisplayTestScreen GetNextTestScreen() override;
  void ResetTestSequence() override;
  size_t GetTestScreenCount() const override;
  bool IsAtStart() const override;

 private:
  LVGLRenderer* parent_;
  size_t current_index_ = 0;

  // Color display: includes colors and grey
  static constexpr DisplayTestScreen kSequence[] = {
      DisplayTestScreen::TEST_SCREEN_DEFAULT,     // "TEST" text
      DisplayTestScreen::TEST_SCREEN_LOW_LIGHT,   // 10% brightness
      DisplayTestScreen::TEST_SCREEN_HIGH_LIGHT,  // 100% brightness
      DisplayTestScreen::TEST_SCREEN_RED,         // Red
      DisplayTestScreen::TEST_SCREEN_GREEN,       // Green
      DisplayTestScreen::TEST_SCREEN_BLUE,        // Blue
      DisplayTestScreen::TEST_SCREEN_WHITE,       // Full white
      DisplayTestScreen::TEST_SCREEN_BLACK,       // Full black
      DisplayTestScreen::TEST_SCREEN_GREY,        // 50% grey
      DisplayTestScreen::TEST_SCREEN_OFF,         // Backlight off
  };
};

/**
 * @brief LVGL-based display renderer - Retained-Mode UI Strategy
 *
 * ## Rendering Strategy: Retained-Mode (Object Tree)
 *
 * This renderer implements a retained-mode UI approach where:
 * - ESP32 creates and manages LVGL UI objects (labels, bars, charts, images)
 * - LVGL maintains an object tree in memory
 * - ESP32 CPU does all rendering to framebuffer
 * - Higher CPU/memory overhead but maximum flexibility
 *
 * **Architecture:**
 * ```
 * ESP32 (LVGLRenderer)          LVGL UI Toolkit           LCD Hardware
 *     |                              |                          |
 *     | lv_label_create()            |                          |
 *     |----------------------------->|                          |
 *     |                              | ← Stores in object tree  |
 *     |                              |                          |
 *     | lv_label_set_text()          |                          |
 *     |----------------------------->|                          |
 *     |                              | ← Updates object         |
 *     |                              |                          |
 *     |         [Refresh Timer]      |                          |
 *     |                              | Renders objects          |
 *     |                              |------------------------->|
 *     |                              |                          | ← Display
 * ```
 *
 * ## Key Differences from Command-Based (FPGA):
 *
 * | Aspect           | LVGLRenderer (Retained)    | FPGARenderer (Command) |
 * |------------------|----------------------------|------------------------|
 * | UI State         | Retained in memory         | No state on ESP32      |
 * | Rendering        | ESP32 CPU                  | FPGA hardware          |
 * | Text Support     | Full font engine           | None                   |
 * | Memory Usage     | Higher (object tree)       | Lower (no objects)     |
 * | CPU Usage        | Higher (software render)   | Lower (hw accelerated) |
 * | Flexibility      | Maximum (any UI possible)  | Limited (predefined)   |
 *
 * ## Capabilities:
 * - **Arbitrary text rendering** with multiple fonts (small, medium, large)
 * - **Rich progress bars** with colors (success, warning, error)
 * - **Custom icons** from image files (PNG/BMP support)
 * - **Complex layouts** using LVGL flexbox and grid
 * - **Power charts** using LVGL chart widget with multiple series
 * - **Animations** using LVGL animation engine
 * - **Full styling** support (colors, borders, shadows, etc.)
 *
 * ## Implementation Notes:
 * - Uses `bsp_display_lock()` for thread-safe access to LVGL
 * - Tracks `current_content_` to manage object lifecycle
 * - Calls `lv_obj_clean()` in `Clear()` to remove old objects
 * - Font selection via helper methods (GetFontForSize)
 * - Hardware control delegated to `LVGLDisplayDriver`
 *
 * @see LVGLDisplayDriver for BSP-level display control
 * @see FPGARenderer for comparison with command-based rendering
 * @see GPURenderer for another rendering strategy with image support
 */
class LVGLRenderer : public IDisplayRenderer {
 public:
  explicit LVGLRenderer(LVGLDisplayDriver* driver);

  // =========================================================================
  // IDisplayRenderer implementation
  // =========================================================================

  esp_err_t Init() override;
  esp_err_t Deinit() override;
  void Clear() override;
  void ShowPercentage(uint8_t percentage, ProgressStyle style) override;

  IDisplayDriver* GetDriver() override;

  // Intent-based rendering (hardware-agnostic state display)
  void ShowAdvertisingState() override;
  void ShowConnectedState() override;
  void ShowBootState() override;
  void ShowBootProgress(float progress) override;
  void ShowWifiConnecting(float progress) override;
  void ShowOTAProgress(float progress, bool is_download_phase) override;

  // Extended rendering (text, images, complex layouts)
  void ShowText(const char* text, const TextStyle& style) override;
  void ShowImage(const char* path, Alignment align) override;
  void ShowAnimatedImage(const char* dir_path, uint32_t frame_count,
                         uint32_t period_ms) override;
  void ShowInfoScreen(const char* title, const char* content) override;
  void ShowPowerChart(const PowerDataPoint* data, size_t count) override;
  void ShowChargingPorts(const std::vector<PortChargingInfo>& ports,
                         bool show_power) override;
  void ShowTestPattern(DisplayTestScreen screen) override;

  // Charging display (intent-based)
  WakeInterval ShowChargingStatus(IAnimationDataSource* data_source,
                                  int64_t dt_us) override;
  WakeInterval ShowIdleState(IAnimationDataSource* data_source,
                             int64_t dt_us) override;

  // =========================================================================
  // Test capabilities
  // =========================================================================

  IDisplayTestCapabilities* GetTestCapabilities() override {
    return &test_capabilities_;
  }

 private:
  LVGLDisplayDriver* driver_;
  lv_obj_t* current_content_;
  LVGLTestCapabilities test_capabilities_{this};

  // Internal charging display helper
  void ShowChargingTotal(const std::vector<PortChargingInfo>& ports);

  // Progress-based animation state (autonomous mode)
  // LVGL handles animation internally, we just track if started
  bool boot_animation_started_ = false;
  bool wifi_animation_started_ = false;

  // Idle state tracking (avoid re-creating LVGL objects unnecessarily)
  bool idle_showing_pin_ = false;
  bool idle_last_wifi_connected_ = false;
  uint8_t idle_last_type_ = UINT8_MAX;
  uint8_t idle_last_power_tier_ = UINT8_MAX;

  // Animated image storage (static buffers to avoid heap fragmentation)
  static constexpr size_t kMaxAnimFrames = 100;
  std::array<std::string, kMaxAnimFrames> anim_frame_paths_;  // Path strings
  std::array<const char*, kMaxAnimFrames>
      anim_frame_ptrs_;          // Pointers for LVGL
  size_t anim_frame_count_ = 0;  // Actual number of frames loaded

  // Font resources
  typedef enum __attribute__((packed)) {
    FONT_KIND_SMALL,
    FONT_KIND_MIDDLE,
    FONT_KIND_LARGE,
    FONT_KIND_MAX,
  } FontKind;
  lv_font_t* fonts_[FONT_KIND_MAX] = {};

  // LVGL-specific styles (moved from BaseAnimation)
  typedef enum __attribute__((packed)) {
    STYLE_KIND_DEFAULT,
    STYLE_KIND_DASHED_LINE,
    STYLE_KIND_MIDDLE_FONT,
    STYLE_KIND_LARGE_FONT,
    STYLE_KIND_SMALL_FONT_AND_COLOR,
    STYLE_KIND_SMALL_FONT_AND_ALIGN_RIGHT,
    STYLE_KIND_MAX,
  } StyleKind;

  std::array<lv_style_t, STYLE_KIND_MAX> styles_;

  // Resource initialization
  esp_err_t InitFonts();
  esp_err_t InitDecoder();
  esp_err_t InitScreenStyle();
  void InitStyles();

  // Helper methods
  lv_font_t* GetFontForSize(TextStyle::Size size);
  lv_font_t* GetFont(FontKind kind);
  lv_align_t GetLVGLAlign(Alignment align);
  lv_color_t GetColorForStyle(ProgressStyle style);
  const char* GetIconPath(IconType icon);
  void ShowIcon(IconType icon);  // Internal helper for
                                 // ShowAdvertisingState/ShowConnectedState

  // Allow test capabilities to access private members if needed
  friend class LVGLTestCapabilities;
};

#endif  // CONFIG_DISPLAY_DRIVER_LVGL

#endif  // H_LVGL_DISPLAY_RENDERER_
