#ifndef H_DISPLAY_RENDERER_
#define H_DISPLAY_RENDERER_

/**
 * @file display_renderer.h
 * @brief Display Renderer Interface - Hardware-Agnostic Rendering Strategy
 *
 * ## Three-Layer Architecture
 *
 * This header defines the middle layer of the display architecture, which
 * separates WHAT to show (animations) from HOW to render (renderers) and
 * WHERE to render (drivers):
 *
 * ```
 * ┌─────────────────────────────────────────────────────────┐
 * │  Animation Layer (Business Logic)                       │
 * │  - ChargingAnimation, IdleAnimation, etc.               │
 * │  - Knows WHAT to show, not HOW                          │
 * │  - Hardware-agnostic code (no #ifdef blocks)            │
 * └────────────────┬────────────────────────────────────────┘
 *                  ↓
 * ┌─────────────────────────────────────────────────────────┐
 * │  Renderer Layer (Rendering Strategy) ← THIS FILE        │
 * │  - IDisplayRenderer interface                           │
 * │  - FPGARenderer: Send commands to display hardware      │
 * │  - LVGLRenderer: Create LVGL UI objects                 │
 * │  - GPURenderer: Send RPC commands to GPU chip           │
 * │  - Knows HOW to render, delegates hardware control      │
 * └────────────────┬────────────────────────────────────────┘
 *                  ↓
 * ┌─────────────────────────────────────────────────────────┐
 * │  Driver Layer (Hardware Control)                        │
 * │  - IDisplayDriver interface                             │
 * │  - Controls: mode, intensity, rotation                  │
 * │  - Hardware abstraction ONLY                            │
 * └─────────────────────────────────────────────────────────┘
 * ```
 *
 * ## Separation of Concerns
 *
 * | Layer      | Responsibility                  | Example                    |
 * |------------|---------------------------------|----------------------------|
 * | Animation  | Business logic: what, when, why | "Show 75% with dim light"  |
 * | Renderer   | Rendering strategy: how to draw | "Create LVGL bar" or cmd   |
 * | Driver     | Hardware control: brightness    | "Set backlight to 30%"     |
 *
 * ## Renderer vs Driver Separation
 *
 * **IDisplayRenderer** (visual content rendering):
 * - `ShowText()` - Display text with styling
 * - `ShowPercentage()` - Display progress bars
 * - `ShowImage()` - Display static/animated images
 * - `ShowPowerChart()` - Display graphs and charts
 *
 * **IDisplayDriver** (hardware control via `GetRenderer()->GetDriver()`):
 * - `SetMode()` - Turn display on/off, set display mode
 * - `SetIntensity()` - Adjust brightness (0-100%)
 * - `SetRotation()` - Change display orientation
 *
 * ## Key Design Principles
 *
 * 1. **Hardware agnosticism**: Animations never call BSP/LVGL/FPGA APIs
 * directly
 * 2. **Strategy pattern**: Each renderer implements rendering differently
 *    - FPGA sends commands, hardware does rendering
 *    - LVGL creates UI objects, ESP32 does rendering
 *    - GPU sends RPC, external chip does rendering
 * 3. **Single responsibility**: Renderer = rendering, Driver = hardware
 * control
 * 4. **State synchronization**: All hardware control goes through driver
 *    to prevent state desync
 *
 * ## Adding New Display Hardware
 *
 * To support new display hardware:
 * 1. Implement IDisplayDriver interface (hardware abstraction)
 * 2. Implement IDisplayRenderer interface (rendering strategy)
 * 3. Update main.cpp to instantiate your driver/renderer
 * 4. No changes needed to animation code!
 *
 * @see display_driver.h for hardware control interface
 * @see display_animation.h for animation base classes
 * @see docs/animation.md for architecture documentation
 */

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "display_driver.h"
#include "display_test.h"
#include "display_types.h"
#include "esp_err.h"

// Forward declaration for intent-based API
class IAnimationDataSource;

// Universal alignment for any content type (text, image, icon)
enum class Alignment {
  LEFT,    // Left-aligned
  CENTER,  // Center-aligned
  RIGHT,   // Right-aligned
};

// Text-specific styling
struct TextStyle {
  enum class Size {
    SMALL,   // Small font (e.g., 6x12 for GPU, small LVGL font)
    MEDIUM,  // Medium font
    LARGE,   // Large font (e.g., 8x16 for GPU, large LVGL font)
  };

  Size size = Size::MEDIUM;
  Alignment align = Alignment::LEFT;
};

// Progress bar styling options
enum class ProgressStyle {
  NORMAL,   // Full brightness, standard color
  DIM,      // Dimmed for background operations
  SUCCESS,  // Green/success indicator
  WARNING,  // Yellow/warning indicator
  ERROR,    // Red/error indicator
};

// Icon types
enum class IconType {
  CHARGING,
  IDLE,
  CONNECTED,
  DISCONNECTED,
  WARNING,
  ERROR,
  CHECK,
  POWER,
};

// Power chart data point
struct PowerDataPoint {
  uint32_t timestamp;
  uint16_t power_mw;
  uint8_t port_id;
};

// Per-port charging information for ShowChargingPorts()
struct PortChargingInfo {
  uint8_t port_id;      // Port identifier (e.g., 0 for C1)
  uint16_t power_w;     // Power in watts
  uint16_t voltage_mv;  // Voltage in millivolts
  uint16_t current_ma;  // Current in milliamps
  uint8_t fc_protocol;  // Fast charging protocol
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Utility functions for display rendering
 *
 * These helpers are used across multiple renderer implementations
 * to maintain consistent behavior.
 */
namespace display_renderer_utils {

/**
 * @brief Map ProgressStyle to display intensity
 * @param style Progress bar style
 * @return Intensity value (0-100)
 *
 * DIM style uses 30% intensity for background operations.
 * All other styles use 100% intensity.
 */
inline uint8_t GetIntensityForStyle(ProgressStyle style) {
  return (style == ProgressStyle::DIM) ? 30 : 100;
}

/**
 * @brief Get display name for port
 * @param port_id Port identifier (0-based)
 * @return "A" for port 0, "C1"/"C2"/... for others
 */
inline std::string GetPortDisplayName(uint8_t port_id) {
  return port_id == 0 ? "A" : "C" + std::to_string(port_id);
}

// Common constants used across renderers
constexpr uint8_t kMaxPercentage = 100;   // Maximum percentage value
constexpr uint32_t kMaxPowerMw = 100000;  // 100W in milliwatts
constexpr size_t kFCProtocolCount = 22;   // Fast charge protocol array size

// Protocol names mapped from FastChargingProtocol enum (data_types.proto)
constexpr const char* kFCProtocolNames[kFCProtocolCount] = {
    "_",     "QC2",   "QC3",    "QC3+", "SFCP", "AFC", "FCP", "SCP",
    "VOOC1", "VOOC4", "SVOOC2", "TFCP", "UFCS", "PE1", "PE2", "PD5V",
    "PDHV",  "AVS",   "PPS",    "EPR",  "AVS",  "_"};

/**
 * @brief Get protocol name from FastChargingProtocol enum value
 * @param protocol Protocol value (0-21)
 * @return Short protocol name for display (max 6 chars)
 */
inline const char* GetFCProtocolName(uint8_t protocol) {
  return protocol < kFCProtocolCount ? kFCProtocolNames[protocol]
                                     : kFCProtocolNames[0];
}

}  // namespace display_renderer_utils

// ============================================================================
// Display Test Capabilities
// ============================================================================

/**
 * @brief Optional test capability interface for display renderers
 *
 * Only renderers with advanced test modes implement this interface.
 * Used for production testing and QA validation.
 *
 * This capability-based design allows:
 * - Runtime discovery (no compile-time #ifdef in DisplayManager)
 * - Hardware-specific test features in the correct layer
 * - Clean separation: test logic in renderer, not in business layer
 */
class IDisplayTestCapabilities {
 public:
  virtual ~IDisplayTestCapabilities() = default;

  /**
   * @brief Set specific test screen (LVGL screens, GPU patterns, etc.)
   * @param screen Screen index to display
   * @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED if not available
   */
  virtual esp_err_t SetScreen(uint8_t screen) = 0;

  /**
   * @brief Enable automatic test mode cycling through screens
   * @param duration_secs Duration per screen in seconds (0 = default)
   * @param loop_once True to cycle once and stop, false for infinite loop
   * @return ESP_OK on success
   */
  virtual esp_err_t SetAutoMode(uint8_t duration_secs, bool loop_once) = 0;

  /**
   * @brief Get next test screen in renderer-specific sequence
   *
   * Stateful iterator - advances internal index each call.
   * Wraps to beginning after last screen.
   *
   * @return Next DisplayTestScreen in the sequence
   */
  virtual DisplayTestScreen GetNextTestScreen() = 0;

  /**
   * @brief Reset sequence to first screen
   */
  virtual void ResetTestSequence() = 0;

  /**
   * @brief Get total number of screens in sequence
   * @return Number of screens
   */
  virtual size_t GetTestScreenCount() const = 0;

  /**
   * @brief Check if iterator is at start position
   * @return true if next GetNextTestScreen() will return first screen
   */
  virtual bool IsAtStart() const = 0;
};

/**
 * @brief Display renderer interface
 *
 * This interface separates rendering strategy from animation logic.
 * Different implementations handle different display hardware:
 *
 * - FPGARenderer: Sends commands to FPGA display hardware
 * - LVGLRenderer: Creates LVGL UI objects for direct LCD control
 * - GPURenderer: Sends RPC commands to GPU display chip
 * - StubRenderer: No-op for testing
 *
 * Hardware control (brightness, mode, rotation) is delegated to
 * the underlying IDisplayDriver through GetDriver().
 *
 * Architecture:
 * Animation → IDisplayRenderer → IDisplayDriver → Hardware
 *
 * Example usage:
 * @code
 * void ChargingAnimation::Play() {
 *   auto* r = GetRenderer();
 *   auto* driver = r->GetDriver();
 *
 *   // Hardware control
 *   driver->SetIntensity(100);
 *   driver->SetMode(DisplayMode::ON);
 *
 *   // Render content (hardware-agnostic)
 *   TextStyle style;
 *   style.size = TextStyle::Size::LARGE;
 *   style.align = Alignment::CENTER;
 *   r->ShowText("Charging", style);
 *   r->ShowPercentage(75, ProgressStyle::NORMAL);
 * }
 * @endcode
 */
class IDisplayRenderer {
 public:
  // =========================================================================
  // CORE METHODS - Override as needed
  // =========================================================================
  // Lifecycle and rendering operations. All have sensible defaults.
  // Only GetDriver() is pure virtual (must be implemented).

  virtual esp_err_t Init() { return ESP_OK; }
  virtual esp_err_t Deinit() { return ESP_OK; }
  virtual void Clear() {}
  virtual void ShowPercentage(uint8_t percentage, ProgressStyle style) {
    (void)percentage;
    (void)style;
  }

  // =========================================================================
  // EXTENDED METHODS - Optional rendering features
  // =========================================================================
  // These methods may be no-ops on minimal renderers (e.g., FPGA).

  virtual void ShowText(const char* text, const TextStyle& style) {
    (void)text;
    (void)style;
  }

  /**
   * @brief Show an image from file
   * @param path Image file path
   * @param align Alignment on screen
   *
   * Renderers without image support (FPGA) will gracefully degrade.
   */
  virtual void ShowImage(const char* path,
                         Alignment align = Alignment::CENTER) {
    (void)path;
    (void)align;
  }
  virtual void ShowAnimatedImage(const char* dir_path, uint32_t frame_count,
                                 uint32_t period_ms = 100) {
    (void)dir_path;
    (void)frame_count;
    (void)period_ms;
  }

  // =========================================================================
  // INTENT METHODS - Hardware-agnostic state display
  // =========================================================================
  // These methods express WHAT to display, not HOW. Each renderer implements
  // them according to its hardware capabilities with graceful degradation:
  //   - FPGA: Shows percentage bar, changes mode, or does nothing
  //   - GPU: Shows icons/images via RPC commands
  //   - LVGL: Shows full UI with icons and text
  // Animations call these to remain hardware-agnostic.

  virtual void ShowAdvertisingState() {}  // BLE advertising indicator
  virtual void ShowConnectedState() {}    // BLE connected indicator
  virtual void ShowBootState() {}         // Boot animation indicator

  // Progress-based intent methods (animation drives timing)
  // Animations call these every frame with progress 0.0-1.0
  // Renderers convert progress to hardware-specific visualization

  /**
   * @brief Show boot progress animation
   * @param progress Boot progress from 0.0 (start) to 1.0 (complete)
   *
   * Animation drives timing, renderer decides visualization:
   * - FPGA: LED bar percentage
   * - GPU: Frame sequence based on progress
   * - LVGL: Autonomous animation (progress ignored after start)
   */
  virtual void ShowBootProgress(float progress) { (void)progress; }

  /**
   * @brief Show WiFi connecting animation
   * @param progress Animation progress from 0.0 to 1.0 (loops)
   *
   * Animation drives timing, renderer decides visualization:
   * - FPGA: Pulsing LED bar effect
   * - GPU: Frame sequence based on progress
   * - LVGL: Autonomous animation (progress ignored after start)
   */
  virtual void ShowWifiConnecting(float progress) { (void)progress; }

  /**
   * @brief Show greeting animation
   * @param progress Animation progress from 0.0 to 1.0
   */
  virtual void ShowGreetingProgress(float progress) { (void)progress; }

  /**
   * @brief Show OTA update animation
   * @param progress Animation progress from 0.0 to 1.0
   * @param is_download_phase true during download phase, false during update
   *
   * Animation drives timing, renderer decides visualization:
   * - FPGA: Progress bar percentage
   * - GPU: Static image (download.bin/update.bin) with looping progress
   * - LVGL: Autonomous animation
   */
  virtual void ShowOTAProgress(float progress, bool is_download_phase) {
    (void)progress;
    (void)is_download_phase;
  }

  // =========================================================================
  // EXTENDED METHODS (continued) - Complex layouts
  // =========================================================================

  virtual void ShowPowerChart(const PowerDataPoint* data, size_t count) {
    (void)data;
    (void)count;
  }

  // Information display
  // Shows title + multi-line content (e.g., "PSN:" + serial number)
  // No decorative lines - keep it simple
  //
  virtual void ShowInfoScreen(const char* title, const char* content) {
    (void)title;
    (void)content;
  }

  /**
   * @brief Show charging status - intent-based, renderer decides how to display
   *
   * Animations call this with data source, renderer gets what it needs.
   * Returns suggested wake interval so animation knows when to call again.
   *
   * @param data_source Data source for port info (GetPortsData())
   * @param dt_us Delta time since last call, provided by AnimationManager
   * @return WakeInterval:
   *         - WakeInterval::Never(): Autonomous mode (FPGA POWER_METER)
   *         - WakeInterval::Polling(n): Periodic update interval
   */
  virtual WakeInterval ShowChargingStatus(IAnimationDataSource* data_source,
                                          int64_t dt_us) {
    (void)data_source;
    (void)dt_us;
    return WakeInterval::Polling(0);  // Default: continuous updates
  }

  /**
   * @brief Show idle state - intent-based, renderer decides how to display
   *
   * @param data_source Data source for WiFi/PIN/idle settings
   * @param dt_us Delta time since last call, provided by AnimationManager
   * @return WakeInterval:
   *         - WakeInterval::Never(): Autonomous mode (no periodic updates
   * needed)
   *         - WakeInterval::Polling(n): Polling/update interval for idle state
   * changes
   */
  virtual WakeInterval ShowIdleState(IAnimationDataSource* data_source,
                                     int64_t dt_us) {
    (void)data_source;
    (void)dt_us;
    return WakeInterval::CheckInterval();
  }

  // Charging display (Per-port information)
  virtual void ShowChargingPorts(const std::vector<PortChargingInfo>& ports,
                                 bool show_power) {
    (void)ports;
    (void)show_power;
  }

  // Display test patterns
  // Shows test screens for hardware validation (color tests, brightness tests)
  virtual void ShowTestPattern(DisplayTestScreen screen) { (void)screen; }

  /**
   * Get underlying display driver
   *
   * Animations use this to access hardware control methods:
   * - SetIntensity() - Control display brightness
   * - SetMode() - Turn display on/off
   * - SetRotation() - Change display orientation
   *
   * Example usage:
   *   auto* driver = GetRenderer()->GetDriver();
   *   driver->SetIntensity(50);
   *   driver->SetMode(DisplayMode::ON);
   */
  virtual IDisplayDriver* GetDriver() = 0;

  /**
   * @brief Get test capabilities if supported
   * @return Pointer to test capabilities, or nullptr if not supported
   *
   * Example usage:
   *   auto* caps = renderer->GetTestCapabilities();
   *   if (caps) {
   *     caps->SetScreen(5);
   *   } else {
   *     return ESP_ERR_NOT_SUPPORTED;
   *   }
   */
  virtual IDisplayTestCapabilities* GetTestCapabilities() { return nullptr; }

  /**
   * @brief Set current charging mode for icon display
   * @param mode Charging strategy type (StrategyType enum value)
   *
   * Called by ChargingAnimation::Entering() to set the current charging mode.
   * Renderer uses this to select appropriate mode icon (superfast, sleep, etc.)
   * Default implementation is empty for renderers that don't support this.
   */
  virtual void SetChargingMode(uint8_t mode) { (void)mode; }

  virtual ~IDisplayRenderer() = default;
};

#endif  // H_DISPLAY_RENDERER_
