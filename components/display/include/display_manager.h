#ifndef H_DISPLAY_MANAGER_
#define H_DISPLAY_MANAGER_

#include <cstdint>

#include "display_animation.h"
#include "display_driver.h"
#include "display_renderer.h"
#include "display_test.h"
#include "esp_err.h"

/**
 * @brief Display Manager - Orchestration & State Management Layer
 *
 * ## Role in Architecture
 *
 * DisplayManager is the orchestration layer that sits above the animation
 * system and provides a stable public API for other components. It manages:
 *
 * ```
 * ┌─────────────────────────────────────────────────────────┐
 * │  Other Components (Port, Power, BLE, MQTT, etc.)        │
 * │  - Call DisplayManager API to trigger display changes   │
 * └────────────────┬────────────────────────────────────────┘
 *                  ↓
 * ┌─────────────────────────────────────────────────────────┐
 * │  DisplayManager (Orchestration & State) ← THIS CLASS    │
 * │  - NVS state persistence (brightness, mode, rotation)   │
 * │  - Display state management and lifecycle coordination  │
 * │  - Public API for system components                     │
 * │  - Test orchestration via TestDisplay()                 │
 * └────────────────┬────────────────────────────────────────┘
 *                  ↓
 * ┌─────────────────────────────────────────────────────────┐
 * │  AnimationManager (Animation Transitions & Lifecycle)   │
 * │  - Owns AnimationDataSource for data injection          │
 * │  - FreeRTOS task loop for animation execution           │
 * │  IDisplayRenderer (Rendering Strategy)                  │
 * │  IDisplayDriver (Hardware Control)                      │
 * └─────────────────────────────────────────────────────────┘
 * ```
 *
 * ## Responsibilities
 *
 * 1. **State Management**: Tracks persistent display state with NVS backing
 *    - `display_mode_` - Current display mode
 *    - `display_intensity_` - Brightness level 0-100%
 *    - `display_rotation_` - Screen orientation
 *    - `display_flip_mode_` - Flip mode
 *    - `display_testing_` - Test mode flag
 * 2. **Lifecycle**: Initializes driver, renderer, and AnimationManager task
 * 3. **Public API**: Single entry point for all display operations
 * 4. **Renderer Access**: Provides GetRenderer() for animations to use
 * 5. **Testing**: Orchestrates display test mode via runtime capability
 * discovery
 *
 * ## State Persistence with NVS
 *
 * DisplayManager automatically loads and saves display state to NVS:
 *
 * **On Init()**:
 * - Loads saved state from NVS namespace `DISPLAY_DATA`
 * - Keys: `DISPLAY_MODE`, `DISPLAY_INTENSITY`, `DISPLAY_ROTATION`
 * - Falls back to config defaults if no saved state exists
 * - Migrates legacy `DISPLAY_FLIP` values to `DISPLAY_ROTATION` if found
 *
 * **On State Change**:
 * - Methods like SetDisplayMode() update both runtime state and driver
 * - State changes are persisted automatically (implementation-dependent)
 *
 * ## Initialization Flow
 *
 * ```cpp
 * // In main.cpp:
 * static FPGADisplayDriver driver;
 * static FPGARenderer renderer(&driver);
 * DisplayManager::GetInstance().Init(&driver, &renderer);
 *
 * // DisplayManager.Init() internally:
 * // 1. Load saved state from NVS
 * DISPLAY_DATA.GetOrDefault(nvs_keys::DISPLAY_MODE, display_mode_);
 * DISPLAY_DATA.GetOrDefault(nvs_keys::DISPLAY_INTENSITY, display_intensity_);
 * // 2. Initialize hardware
 * driver_->Init();
 * renderer_->Init();
 * // 3. Start AnimationManager task (owns AnimationDataSource)
 * AnimationManager::GetInstance().StartTask();
 *
 * // Animations access renderer via DisplayManager singleton:
 * auto* renderer = DisplayManager::GetInstance().GetRenderer();
 * ```
 *
 * ## Thread Safety
 *
 * **Threading Model:**
 * - **DisplayManager**: Singleton with no internal locking (stateless getters)
 * - **AnimationManager**: Dedicated FreeRTOS task with mutex protection
 * - **Driver/Renderer**: Hardware abstraction with driver-specific locking
 *
 * **Thread-Safe Operations:**
 * - `GetRenderer()` - Read-only, returns pointer (safe from any task)
 * - `GetMode()`, `GetIntensity()`, `GetRotation()` - Read-only getters
 * - `SetMode()`, `SetIntensity()`, `SetRotation()` - Delegate to driver
 *   (driver handles thread safety if needed)
 * - `DisplayOn()`, `DisplayOff()` - Delegate to SetMode() (driver-safe)
 * - `TestDisplay()` - Calls AnimationManager (mutex protected)
 *
 * **Renderer Thread Safety:**
 * - **LVGLRenderer**: Uses `bsp_display_lock()` (mutex) for all LVGL calls
 * - **FPGARenderer**: UART RPC has internal locking
 * - **GPURenderer**: UART RPC has internal locking
 * - **StubRenderer**: No-op, inherently thread-safe
 *
 * **State Consistency:**
 * - Display state (`display_mode_`, `display_intensity_`, etc.) only modified
 *   by calling task (no concurrent modification)
 * - State updates are atomic (single assignment after driver success)
 * - No race conditions because state is read/write from control tasks only
 *
 * **Typical Call Pattern:**
 * ```cpp
 * // From main task or command handler (safe)
 * DisplayManager::GetInstance().SetIntensity(80);
 *
 * // From animation task (safe via AnimationManager mutex)
 * auto* r = DisplayManager::GetInstance().GetRenderer();
 * r->ShowText("Hello", style);  // Renderer handles its own locking
 * ```
 *
 * ## Why Keep DisplayManager?
 *
 * While renderer and driver provide the display abstraction, DisplayManager
 * serves distinct purposes:
 * - **Single Point of Control**: Other components call one API, not multiple
 *   subsystems
 * - **State Persistence**: Display state outlives individual animations,
 * persisted to NVS
 * - **Lifecycle Management**: Coordinates initialization of all display
 * subsystems
 * - **Test Orchestration**: Centralized entry point for display testing
 * - **Backward Compatibility**: Existing code relies on this API
 *
 * ## Usage Examples
 *
 * ### Example 1: Basic Initialization (from main.cpp)
 *
 * ```cpp
 * #include "display_manager.h"
 * #include "fpga_display_driver.h"
 * #include "fpga_display_renderer.h"
 *
 * // Static driver and renderer instances
 * static FPGADisplayDriver g_display_driver;
 * static FPGARenderer g_display_renderer(&g_display_driver);
 *
 * void initialize_display() {
 *   DisplayManager& manager = DisplayManager::GetInstance();
 *
 *   // Initialize display subsystem
 *   ESP_ERROR_CHECK(manager.Init(&g_display_driver, &g_display_renderer));
 *
 *   // Optional: apply saved settings
 *   manager.SetDisplayRotation(manager.GetDisplayRotation());
 *   manager.SetDisplayIntensity(manager.GetDisplayIntensity());
 * }
 * ```
 *
 * ### Example 2: Triggering Animations from Other Components
 *
 * ```cpp
 * #include "display_manager.h"
 * #include "animation_manager.h"
 *
 * // In port_handler.cpp or similar:
 * void on_device_attached(uint8_t port_id) {
 *   // Trigger charging animation
 *   AnimationManager::GetInstance().SetNextAnimation(
 *       AnimationType::CHARGING, true);
 * }
 *
 * void on_ota_start() {
 *   // Immediately switch to OTA animation (no queue)
 *   AnimationManager::GetInstance().SetAnimation(
 *       AnimationType::OTA_ANIMATION, false);
 * }
 * ```
 *
 * ### Example 3: Animation Using Renderer
 *
 * ```cpp
 * #include "display_manager.h"
 * #include "display_animation.h"
 *
 * class MyAnimation : public BaseAnimation {
 *  public:
 *   void Entering() override {
 *     // Get renderer via DisplayManager singleton
 *     auto* r = DisplayManager::GetInstance().GetRenderer();
 *     r->Clear();
 *     r->ShowText("Starting...", {TextStyle::Size::LARGE,
 *                                  Alignment::CENTER});
 *   }
 *
 *   AnimationResult Play() override {
 *     auto* r = DisplayManager::GetInstance().GetRenderer();
 *
 *     // Visual content via renderer
 *     r->ShowPercentage(75, ProgressStyle::NORMAL);
 *
 *     // Hardware control via driver
 *     r->GetDriver()->SetIntensity(100);
 *
 *     return {AnimationResult::CONTINUE};
 *   }
 * };
 * ```
 *
 * ### Example 4: Display Control from User Commands
 *
 * ```cpp
 * #include "display_manager.h"
 *
 * // BLE command handler:
 * void handle_brightness_command(uint8_t brightness) {
 *   DisplayManager::GetInstance().SetDisplayIntensity(brightness);
 * }
 *
 * // Power management:
 * void enter_low_power_mode() {
 *   DisplayManager::GetInstance().DisplayOff();
 * }
 *
 * void exit_low_power_mode() {
 *   DisplayManager::GetInstance().DisplayOn();  // Restores previous mode
 * }
 * ```
 *
 * ### Example 5: Display Testing
 *
 * ```cpp
 * #include "display_manager.h"
 * #include "display_test.h"
 *
 * void start_display_test() {
 *   DisplayTestEvent event = {
 *     .kind = DisplayTestKind::TEST_ENTER,
 *     .param = nullptr
 *   };
 *
 *   esp_err_t ret = DisplayManager::GetInstance().TestDisplay(event);
 *   if (ret == ESP_ERR_NOT_SUPPORTED) {
 *     ESP_LOGW(TAG, "Display testing not supported by current renderer");
 *   }
 * }
 *
 * void set_test_brightness(uint8_t brightness) {
 *   DisplayTestParam param = { .lightness = brightness };
 *   DisplayTestEvent event = {
 *     .kind = DisplayTestKind::TEST_LIGHTNESS,
 *     .param = &param
 *   };
 *   DisplayManager::GetInstance().TestDisplay(event);
 * }
 *
 * void exit_display_test() {
 *   DisplayTestEvent event = {
 *     .kind = DisplayTestKind::TEST_EXIT,
 *     .param = nullptr
 *   };
 *   DisplayManager::GetInstance().TestDisplay(event);
 * }
 * ```
 *
 * @see AnimationManager for animation transition logic
 * @see IAnimationDataSource for data injection pattern enabling testability
 * @see IDisplayRenderer for rendering abstraction
 * @see IDisplayDriver for hardware control
 * @see docs/display_component.md for comprehensive architecture documentation
 */
class DisplayManager {
 public:
  /**
   * @brief Get the singleton instance of DisplayManager
   *
   * @return DisplayManager& Reference to the singleton instance
   */
  static DisplayManager &GetInstance();

  /**
   * @brief Initialize the display subsystem
   *
   * Initialization sequence:
   * 1. Loads saved display state from NVS (mode, intensity, flip, rotation)
   * 2. Initializes the display driver hardware
   * 3. Initializes the renderer (fonts, resources, etc.)
   * 4. Starts the AnimationManager FreeRTOS task (unless skip_animations=true)
   *
   * @param driver Pointer to display driver implementation (must not be null)
   * @param renderer Pointer to renderer implementation (must not be null)
   * @param skip_animations If true, skip starting animation task (for devices
   *        missing animation resources)
   * @return esp_err_t
   *   - ESP_OK on success
   *   - ESP_ERR_INVALID_ARG if driver or renderer is null
   *   - Driver/renderer specific errors on initialization failure
   *
   * @note Must be called before any other DisplayManager operations
   * @note Driver and renderer pointers must remain valid for lifetime of
   * program
   *
   * @see display_manager.cpp:56-77 for implementation details
   */
  esp_err_t Init(IDisplayDriver *driver, IDisplayRenderer *renderer,
                 bool skip_animations = false);

  /**
   * @brief Deinitialize the display subsystem
   *
   * Cleanup sequence:
   * 1. Stops the AnimationManager task
   * 2. Deinitializes the display driver
   *
   * @return esp_err_t
   *   - ESP_OK on success
   *   - Driver-specific errors on deinitialization failure
   *
   * @note Typically called during system shutdown or OTA updates
   */
  esp_err_t Deinit();

  /**
   * @brief Stop animation task immediately
   */
  void Stop();

  /**
   * @brief Prepare display for OTA update
   *
   * Selectively deinitializes display based on driver memory requirements:
   * - Memory-intensive drivers (LVGL): Calls Deinit() to free resources
   * - Lightweight drivers (FPGA, GPU): Keeps running to show OTA animations
   *
   * When display is deinitialized (LVGL case):
   * - AnimationManager task stops
   * - Subsequent SetAnimation/SetOTAStage calls are ignored silently
   * - Display resources freed for OTA operations
   *
   * @return esp_err_t
   *   - ESP_OK on success
   *   - Driver-specific errors on deinitialization failure
   *
   * @note Hardware-agnostic: caller doesn't need driver-specific knowledge
   * @note Safe to call SetOTAStage() after this - will no-op if deinitialized
   * @see IDisplayDriver::IsMemoryIntensive() for driver classification
   */
  esp_err_t PrepareForOTA();

  /**
   * @brief Reset display to default configuration values
   *
   * Resets display state to compile-time defaults and saved rotation:
   * - Display mode → CONFIG_DISPLAY_MODE
   * - Display intensity → CONFIG_DISPLAY_INTENSITY
   * - Display rotation → loaded from NVS (driver-agnostic)
   *
   * @return esp_err_t
   *   - ESP_OK on success
   *   - Errors from SetMode(), SetIntensity(), etc.
   *
   * @note Useful for factory reset scenarios
   * @note Does not clear NVS, only resets runtime state
   * @note SetRotation() may return ESP_ERR_NOT_SUPPORTED for unsupported angles
   *
   * @see display_manager.cpp:84-100 for implementation
   */
  esp_err_t Reset();

  /**
   * @brief Set display percentage directly
   *
   * Directly controls percentage display on hardware that supports it (e.g.,
   * FPGA). This bypasses the animation system and directly commands the
   * driver.
   *
   * @param percentage Percentage value (0-100)
   * @return esp_err_t
   *   - ESP_OK on success
   *   - ESP_ERR_NOT_SUPPORTED if driver doesn't support percentage display
   *   - ESP_ERR_INVALID_ARG if percentage > 100
   *
   * @note This is a legacy method, prefer using animations for display updates
   * @note Delegates directly to driver->SetPercentage()
   */
  esp_err_t SetPercentage(uint8_t percentage);

  /**
   * @brief Set display mode
   *
   * Changes the display mode (e.g., on/off, different display modes).
   * Updates internal state only if driver call succeeds.
   *
   * State tracking:
   * - Stores previous mode in `previous_display_mode_` for DisplayOn()
   * - Updates `display_mode_` only on successful driver operation
   * - Skips driver call if mode already matches current state
   *
   * @param mode Display mode value (hardware-specific, e.g., 0=off, 1=on)
   * @return esp_err_t
   *   - ESP_OK on success (or if mode already set)
   *   - Driver-specific errors on failure
   *
   * @see DisplayMode enum in display_types.h for standard mode values
   * @see DisplayOn() and DisplayOff() for convenience wrappers
   * @see display_manager.cpp:106-116 for implementation
   */
  esp_err_t SetMode(uint8_t mode);

  /**
   * @brief Set display brightness intensity
   *
   * Controls display brightness/backlight. Updates internal state only if
   * driver call succeeds.
   *
   * @param intensity Brightness level (0-100%)
   *   - 0 = minimum brightness (may be off)
   *   - 100 = maximum brightness
   * @return esp_err_t
   *   - ESP_OK on success
   *   - ESP_ERR_INVALID_ARG if intensity > 100
   *   - Driver-specific errors on failure
   *
   * @note State is persisted to NVS (implementation-dependent)
   * @note Actual brightness mapping is hardware-specific (driver
   * implementation)
   * @see display_manager.cpp:118-124 for implementation
   */
  esp_err_t SetIntensity(uint8_t intensity);

  /**
   * @brief Reapply current display mode to hardware
   *
   * Useful for:
   * - Restoring mode after hardware reset
   * - Synchronizing state with hardware
   *
   * @return esp_err_t Result from SetMode(display_mode_)
   */
  esp_err_t SetMode() { return SetMode(display_mode_); }

  /**
   * @brief Reapply current display intensity to hardware
   *
   * Useful for:
   * - Restoring brightness after hardware reset
   * - Synchronizing state with hardware
   *
   * @return esp_err_t Result from SetIntensity(display_intensity_)
   */
  esp_err_t SetIntensity() { return SetIntensity(display_intensity_); }

  /**
   * @brief Set display rotation
   *
   * Controls display orientation rotation (0, 90, 180, 270 degrees).
   * Hardware support varies:
   * - FPGA: Supports 0° and 180° only (mapped to flip register)
   * - LVGL: Supports all rotations (0°, 90°, 180°, 270°)
   *
   * @param rotation Rotation value in degrees (0, 90, 180, 270)
   * @return esp_err_t
   *   - ESP_OK on success
   *   - ESP_ERR_NOT_SUPPORTED if driver doesn't support the requested rotation
   *   - ESP_ERR_INVALID_ARG if rotation value is invalid
   *   - Driver-specific errors on failure
   *
   * @note Updates `display_rotation_` on success
   * @note Driver decides which rotation angles are supported
   */
  esp_err_t SetRotation(uint8_t rotation);
  /**
   * @brief Get current display mode
   *
   * @return uint8_t Current display mode value
   */
  uint8_t GetMode() const { return display_mode_; }

  /**
   * @brief Get current display brightness intensity
   *
   * @return uint8_t Current intensity (0-100%)
   */
  uint8_t GetIntensity() const { return display_intensity_; }

  /**
   * @brief Get current display rotation
   *
   * @return uint8_t Current rotation value (0, 90, 180, 270 degrees)
   */
  uint8_t GetRotation() const { return display_rotation_; }

  /**
   * @brief Get the renderer instance for animations to use
   *
   * Animations should call this method to access the renderer:
   *
   * ```cpp
   * void MyAnimation::Play() {
   *   auto* r = DisplayManager::GetInstance().GetRenderer();
   *   r->ShowText("Hello", TextStyle{});
   *   r->GetDriver()->SetIntensity(100);
   * }
   * ```
   *
   * @return IDisplayRenderer* Pointer to renderer instance (never null after
   * Init)
   *
   * @note Thread-safe: returns pointer, doesn't modify state
   * @note Renderer pointer is valid for lifetime of program
   * @see IDisplayRenderer for rendering interface documentation
   */
  IDisplayRenderer *GetRenderer() const { return renderer_; }

  /**
   * @brief Execute display test commands
   *
   * Orchestrates display testing through runtime capability discovery.
   * Delegates actual test operations to IDisplayTestCapabilities interface
   * if supported by current renderer.
   *
   * Test flow:
   * 1. Checks if renderer supports testing via GetTestCapabilities()
   * 2. For TEST_ENTER: starts DISPLAY_TEST animation, saves original
   * intensity
   * 3. For TEST_EXIT: stops test animation, restores original intensity
   * 4. For other events: delegates to test capabilities interface
   *
   * Supported test events:
   * - TEST_ENTER: Enter test mode
   * - TEST_EXIT: Exit test mode
   * - TEST_SCREEN: Switch to specific test screen (manual)
   * - TEST_LIGHTNESS: Adjust brightness during test
   * - TEST_AUTO: Enable automatic screen cycling (infinite loop)
   * - TEST_AUTO_ONCE: Auto-cycle once through all screens
   *
   * @param event Display test event with kind and optional parameters
   * @return esp_err_t
   *   - ESP_OK on success
   *   - ESP_ERR_NOT_SUPPORTED if renderer doesn't support testing
   *   - ESP_ERR_INVALID_STATE if event received in wrong state
   *   - ESP_ERR_INVALID_ARG for unknown test kind
   *
   * @note Uses runtime capability discovery (no compile-time #ifdef)
   * @note Test state managed via display_testing_ flag
   * @see IDisplayTestCapabilities for test interface
   * @see DisplayTestEvent in display_test.h for event types
   * @see display_manager.cpp:153-211 for implementation
   */
  esp_err_t TestDisplay(const DisplayTestEvent &event);

  /**
   * @brief Queue the next animation to play.
   *
   * Thread-safe wrapper that forwards to AnimationManager. The animation
   * task will switch on its next loop iteration.
   */
  void SetNextAnimation(AnimationType animation_type, bool loop = false);

  /**
   * @brief Stop the animation if it is currently active.
   */
  void StopAnimation(AnimationType animation_type);

  /**
   * @brief Update the OTA animation stage while ensuring animation task wakes.
   */
  void SetOTAStage(OTAStage stage);

  /**
   * @brief Turn display on
   *
   * Restores display to previous mode if currently off.
   * If display is already on, does nothing (returns ESP_OK).
   *
   * State behavior:
   * - If current mode is DisplayMode::OFF, switches to previous_display_mode_
   * - Otherwise, no operation (already on)
   *
   * @return esp_err_t
   *   - ESP_OK on success or if already on
   *   - Errors from SetMode()
   *
   * @see DisplayOff() to turn display off
   * @see display_manager.cpp:142-147 for implementation
   */
  esp_err_t DisplayOn();

  /**
   * @brief Turn display off
   *
   * Switches display mode to DisplayMode::OFF, preserving previous mode
   * for restoration via DisplayOn().
   *
   * @return esp_err_t Result from SetMode(DisplayMode::OFF)
   *
   * @note Previous mode is saved automatically in SetMode()
   * @see DisplayOn() to restore display
   * @see display_manager.cpp:149-151 for implementation
   */
  esp_err_t DisplayOff();

 private:
  DisplayManager();
  ~DisplayManager() = default;

  // Display driver (compile-time selected)
  IDisplayDriver *driver_;
  IDisplayRenderer *renderer_;

  uint8_t display_mode_, previous_display_mode_, display_intensity_,
      display_rotation_ = 0;
  bool display_testing_ = false;
  uint8_t test_original_intensity_ = 0;
  AnimationType test_previous_animation_ = AnimationType::IDLE;

  // singleton
  DisplayManager(const DisplayManager &) = delete;
  DisplayManager &operator=(const DisplayManager &) = delete;
};

#endif /* H_DISPLAY_MANAGER_ */
