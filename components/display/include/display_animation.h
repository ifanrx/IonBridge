#ifndef H_DISPLAY_ANIMATION_
#define H_DISPLAY_ANIMATION_

#include <cstdint>

#include "animation_data_source.h"
#include "display_renderer.h"
#include "singleton.h"

// Animation constants
#define POWER_CHART_POINT_COUNT 100
// Calculate frame interval from configured FPS (1 second = 1,000,000
// microseconds)
#define ANIMATION_FRAME_INTERVAL_US (1000000 / CONFIG_ANIMATION_FRAME_RATE)

// animation_timing namespace is defined in display_types.h
// (included via display_renderer.h)

/**
 * @brief OTA (Over-The-Air) update stages
 *
 * Represents the different stages of firmware download and update process.
 * Used by OTAAnimation to display progress for each stage.
 */
enum class OTAStage : uint8_t {
  DOWNLOAD_SW3566,  // Download SW3566 firmware (0-33%)
  DOWNLOAD_FPGA,    // Download FPGA firmware (34-66%)
  DOWNLOAD_ESP32,   // Download ESP32 firmware (67-100%)
  UPDATE_ERASE,     // Erase SW3566 flash (0-20%)
  UPDATE_STAGE1,    // Program SW3566 stage 1 (20-40%)
  UPDATE_STAGE2,    // Program SW3566 stage 2 (40-60%)
  UPDATE_STAGE3,    // Program SW3566 stage 3 (60-80%)
  UPDATE_STAGE4,    // Program SW3566 stage 4 (80-100%)
  COMPLETE,         // OTA complete (100%)
};

class AnimationType {
 public:
  enum Value : uint8_t {
    BOOT_APP,
    BLE_ADVERTISING,
    BLE_CONNECTED,
    START_OTA,
    OTA_ANIMATION,
    WIFI_CONNECTING,
    IDLE,
    DEVICE_INSERTED,
    DEVICE_REMOVED,
    SCREEN_OFF,
    DEVICE_INFO,
    CHARGING,
    POWER_CHART,
    DISPLAY_TEST,
    DATA_MATRIX,

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
  const char* toString() const;

 private:
  Value value;
};

/**
 * @brief Smooth value transition helper
 *
 * Encapsulates linear interpolation state for smooth animations.
 * Tracks elapsed time internally and updates via delta time.
 * Automatically captures start value when setting new target, preventing
 * coordination bugs.
 *
 * Example usage:
 * @code
 * Transition transition;
 * transition.SetTarget(100.0f);  // Animate to 100 over 1 second
 *
 * AnimationAction PlayNextFrame(int64_t dt_us) {
 *   float value = transition.Update(dt_us);
 *   // Use value...
 *   return AnimationAction::Continue(
 *       transition.IsAnimating() ? 0 : INT64_MAX);
 * }
 * @endcode
 */
struct Transition {
  /**
   * @brief Set new target value and start animation
   * @param new_target Target value to animate towards
   * @param duration_us Animation duration in microseconds (default 1 second)
   */
  void SetTarget(float new_target, int64_t duration_us = 1000000) {
    start_ = current_;
    target_ = new_target;
    duration_us_ = duration_us;
    elapsed_us_ = 0;
  }

  /**
   * @brief Update with delta time and get current interpolated value
   * @param dt_us Microseconds elapsed since last update
   * @return Current interpolated value between start and target
   */
  float Update(int64_t dt_us) {
    elapsed_us_ += dt_us;
    float t = std::min(1.0f, (float)elapsed_us_ / duration_us_);
    current_ = start_ + (target_ - start_) * t;
    return current_;
  }

  /**
   * @brief Check if animation is still in progress
   * @return true if current value hasn't reached target yet
   */
  bool IsAnimating() const { return current_ < target_; }

  /**
   * @brief Immediately set current value without animation
   * @param value The value to jump to
   */
  void SetCurrent(float value) {
    start_ = value;
    current_ = value;
    elapsed_us_ = 0;
  }

 private:
  float start_ = 0.0f;
  float current_ = 0.0f;
  float target_ = 0.0f;
  int64_t duration_us_ = 1000000;
  int64_t elapsed_us_ = 0;
};

/**
 * @brief Action command returned by animation PlayNextFrame() method
 *
 * Optimized to 16 bytes (reduced from original 32 bytes).
 * Contains transition intent + fallback policy for rejected transitions.
 */
struct AnimationAction {
  enum Status : uint8_t {
    CONTINUE = 0,   // Animation continues, call PlayNextFrame() again
    COMPLETE = 1,   // Animation finished, switch to default if not looping
    TRANSITION = 2  // Animation requests transition to specific animation
  } status = CONTINUE;

  uint8_t next_animation = 0;      // Target animation for TRANSITION
  bool next_loop = false;          // Should target animation loop?
  uint8_t fallback_animation = 0;  // Fallback if target rejected
  bool fallback_loop = false;      // Should fallback loop?
  uint8_t _reserved[3] = {0};      // Reserved for future use

  WakeInterval wake_interval = WakeInterval::Polling();

  // Helper constructors for common cases
  static AnimationAction Continue(
      WakeInterval interval = WakeInterval::Polling()) {
    return {.status = CONTINUE, .wake_interval = interval};
  }

  static AnimationAction Continue(int64_t interval_us) {
    return {.status = CONTINUE,
            .wake_interval = WakeInterval::Polling(interval_us)};
  }

  static AnimationAction Complete(
      WakeInterval interval = WakeInterval::Never()) {
    return {.status = COMPLETE, .wake_interval = interval};
  }

  static AnimationAction Transition(
      AnimationType next, bool loop = false,
      AnimationType fallback = AnimationType::NO_ANIMATIONS,
      bool fallback_loop = false,
      WakeInterval interval = WakeInterval::Polling()) {
    return {.status = TRANSITION,
            .next_animation = static_cast<uint8_t>(next),
            .next_loop = loop,
            .fallback_animation = static_cast<uint8_t>(fallback),
            .fallback_loop = fallback_loop,
            .wake_interval = interval};
  }
};

class BaseAnimation {
 protected:
  AnimationType type_;
  IAnimationDataSource* data_source_ = nullptr;

  BaseAnimation() = default;
  ~BaseAnimation() = default;

  explicit BaseAnimation(AnimationType animation_type)
      : type_(animation_type) {}

  /**
   * @brief Get the display renderer instance
   * @return Pointer to the current renderer
   *
   * Helper method to reduce boilerplate in animation implementations.
   * Replaces: DisplayManager::GetInstance().GetRenderer()
   */
  IDisplayRenderer* GetRenderer() const;

  /**
   * @brief Transition to CHARGING if ports attached, otherwise to IDLE
   * @return AnimationAction for the appropriate transition
   *
   * Common transition logic used by multiple animations after completion.
   */
  AnimationAction TransitionToChargingOrIdle() const;

 public:
  static void Init();

  void SetDataSource(IAnimationDataSource* data_source) {
    data_source_ = data_source;
  }
  IAnimationDataSource* GetDataSource() const { return data_source_; }

  virtual void Entering() {};

  /**
   * @brief Play next animation frame with delta time
   *
   * @param dt_us Microseconds elapsed since last frame
   * @return Animation action with wake interval
   *         - WakeInterval::Polling(): Continuous (manager uses frame rate)
   *         - WakeInterval::Never(): Never wake (static animation)
   *         - WakeInterval::Polling(n): Specific wake interval
   */
  virtual AnimationAction PlayNextFrame(int64_t dt_us) {
    return AnimationAction::Continue(WakeInterval::Never());
  }

  virtual void Exiting() {};

  AnimationType Type() const { return type_; }

  /**
   * @brief Get fallback animation when this one is rejected
   *
   * Called by AnimationManager when CanBePlayed() returns false.
   * Override in animations that have a meaningful fallback.
   *
   * @return Fallback animation type, or NO_ANIMATIONS to stay at current
   */
  virtual AnimationType GetFallbackAnimation() const {
    return AnimationType::NO_ANIMATIONS;  // Default: no fallback
  }

  /**
   * @brief Check if this animation can be interrupted by another animation
   *
   * Critical animations (factory mode, OTA updates) override this to control
   * which animations can interrupt them.
   *
   * @param incoming Animation type trying to interrupt
   * @return true if interruption allowed (default), false to block
   */
  virtual bool CanBeInterrupted(AnimationType incoming) const {
    (void)incoming;  // Suppress unused parameter warning
    return true;     // Default: can be interrupted by anything
  }

  /**
   * @brief Check if this animation can be played on current product family
   *
   * Override in animations that should be skipped for certain product families.
   * When false, AnimationManager will use GetFallbackAnimation() instead.
   *
   * Called once per animation switch (not per frame), so runtime check is fine.
   *
   * @return true if animation can play (default), false to skip
   */
  virtual bool CanBePlayed() const { return true; }
};

/**
 * @brief Null object pattern for invalid/no animation state
 *
 * NoAnimation provides fail-safe behavior when:
 * - Animation system is initializing
 * - Invalid animation type is requested
 * - NO_ANIMATIONS sentinel value is used
 *
 * This replaces nullptr checks throughout AnimationManager with a proper
 * null object that does nothing but does it safely. Matches v0.8.39 behavior.
 */
class NoAnimation : public BaseAnimation, public Singleton<NoAnimation> {
 public:
  void Entering() override {}
  void Exiting() override {}
  AnimationAction PlayNextFrame(int64_t dt_us) override {
    return AnimationAction::Continue(WakeInterval::Never());
  }

 private:
  friend class Singleton<NoAnimation>;
  NoAnimation() : BaseAnimation(AnimationType::NO_ANIMATIONS) {}
};

/**
 * @brief Base class for multi-stage animations
 *
 * Eliminates boilerplate code for animations that cycle through
 * multiple stages with fixed durations. Subclasses implement 3 simple
 * virtual methods instead of using std::function callbacks.
 *
 * Example usage:
 * @code
 * class MyAnimation : public StagedAnimation {
 *  protected:
 *   size_t GetStageCount() const override { return 2; }
 *   int64_t GetStageDuration(size_t index) const override {
 *     return index == 0 ? 2000000 : 3000000;  // 2s, 3s
 *   }
 *   void RenderStage(size_t index) override {
 *     if (index == 0) ShowStage1();
 *     else ShowStage2();
 *   }
 * };
 * @endcode
 *
 * The base class handles:
 * - Automatic stage advancement based on duration
 * - Looping back to stage 0 after last stage
 * - Delta time tracking per stage
 */
class StagedAnimation : public BaseAnimation {
 protected:
  /** @brief Get total number of stages */
  virtual size_t GetStageCount() const = 0;

  /** @brief Get duration in microseconds for a given stage index */
  virtual int64_t GetStageDuration(size_t index) const = 0;

  /** @brief Render the given stage index */
  virtual void RenderStage(size_t index) = 0;

  void Entering() override;
  AnimationAction PlayNextFrame(int64_t dt_us) override;

 private:
  size_t current_stage_ = 0;
  int64_t stage_elapsed_us_ = 0;
};

/**
 * @brief Base class for linear progress animations
 *
 * Handles animations that progress from 0.0 to 1.0 over a fixed duration,
 * then transition to another animation. Eliminates boilerplate code for
 * simple progress-based animations.
 *
 * Subclasses implement:
 * - GetDurationUs(): Total animation duration
 * - RenderProgress(float): Render current progress [0.0, 1.0]
 * - OnComplete(): Return transition action when progress reaches 1.0
 *
 * Example:
 * @code
 * class MyProgressAnimation : public ProgressAnimation {
 *  protected:
 *   int64_t GetDurationUs() const override { return 2000 * 1000; }
 *   void RenderProgress(float p) override {
 *     GetRenderer()->ShowMyProgress(p);
 *   }
 *   AnimationAction OnComplete() override {
 *     return AnimationAction::Transition(AnimationType::NEXT, false);
 *   }
 * };
 * @endcode
 */
class ProgressAnimation : public BaseAnimation {
 protected:
  virtual int64_t GetDurationUs() const = 0;
  virtual void RenderProgress(float progress) = 0;
  virtual AnimationAction OnComplete() = 0;

 public:
  void Entering() override;
  AnimationAction PlayNextFrame(int64_t dt_us) override;
  void Exiting() override;

 private:
  int64_t elapsed_us_ = 0;
};

class BootAppAnimation : public ProgressAnimation,
                         public Singleton<BootAppAnimation> {
 protected:
  int64_t GetDurationUs() const override { return 3000 * 1000; }
  void RenderProgress(float progress) override;
  AnimationAction OnComplete() override;

  // Boot animation cannot be interrupted by BLE advertising
  bool CanBeInterrupted(AnimationType incoming) const override { return false; }

 private:
  friend class Singleton<BootAppAnimation>;
  BootAppAnimation() { type_ = AnimationType::BOOT_APP; }
};

/**
 * @brief BLE Advertising Animation
 *
 * Displays static bluetooth advertising state.
 * Shows advertising icon to indicate device is discoverable.
 */
class BleAdvertisingAnimation : public BaseAnimation,
                                public Singleton<BleAdvertisingAnimation> {
 public:
  void Entering() override;
  AnimationAction PlayNextFrame(int64_t dt_us) override;
  bool CanBePlayed() const override;

 private:
  friend class Singleton<BleAdvertisingAnimation>;
  BleAdvertisingAnimation() : BaseAnimation(AnimationType::BLE_ADVERTISING) {}

  int64_t elapsed_us_ = 0;
};

/**
 * @brief BLE Connected Animation
 *
 * Breathing pattern: 10s ON, 60s OFF, repeat.
 * Shows connection state briefly to save power.
 */
class BleConnectedAnimation : public StagedAnimation,
                              public Singleton<BleConnectedAnimation> {
 protected:
  size_t GetStageCount() const override { return 2; }
  int64_t GetStageDuration(size_t index) const override;
  void RenderStage(size_t index) override;

 private:
  friend class Singleton<BleConnectedAnimation>;
  BleConnectedAnimation() { type_ = AnimationType::BLE_CONNECTED; }
};

class WifiConnectingAnimation : public BaseAnimation,
                                public Singleton<WifiConnectingAnimation> {
 public:
  void Entering() override;
  AnimationAction PlayNextFrame(int64_t dt_us) override;
  void Exiting() override;

  bool CanBeInterrupted(AnimationType incoming) const override {
    // WiFi connecting animation can be interrupted by:
    // - IDLE (when WiFi connects or fails)
    // - DEVICE_INSERTED / CHARGING (when device is connected)
    // - OTA animations (higher priority)
    // - Display test (QA mode)
    // - Screen off (user command)
    return incoming == AnimationType::IDLE ||
           incoming == AnimationType::DEVICE_INSERTED ||
           incoming == AnimationType::CHARGING ||
           incoming == AnimationType::START_OTA ||
           incoming == AnimationType::OTA_ANIMATION ||
           incoming == AnimationType::DISPLAY_TEST ||
           incoming == AnimationType::SCREEN_OFF;
  }
  bool CanBePlayed() const override;

 private:
  friend class Singleton<WifiConnectingAnimation>;
  WifiConnectingAnimation() : BaseAnimation(AnimationType::WIFI_CONNECTING) {}

  // Internal stage management
  enum class Stage { GREETING, WIFI_CONNECTING };
  Stage current_stage_ = Stage::GREETING;
  int64_t stage_elapsed_us_ = 0;

  // Stage duration configuration
  static constexpr int64_t kGreetingDurationUs = 1500 * 1000;  // 1.5 seconds
  static constexpr int64_t kConnectingCycleDurationUs =
      1000 * 1000;  // 1 second/cycle
};

class IdleAnimation : public BaseAnimation, public Singleton<IdleAnimation> {
 public:
  void Entering() override;
  AnimationAction PlayNextFrame(int64_t dt_us) override;
  void Exiting() override;

 private:
  friend class Singleton<IdleAnimation>;
  IdleAnimation() : BaseAnimation(AnimationType::IDLE) {}
};

/**
 * @brief Device Inserted Animation
 *
 * Shows greeting animation when a device is plugged in.
 * Displays for 2 seconds, then transitions to charging animation.
 *
 * Restarts automatically if a new device is inserted on a different port
 * while the animation is still playing.
 */
class DeviceInsertedAnimation : public ProgressAnimation,
                                public Singleton<DeviceInsertedAnimation> {
 public:
  void Entering() override;
  AnimationAction PlayNextFrame(int64_t dt_us) override;

  bool CanBeInterrupted(AnimationType incoming) const override {
    // Don't allow IDLE to interrupt device greeting animation
    return incoming != AnimationType::IDLE;
  }
  bool CanBePlayed() const override;

  AnimationType GetFallbackAnimation() const override {
    return AnimationType::CHARGING;
  }

 protected:
  int64_t GetDurationUs() const override;
  void RenderProgress(float progress) override;
  AnimationAction OnComplete() override;

 private:
  friend class Singleton<DeviceInsertedAnimation>;
  DeviceInsertedAnimation() { type_ = AnimationType::DEVICE_INSERTED; }

  uint8_t stored_port_id_ = 0;  // Port ID captured at animation start
};

/**
 * @brief Device Removed Animation
 *
 * Shows "unattached" image when a device is unplugged.
 * Displays for configured duration, then transitions to charging (if other
 * ports active) or idle animation.
 */
class DeviceRemovedAnimation : public BaseAnimation,
                               public Singleton<DeviceRemovedAnimation> {
 public:
  void Entering() override;
  AnimationAction PlayNextFrame(int64_t dt_us) override;
  void Exiting() override;

  bool CanBePlayed() const override;

  AnimationType GetFallbackAnimation() const override {
    // Dynamic fallback based on port state
    if (data_source_ && data_source_->GetAttachedPortCount() > 0) {
      return AnimationType::CHARGING;
    }
    return AnimationType::IDLE;
  }

 private:
  friend class Singleton<DeviceRemovedAnimation>;
  DeviceRemovedAnimation() : BaseAnimation(AnimationType::DEVICE_REMOVED) {}

  int64_t elapsed_us_ = 0;
};

class PowerChartAnimation : public BaseAnimation,
                            public Singleton<PowerChartAnimation> {
 public:
  void Entering() override;
  AnimationAction PlayNextFrame(int64_t dt_us) override;
  void Exiting() override;

  bool CanBePlayed() const override;
  AnimationType GetFallbackAnimation() const override;

 private:
  friend class Singleton<PowerChartAnimation>;
  PowerChartAnimation() : BaseAnimation(AnimationType::POWER_CHART) {}

  // Animation state
  int64_t elapsed_us_ = 0;
  int64_t port_update_elapsed_us_ = 0;
  int64_t power_update_elapsed_us_ = 0;
  uint8_t port_id_ = 0;

  // Power data for chart rendering
  PowerDataPoint power_data_[POWER_CHART_POINT_COUNT] = {};
  size_t power_data_count_ = 0;

  // Helper methods
  void RenderChart();
  void UpdatePowerData();
};

class ScreenOffAnimation : public BaseAnimation,
                           public Singleton<ScreenOffAnimation> {
 public:
  void Entering() override;
  AnimationAction PlayNextFrame(int64_t dt_us) override {
    return AnimationAction::Continue(WakeInterval::Never());
  };
  void Exiting() override;

 private:
  friend class Singleton<ScreenOffAnimation>;
  ScreenOffAnimation() : BaseAnimation(AnimationType::SCREEN_OFF) {}
};

class DeviceInfoAnimation : public StagedAnimation,
                            public Singleton<DeviceInfoAnimation> {
 public:
  void Exiting() override;

  bool CanBePlayed() const override;
  AnimationType GetFallbackAnimation() const override {
    return AnimationType::IDLE;
  }

 protected:
  size_t GetStageCount() const override { return 2; }
  int64_t GetStageDuration(size_t index) const override;
  void RenderStage(size_t index) override;

 private:
  friend class Singleton<DeviceInfoAnimation>;
  DeviceInfoAnimation() { type_ = AnimationType::DEVICE_INFO; }

  void ShowPSN();
  void ShowMac();
  void ShowInfo(bool show_psn);
};

class ChargingAnimation : public BaseAnimation,
                          public Singleton<ChargingAnimation> {
 public:
  void Entering() override;
  AnimationAction PlayNextFrame(int64_t dt_us) override;
  void Exiting() override;

  bool CanBeInterrupted(AnimationType incoming) const override {
    // Allow IDLE only when no ports are attached (device removal)
    if (incoming == AnimationType::IDLE) {
      return data_source_ && data_source_->GetAttachedPortCount() == 0;
    }
    return true;
  }

 private:
  friend class Singleton<ChargingAnimation>;
  ChargingAnimation() : BaseAnimation(AnimationType::CHARGING) {}

  /**
   * @brief Render charging status via intent-based API
   * @param dt_us Delta time since last call
   * @return Suggested wake interval from renderer
   */
  WakeInterval RenderChargingStatus(int64_t dt_us = 0);

  int64_t elapsed_us_ = 0;
  uint8_t last_port_count_ = 0;
};

class DisplayTestAnimation : public BaseAnimation,
                             public Singleton<DisplayTestAnimation> {
 public:
  void Entering() override;
  AnimationAction PlayNextFrame(int64_t dt_us) override;
  void Exiting() override;

 private:
  friend class Singleton<DisplayTestAnimation>;
  friend class LVGLTestCapabilities;
  friend class GPUTestCapabilities;
  DisplayTestAnimation() : BaseAnimation(AnimationType::DISPLAY_TEST) {}

  // Hardware-agnostic rendering helper
  void ShowTestPattern(DisplayTestScreen screen);

  // State tracking (hardware-agnostic)
  bool auto_mode_ = false;
  bool loop_once_ = false;
  bool completed_ = false;
  uint16_t auto_duration_ms = 0;
  int64_t elapsed_us_ = 0;
};

class StartOTAAnimation : public BaseAnimation,
                          public Singleton<StartOTAAnimation> {
 public:
  void Entering() override;
  AnimationAction PlayNextFrame(int64_t dt_us) override;
  void Exiting() override;

 private:
  friend class Singleton<StartOTAAnimation>;
  StartOTAAnimation() : BaseAnimation(AnimationType::START_OTA) {}

  static constexpr int64_t kDisplayDurationMs =
      500;                     // Duration before auto-transition
  int64_t expired_at_us_ = 0;  // Absolute time when animation should transition
};

/**
 * @brief Data Matrix image display animation
 *
 * Displays a 96x32 pixel monochrome Data Matrix image from littlefs
 * when connecting to the default WiFi SSID. The image (dm.bin) is
 * loaded and uploaded to the GPU display chip via RPC.
 *
 * Trigger: WiFi connects to default SSID with DEVICE_DATA_MATRIX_FLAG set
 * Duration: 5 seconds
 */
class DataMatrixAnimation : public BaseAnimation,
                            public Singleton<DataMatrixAnimation> {
 public:
  void Entering() override;
  AnimationAction PlayNextFrame(int64_t dt_us) override;
  void Exiting() override;
  bool CanBeInterrupted(AnimationType incoming) const override;

 private:
  friend class Singleton<DataMatrixAnimation>;
  DataMatrixAnimation() : BaseAnimation(AnimationType::DATA_MATRIX) {}

  bool dm_missing_ = false;
};

/**
 * @brief Consolidated OTA (Over-The-Air) update animation
 *
 * Replaces 8 separate OTA animation types (DOWNLOAD_STAGE1-3,
 * UPDATE_3566_STAGE0-4) with a single animation that manages
 * internal stages via SetStage().
 *
 * External code (upgrade.cpp, sw3566.cpp) controls stage progression
 * by calling SetStage() with the appropriate OTAStage enum value.
 *
 * Benefits:
 * - Type-safe stage management (no enum arithmetic)
 * - Single animation type instead of 8
 * - Hardware-agnostic (uses renderer interface)
 * - Eliminates AnimationTransitionRules for OTA stages
 */
class OTAAnimation : public BaseAnimation, public Singleton<OTAAnimation> {
 public:
  /**
   * @brief Set the current OTA stage
   *
   * Updates the displayed progress bar to reflect the current stage.
   * Should be called by external code (upgrade.cpp, sw3566.cpp) to
   * advance through the OTA process.
   *
   * @param stage The OTA stage to display
   */
  void SetStage(OTAStage stage);

  void Entering() override;
  AnimationAction PlayNextFrame(int64_t dt_us) override;
  void Exiting() override;

 private:
  friend class Singleton<OTAAnimation>;
  OTAAnimation() : BaseAnimation(AnimationType::OTA_ANIMATION) {}

  /**
   * @brief Get progress percentage for a given stage
   *
   * Maps OTAStage enum values to progress percentages (0-100).
   *
   * @param stage The OTA stage
   * @return Progress percentage (0-100)
   */
  static uint8_t GetProgressPercentage(OTAStage stage);

  /** @brief Check if stage is a download stage (SW3566/FPGA/ESP32) */
  static bool IsDownloadStage(OTAStage stage);

  /** @brief Check if stage is an update stage (erase/program) */
  static bool IsUpdateStage(OTAStage stage);

  OTAStage stage_ = OTAStage::DOWNLOAD_SW3566;
  Transition transition_;  // Encapsulates smooth interpolation state
};

#endif /* H_DISPLAY_ANIMATION_ */
