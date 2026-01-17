#ifndef H_ANIMATION_MANAGER_
#define H_ANIMATION_MANAGER_

#include "animation_data_source.h"
#include "display_animation.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/semphr.h"
#include "freertos/task.h"

/**
 * @brief Animation Manager - Animation Lifecycle & Task Coordination
 *
 * ## Thread Safety Model
 *
 * AnimationManager runs a dedicated FreeRTOS task and uses mutex protection
 * for all shared state access:
 *
 * **Protected State (mutex_ guards):**
 * - `animation_` - Current animation pointer
 * - `loop_` - Current loop flag
 * - `next_animation_type_` - Queued animation type
 * - `next_animation_loop_` - Queued loop flag
 *
 * **Thread-Safe Public Methods:**
 * - `SetNextAnimation()` - Can be called from any task (mutex protected)
 * - `StopAnimation()` - Can be called from any task (mutex protected)
 * - `SetAnimation()` - Can be called from any task (mutex protected)
 * - `SetOTAStage()` - Can be called from any task (uses SetAnimation)
 * - `NotifyUpdate()` - Can be called from any task (only wakes task)
 *
 * **Task Synchronization:**
 * - Animation task runs in continuous loop with smart sleep
 * - External threads call SetNextAnimation() to queue transitions
 * - NotifyUpdate() wakes task immediately via xTaskNotifyGive()
 * - Task checks `running_` flag on each iteration for graceful shutdown
 *
 * **Initialization Requirements:**
 * - Must call StartTask() before any other methods
 * - Public methods check for null mutex and return early if not initialized
 * - Destructor calls StopTask() to ensure cleanup before mutex deletion
 *
 * **Data Source Threading:**
 * - All animations share ONE AnimationDataSource instance (`data_source_`)
 * - Data source methods query singletons (PortManager, MachineInfo, WiFi)
 * - Data source returns by value to avoid lifetime issues
 * - Animations only access data source from animation task (single-threaded)
 *
 * ## Usage Example
 *
 * ```cpp
 * // Initialization (main task)
 * AnimationManager::GetInstance().StartTask();
 *
 * // Queue animation from any task (e.g., port handler)
 * AnimationManager::GetInstance().SetNextAnimation(
 *     AnimationType::CHARGING, true);
 *
 * // Immediate switch from any task (e.g., OTA handler)
 * AnimationManager::GetInstance().SetAnimation(
 *     AnimationType::OTA_ANIMATION, false);
 *
 * // Wake task immediately
 * AnimationManager::GetInstance().NotifyUpdate();
 * ```
 */
class AnimationManager {
 public:
  AnimationManager() = default;
  ~AnimationManager();

  static AnimationManager &GetInstance();
  esp_err_t StartTask();
  void StopTask();

  /**
   * @brief Queue the next animation to play
   *
   * Thread-safe method to queue an animation change. The animation task will
   * switch on its next loop iteration.
   *
   * @param animation_type The animation type to queue
   * @param loop Whether the animation should loop
   *
   * @note No-op if task is not running (e.g., after StopTask/Deinit)
   */
  void SetNextAnimation(AnimationType animation_type, bool loop);

  /**
   * @brief Stop the specified animation if currently playing
   *
   * If the specified animation is currently active, switches to IDLE_ANIMATION.
   *
   * @param animation_type The animation type to stop
   *
   * @note No-op if task is not running (e.g., after StopTask/Deinit)
   */
  void StopAnimation(AnimationType animation_type);

  /**
   * @brief Notify the animation task to wake up and check for updates
   *
   * Call this when external events require immediate animation updates
   * (e.g., port events, button presses).
   */
  void NotifyUpdate();

  /**
   * @brief Set the OTA animation stage
   *
   * Convenience method for OTA updates. If not already in OTA mode, starts
   * START_OTA animation which will auto-transition to OTA_ANIMATION after
   * a configurable duration. Updates the stage and wakes the animation task.
   *
   * @param stage The OTA stage to display
   *
   * @note No-op if task is not running (e.g., after StopTask/Deinit)
   * @note Safe to call after PrepareForOTA() - will be ignored for LVGL
   */
  void SetOTAStage(OTAStage stage);

  /**
   * @brief Get the current animation type
   *
   * @return Current animation type, or NO_ANIMATIONS if none
   */
  AnimationType GetCurrentAnimationType() const;

 private:
  BaseAnimation *animation_ = nullptr;
  bool loop_ = false;
  AnimationType next_animation_type_ = AnimationType::NO_ANIMATIONS;
  bool next_animation_loop_ = false;

  TaskHandle_t task_handle_;
  bool running_ = false;
  SemaphoreHandle_t mutex_ = nullptr;
  AnimationDataSource data_source_;  // Data source for all animations
  int64_t last_frame_time_ = 0;      // Last frame timestamp for delta time

  static void TaskLoopWrapper(void *arg);
  void TaskLoop();

  /**
   * @brief Set animation immediately (no checks, assumes valid)
   *
   * Private method that performs the actual animation switch.
   * All validation (CanBePlayed, fallback) done by SetNextAnimation.
   */
  void SetAnimation(AnimationType type, bool loop);

  /**
   * @brief Handle transition action from PlayNextFrame
   *
   * Processes TRANSITION results with product family check and fallback.
   */
  void HandleTransition(const AnimationAction &action);

  BaseAnimation *GetAnimationInstance(AnimationType animation_type);

  /**
   * @brief Check if animation is critical and should bypass display-off state
   *
   * Critical animations (OTA, factory test) should still play even when
   * display mode is OFF. Non-critical animations are rejected when display
   * is intentionally off.
   *
   * @param type Animation type to check
   * @return true if animation should bypass display-off check
   */
  bool IsCriticalAnimation(AnimationType type) const;
};

#endif /* H_ANIMATION_MANAGER_ */
