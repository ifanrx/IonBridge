#pragma once

#include <climits>
#include <cstdint>

/**
 * @brief Animation timing constants
 *
 * Shared timing values used by both animation and renderer layers.
 */
namespace animation_timing {
/** Wake interval for static displays (event-driven only) */
static constexpr int64_t kNeverWake = INT64_MAX;

/** Check interval for polling-based animations (1 second) */
static constexpr int64_t kCheckIntervalUs = 1000000;

/** OTA progress update interval (500ms) */
static constexpr int64_t kOtaUpdateIntervalUs = 500000;
}  // namespace animation_timing

/**
 * @brief Wake interval for intent-based display methods
 *
 * Represents when the animation manager should next call the display method.
 * Use named constructors for clarity instead of raw int64_t values.
 */
struct WakeInterval {
  int64_t us;

  /// Autonomous mode - hardware handles updates, never wake
  static constexpr WakeInterval Never() { return {INT64_MAX}; }

  /// Custom polling interval in microseconds (default 0 = continuous)
  static constexpr WakeInterval Polling(int64_t interval_us = 0) {
    return {interval_us};
  }

  /// Default check interval (1 second)
  static constexpr WakeInterval CheckInterval() {
    return {animation_timing::kCheckIntervalUs};
  }

  /// Comparison operators
  constexpr bool operator==(const WakeInterval& other) const {
    return us == other.us;
  }
  constexpr bool operator!=(const WakeInterval& other) const {
    return us != other.us;
  }
};

namespace DisplayMode {
enum : uint8_t { OFF = 0x00, MANUAL = 0x01, POWER_METER = 0x02, MAX = 0x03 };
}

namespace DisplayFlipMode {
enum : uint8_t { NORMAL = 0x00, FLIP = 0x01, MAX = 0x02 };
}

constexpr uint8_t DisplayRotationMax = 4;  // 0, 90, 180, 270 degrees

// Image path construction macro
#define IMAGE_PATH(name) \
  (CONFIG_ANIMATION_IMAGE_DIR "/" name CONFIG_ANIMATION_IMAGE_EXT)
