#ifndef H_ANIMATION_DATA_SOURCE_
#define H_ANIMATION_DATA_SOURCE_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "display_renderer.h"  // For PortChargingInfo

/**
 * @brief Interface for providing data to animations
 *
 * This interface decouples animations from business logic singletons
 * (PortManager, MachineInfo, WiFi), enabling unit testing with mock
 * data sources while maintaining compatibility with internal animation
 * transitions.
 *
 * ## Design Rationale
 *
 * Follows Linus's principle: "Good taste is no special cases"
 * - Animations query this interface instead of singletons directly
 * - Real implementation queries singletons (production)
 * - Mock implementation returns test data (unit tests)
 * - Single injection point at AnimationManager level
 * - Internal transitions work seamlessly (all animations share context)
 *
 * ## Thread Safety Model
 *
 * **Calling Context:**
 * - All methods are called ONLY from AnimationManager's animation task
 * - Single-threaded access pattern (no concurrent calls)
 * - No internal locking required in implementations
 *
 * **Data Ownership:**
 * - Methods return by value (copy/move) to avoid lifetime issues
 * - Caller owns returned data (animations store locally if needed)
 * - No shared mutable state between calls
 *
 * **Thread Safety of Underlying Singletons:**
 * - PortManager: Thread-safe (uses FreeRTOS mutexes internally)
 * - MachineInfo: Read-only after initialization (inherently thread-safe)
 * - WiFi: Uses esp-idf thread-safe APIs
 *
 * **Why Return By Value:**
 * Previous implementation returned `const reference` to cached data, which
 * caused lifetime issues when animation switching occurred mid-execution.
 * Returning by value eliminates dangling references at minimal cost (RVO/move
 * optimization makes copies cheap).
 *
 * ## Usage Pattern
 *
 * ```cpp
 * // In animation Play() method (runs in animation task)
 * void MyAnimation::Play() {
 *   // Safe: data source only called from animation task
 *   auto ports = data_source_->GetPortsData();  // Returns by value
 *
 *   // Process data locally (owns the vector)
 *   for (const auto& port : ports) {
 *     // ...
 *   }
 * }
 * ```
 */
class IAnimationDataSource {
 public:
  virtual ~IAnimationDataSource() = default;

  /**
   * @brief Get number of attached ports
   * @return Count of ports with attached devices
   *
   * Used by: IdleAnimation, ChargingAnimation
   */
  virtual uint8_t GetAttachedPortCount() = 0;

  /**
   * @brief Get charging data for all attached ports
   * @return Vector of port charging info (by value)
   *
   * Used by: ChargingAnimation
   * Note: Returns by value to avoid lifetime issues. RVO/move optimization
   *       makes this efficient in practice.
   */
  virtual std::vector<PortChargingInfo> GetPortsData() = 0;

  /**
   * @brief Get total power consumption across all ports
   * @return Total power in watts
   *
   * Used by: IdleAnimation (power tier), ChargingAnimation, PowerChartAnimation
   * Note: Convenience method to avoid repeated summation of GetPortsData()
   */
  virtual uint16_t GetTotalWatts() = 0;

  /**
   * @brief Get product serial number
   * @return PSN string (typically 16 characters)
   *
   * Used by: DeviceInfoAnimation
   */
  virtual std::string GetPSN() = 0;

  /**
   * @brief Get BLE MAC address
   * @return 6-byte MAC address array
   *
   * Used by: DeviceInfoAnimation
   */
  virtual std::array<uint8_t, 6> GetBleMac() = 0;

  /**
   * @brief Get 4-digit PIN code
   * @return 4-byte PIN array
   *
   * Used by: IdleAnimation
   */
  virtual std::array<uint8_t, 4> GetPin() = 0;

  /**
   * @brief Get product family name
   * @return Product family string (e.g., CP02, CP02s, PA768s)
   *
   * Used by: Animations that gate playback per product family
   */
  virtual std::string GetProductFamily() const = 0;

  /**
   * @brief Check if WiFi is connected
   * @return true if connected with valid IP, false otherwise
   *
   * Used by: IdleAnimation
   */
  virtual bool IsWifiConnected() = 0;

  /**
   * @brief Get historical power data aggregated across all ports
   * @param max_points Maximum number of data points to return
   * @return Vector of historical power data points (timestamp + power)
   *
   * Used by: PowerChartAnimation
   * Note: Aggregates power consumption across all active ports over time
   */
  virtual std::vector<PowerDataPoint> GetHistoricalPowerData(
      size_t max_points) = 0;

  /**
   * @brief Get the ID of the last inserted port
   * @return Port ID (0-based index)
   *
   * Used by: DeviceInsertedAnimation
   * Note: Set via SetLastInsertedPortId() before triggering animation
   */
  virtual uint8_t GetLastInsertedPortId() = 0;

  /**
   * @brief Get configured idle animation type
   * @return IdleAnimationType enum value (0=NONE, 1=METEOR, 2=GAME_OF_LIFE)
   *
   * Used by: IdleAnimation
   */
  virtual uint8_t GetIdleAnimationType() = 0;
};

/**
 * @brief Real implementation that queries actual system state
 *
 * This implementation queries PortManager, MachineInfo, and WiFi
 * singletons to provide real-time data to animations. Use this
 * in production.
 */
class AnimationDataSource : public IAnimationDataSource {
 public:
  AnimationDataSource() = default;
  ~AnimationDataSource() override = default;

  uint8_t GetAttachedPortCount() override;
  std::vector<PortChargingInfo> GetPortsData() override;
  uint16_t GetTotalWatts() override;
  std::string GetPSN() override;
  std::array<uint8_t, 6> GetBleMac() override;
  std::array<uint8_t, 4> GetPin() override;
  std::string GetProductFamily() const override;
  bool IsWifiConnected() override;
  std::vector<PowerDataPoint> GetHistoricalPowerData(
      size_t max_points) override;
  uint8_t GetLastInsertedPortId() override;
  uint8_t GetIdleAnimationType() override;

  // Static setters for cross-component access
  static void SetLastInsertedPortId(uint8_t port_id);
  static void SetIdleAnimation(uint8_t type);
};

#endif  // H_ANIMATION_DATA_SOURCE_
