#ifndef H_FPGA_DISPLAY_RENDERER_
#define H_FPGA_DISPLAY_RENDERER_

#include "display_renderer.h"
#include "fpga_display_driver.h"

#ifdef CONFIG_DISPLAY_DRIVER_FPGA

/**
 * @brief FPGA-based display renderer - Command-Based Strategy
 *
 * ## Rendering Strategy: Command-Based (Immediate Mode)
 *
 * This renderer implements a "fire and forget" command-based approach where:
 * - ESP32 sends simple commands to FPGA display hardware
 * - FPGA does all the actual rendering (progress bars, icons, animations)
 * - ESP32 has minimal CPU/memory overhead for display operations
 * - No retained UI state on ESP32 side
 *
 * **Architecture:**
 * ```
 * ESP32 (FPGARenderer)          FPGA Display Hardware
 *     |                              |
 *     | SetPercentage(75%)           |
 *     |----------------------------->|
 *     |                              | ← Renders progress bar
 *     |                              |
 *     | SetIntensity(100%)           |
 *     |----------------------------->|
 *     |                              | ← Adjusts brightness
 * ```
 *
 * ## Capabilities:
 * - Percentage display (hardware-rendered progress bars via SetPercentage())
 * - Basic icons (mapped to FPGA icon set via SetMode())
 * - Mode and intensity control (delegated to FPGADisplayDriver)
 * - Low CPU overhead (FPGA does the work)
 *
 * ## Limitations:
 * - **No arbitrary text rendering** - FPGA doesn't have font engine
 * - **Limited icon set** - Only predefined FPGA modes available
 * - **No complex layouts** - Single-element display (bar or icon)
 * - **No retained state** - Must resend commands to update display
 *
 * ## Implementation Notes:
 * - Most methods fall back to showing percentage bars (FPGA's strength)
 * - Text rendering logs warning and shows placeholder
 * - Images/charts fall back to percentage representation
 * - Driver commands are immediate (no buffering)
 *
 * @see FPGADisplayDriver for low-level FPGA communication
 * @see LVGLRenderer for comparison with retained-mode rendering
 */
class FPGARenderer : public IDisplayRenderer {
 public:
  explicit FPGARenderer(FPGADisplayDriver* driver);

  // =========================================================================
  // IDisplayRenderer implementation
  // =========================================================================

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
  WakeInterval ShowChargingStatus(IAnimationDataSource* data_source,
                                  int64_t dt_us) override;
  WakeInterval ShowIdleState(IAnimationDataSource* data_source,
                             int64_t dt_us) override;

 private:
  FPGADisplayDriver* driver_;
};

#endif  // CONFIG_DISPLAY_DRIVER_FPGA

#endif  // H_FPGA_DISPLAY_RENDERER_
