#ifndef UART_RPC_H_
#define UART_RPC_H_

#include <cstddef>
#include <cstdint>

#include "data_types.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include "version.h"

#if CONFIG_MCU_MODEL_SW3566
#include "sw3566_data_types.h"
#elif CONFIG_MCU_MODEL_FAKE_SW3566
#include "fake_sw3566_data_types.h"  // IWYU pragma: keep
#endif

#if CONFIG_DISPLAY_DRIVER_GPU
#include "gpu_data_types.h"
#endif

namespace rpc {
namespace mcu {
#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
esp_err_t send_command(uint8_t mcu, uint16_t command, const uint8_t *request,
                       uint8_t request_size, uint8_t *response,
                       uint8_t *response_size);
esp_err_t keep_alive(uint8_t mcu, KeepAliveStatus *status,
                     uint32_t *uptime_ms = nullptr,
                     uint32_t *reboot_reason = nullptr);

/* Bootloader only */
esp_err_t boot(uint8_t mcu);
esp_err_t reset_system(uint8_t mcu, bool wait_for_response = false);
/* Bringing up */
esp_err_t bringup(uint8_t mcu, bool skip_upgrade = false,
                  const Version *programmer_version = nullptr,
                  bool *upgraded = nullptr);
esp_err_t boot_all(bool *booted, uint8_t count, bool skip_upgrade = false,
                   bool *any_upgraded = nullptr);

/* User application only */
esp_err_t set_subscription(uint8_t mcu, const Subscriptions &subs);
esp_err_t set_system_flags(uint8_t mcu, SystemFlags *flags);
#endif

esp_err_t set_port_state(uint8_t mcu, bool shutdown);
esp_err_t port_connect(uint8_t mcu);
esp_err_t port_disconnect(uint8_t mcu);
esp_err_t set_max_power_budget(uint8_t mcu, uint8_t budget, uint8_t watermark,
                               bool force_rebroadcast,
                               uint8_t *set_budget = nullptr);
esp_err_t get_max_power_budget(uint8_t mcu, uint8_t *budget);
esp_err_t get_pd_status(uint8_t mcu, ClientPDStatus *status);
esp_err_t get_port_details(uint8_t mcu, PortDetails *details);
esp_err_t set_power_features(uint8_t mcu, PowerFeatures features);
esp_err_t get_power_features(uint8_t mcu, PowerFeatures *features);
esp_err_t set_custom_pdo_voltage(uint8_t mcu, uint16_t voltage_mv);

};  // namespace mcu

#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
namespace fpga {
esp_err_t toggle_power_control(bool enable = true);
esp_err_t set_mcu_priority(uint8_t mcu, uint8_t priority);
esp_err_t read_adc_value(uint8_t *value);
esp_err_t toggle_mcu_gpio(uint8_t mcu);
esp_err_t set_adc_threshold(uint8_t low, uint8_t high);
esp_err_t set_action_deadzone(uint8_t deadzone);

esp_err_t set_display_percentage(uint8_t percentage = 0);
esp_err_t set_display_mode(uint8_t mode = 0);
esp_err_t set_display_intensity(uint8_t intensity = 0);
esp_err_t set_display_flip_mode(uint8_t mode = 0);
};  // namespace fpga
#endif

#ifdef CONFIG_DISPLAY_DRIVER_GPU
namespace gpu {
esp_err_t set_backlight_intensity(uint8_t intensity);
esp_err_t set_display_status(bool on);
esp_err_t set_animation_index(GPUAnimationIndex animation_index,
                              PreCompositeMode pre_composite_mode,
                              PostCompositeMode post_composite_mode);
esp_err_t set_animation_data(const SetAnimationDataRequest &data);
esp_err_t draw_text(const DrawTextRequest &request);
esp_err_t prepare_framebuffer(const PrepareFramebufferRequest &request);
esp_err_t set_framebuffer_chunk(const SetFramebufferRequest &request);
};  // namespace gpu
#endif

esp_err_t hard_reset_mcu(uint8_t mcu);
esp_err_t soft_reset_mcu(uint8_t mcu, bool wait_for_response = true);
};  // namespace rpc

#endif
