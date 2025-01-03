#ifndef __CONTORLLER_H
#define __CONTORLLER_H

#include <sys/types.h>

#include <cstdint>

#include "esp_err.h"
#include "esp_timer.h"
#include "power_allocator.h"

#define DISPLAY_DEFAULT_MODE CONFIG_DISPLAY_MODE
#define DISPLAY_DEFAULT_INTENSITY CONFIG_DISPLAY_INTENSITY

enum DeviceControllerStatus : uint8_t {
  POWER_UP,
  POWER_DOWN,
  WAITING_FOR_CONFIRMATION,
  REQUESTING_CONFIRMATION,
};

class DeviceController {
  DeviceControllerStatus status_;
  esp_timer_handle_t confirmation_timer_ = nullptr, request_timer_ = nullptr;
  bool upgrading_ = false;
  uint8_t display_intensity_ = DISPLAY_DEFAULT_INTENSITY;
  uint8_t display_mode_ = DISPLAY_DEFAULT_MODE;

  bool normally_booted_ = false;

  int64_t reboot_at_ = 0;

  void start_request_timer(int timeout_ms);
  void start_confirmation_timer(int timeout_ms);

 public:
  // Delete copy constructor and assignment operator
  DeviceController(DeviceController const &) = delete;
  DeviceController &operator=(DeviceController const &) = delete;
  explicit DeviceController();
  virtual ~DeviceController() = default;

  // Static method to get the singleton instance
  static DeviceController *GetInstance() {
    static DeviceController instance;
    return &instance;
  }

  esp_err_t confirm(const uint8_t *hash, size_t hash_len = 32);
  void waiting_for_confirmation();
  bool is_waiting_for_confirmation() {
    return status_ == DeviceControllerStatus::WAITING_FOR_CONFIRMATION;
  }

  // Device management
  bool should_reboot();
  void reboot_after(int delay_ms = 0);
  void reboot();
  int get_uptime() { return esp_timer_get_time(); }
  bool is_upgrading() { return upgrading_; }
  void set_upgrading(bool upgrading) {
    upgrading_ = upgrading;
    if (upgrading_) {
      PowerAllocator &pAllocator = PowerAllocator::GetInstance();
      pAllocator.GetPortManager().ShutdownAllPorts();
      pAllocator.Stop();
    }
  }
  bool is_in_ota() {
    return upgrading_ ||
           status_ == DeviceControllerStatus::REQUESTING_CONFIRMATION ||
           status_ == DeviceControllerStatus::WAITING_FOR_CONFIRMATION;
  };

  // Power management
  esp_err_t power_off();
  esp_err_t power_on();
  bool is_power_on() { return status_ != POWER_DOWN; }

  esp_err_t set_display_intensity(uint8_t intensity);
  uint8_t get_display_intensity() { return display_intensity_; }
  esp_err_t set_display_mode(uint8_t mode);
  uint8_t get_display_mode() { return display_mode_; }

  void set_normally_booted(bool normally_booted) {
    normally_booted_ = normally_booted;
  }
  bool is_normally_booted() { return normally_booted_; }
};

#endif /* __CONTORLLER_H */
