#ifndef __CONTORLLER_H
#define __CONTORLLER_H

#include <sys/types.h>

#include <cstdint>

#include "esp_err.h"
#include "esp_timer.h"

enum DeviceControllerStatus : uint8_t {
  POWER_UP,
  POWER_DOWN,
  WAITING_FOR_CONFIRMATION,
  REQUESTING_CONFIRMATION,
};

typedef struct {
#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
  uint8_t sw3566_version[3];
  uint8_t fpga_version[3];
#endif
  uint8_t esp32_version[3];
  bool success;
} ConfirmResult;

class DeviceController {
  DeviceControllerStatus status_;
  esp_timer_handle_t confirmation_timer_ = nullptr, request_timer_ = nullptr;
  bool upgrading_ = false;

  bool normally_booted_ = false;

  int64_t reboot_at_ = 0;

  bool associated_ = false;

 public:
  // Delete copy constructor and assignment operator
  DeviceController(DeviceController const &) = delete;
  DeviceController &operator=(DeviceController const &) = delete;
  explicit DeviceController();
  virtual ~DeviceController() = default;

  // Static method to get the singleton instance
  static DeviceController &GetInstance() {
    static DeviceController instance;
    return instance;
  }

  esp_err_t confirm();
  void try_confirm();
  void waiting_for_confirmation();
  bool is_waiting_for_confirmation() {
    return status_ == DeviceControllerStatus::WAITING_FOR_CONFIRMATION;
  }

  // Device management
  bool should_reboot();
  void reboot_after(int delay_ms = 0);
  void reboot();
  bool is_upgrading() { return upgrading_; }
  void set_upgrading(bool upgrading);

  bool is_ota_pending_verify();
  bool is_in_ota() {
    return upgrading_ ||
           status_ == DeviceControllerStatus::REQUESTING_CONFIRMATION ||
           status_ == DeviceControllerStatus::WAITING_FOR_CONFIRMATION;
  };
  void rollback();
  void try_rollback();

  // Power management
  esp_err_t power_off();
  esp_err_t power_on();
  bool is_power_on() { return status_ != POWER_DOWN; }

  void set_normally_booted(bool normally_booted) {
    normally_booted_ = normally_booted;
  }
  bool is_normally_booted() { return normally_booted_; }

  void mark_associated() { associated_ = true; }
  bool is_associated() { return associated_; }
};

#endif /* __CONTORLLER_H */
