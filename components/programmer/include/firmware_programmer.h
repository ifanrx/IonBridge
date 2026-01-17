#ifndef FIRMWARE_PROGRAMMER_H
#define FIRMWARE_PROGRAMMER_H

#include <cstdint>
#include <cstring>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/semphr.h"
#include "uart.h"
#include "version.h"

class FirmwareProgrammer {
 public:
  static FirmwareProgrammer& GetInstance() {
    static FirmwareProgrammer instance;
    return instance;
  }

  // Delete copy constructor and assignment operator to enforce singleton
  FirmwareProgrammer(const FirmwareProgrammer&) = delete;
  FirmwareProgrammer& operator=(const FirmwareProgrammer&) = delete;

  // Clean public interface
  bool IsActivated() const { return port_id_ >= 0; }
  void Start(uint8_t port_id);
  void Stop();

  // UART message handling
  void HandleUARTMessage(const uart_message_t& uart_msg, uint8_t port_id);

  // Telemetry support (read-only access)
  Version GetProgrammerVersion() const { return version_; }
  bool IsStateDone() const { return state_done_; }
  uint8_t GetErrorReason() const { return error_reason_; }
  void SetVersion(const Version& version) { version_ = version; }
  void SetProductFamily(const char* product_family) {
    if (product_family != nullptr) {
      strncpy(product_family_, product_family, sizeof(product_family_));
      product_family_[sizeof(product_family_) - 1] = '\0';
    }
  }

 private:
  FirmwareProgrammer();
  ~FirmwareProgrammer();

  void UpdateFirmware(const Version& version, const char* product_family);
  void HandleStartMessage(uint8_t port_id);
  void HandleDoneMessage(uint8_t port_id);
  void HandleErrorMessage(uint8_t port_id, uint8_t reason);
  void StreamProgrammerStatus(bool start);

  // State variables
  bool programming_ = false;
  bool state_done_ = false;
  bool state_error_ = false;
  uint8_t error_reason_ = 0;
  int8_t port_id_ = -1;  // -1 = inactive, >=0 = active on that port
  Version version_;
  char product_family_[16] = "Unknown";

  // Thread safety
  mutable SemaphoreHandle_t mutex_;

  // Update timer
  static void UpdateFirmwareTimerCallback(void* param);
  esp_timer_handle_t update_timer_;
};

#endif  // FIRMWARE_PROGRAMMER_H
