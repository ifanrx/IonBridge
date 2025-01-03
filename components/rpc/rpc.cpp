#include "rpc.h"

#include "esp_err.h"

#define HARD_RESET_DELAY_MS 3000
#define SOFT_RESET_DELAY_MS 1200

esp_err_t rpc::hard_reset_mcu(uint8_t mcu) {
#ifdef CONFIG_MCU_MODEL_SW3566
  // Toggle MCU GPIO to initiate hard reset
  esp_err_t err = rpc::fpga::toggle_mcu_gpio(mcu);
  if (err == ESP_OK) {
    // Allow time for SW3566 to boot up
    DELAY_MS(HARD_RESET_DELAY_MS);
  }
  return err;
#else
  return ESP_FAIL;  // Return failure if configuration does not match
#endif
}
