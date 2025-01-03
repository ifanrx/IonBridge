#ifndef FIRMWARE_UPGRADE_H_
#define FIRMWARE_UPGRADE_H_

#include <cstdint>

#include "esp_err.h"
#include "sdkconfig.h"

typedef struct {
#ifdef CONFIG_MCU_MODEL_SW3566
  uint8_t sw3566_version[3];
  uint8_t fpga_version[3];
#endif
  uint8_t esp32_version[3];
  bool force;
} firmware_version;

esp_err_t start_upgrade_task(const firmware_version *new_version);

#endif  // FIRMWARE_UPGRADE_H_
