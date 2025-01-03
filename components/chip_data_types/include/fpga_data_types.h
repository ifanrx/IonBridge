#ifndef FPGA_DATA_TYPES_H_
#define FPGA_DATA_TYPES_H_

#include <stdint.h>

typedef enum {
  READ_REG = 0x0000,
  WRITE_REG = 0x0100,
} FPGAOperation;

typedef enum __attribute__((packed)) {
  ENABLE_LED = 0x00,
  DISPLAY_PERCENTAGE = 0x01,
  DISPLAY_INTENSITY = 0x02,
  DISPLAY_OR_LED = 0x03,
  DISPLAY_MODE = 0x04,
  DISPLAY_FLIP = 0x05,
  ENABLE_POWER_CONTROL = 0x10,
  P1_PRIORITY = 0x11,
  P2_PRIORITY = 0x12,
  P3_PRIORITY = 0x13,
  P4_PRIORITY = 0x14,
  P5_PRIORITY = 0x15,
  TOGGLE_SW3566_IO2 = 0x16,
  ADC_THRESHOLD_HIGH = 0x17,
  ADC_THRESHOLD_LOW = 0x18,
  ACTION_DEADZONE = 0x19,
  PORT_STATUS = 0x20,
  ADC_VALUE = 0x21,
} FPGARegAddr;

typedef struct {
  FPGARegAddr addr;
  uint8_t value;
} FPGARequest;

typedef struct {
  FPGARegAddr addr;
  uint8_t data;
} FPGAResponse;

#endif
