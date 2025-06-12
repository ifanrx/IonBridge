#pragma once

#include <cstdint>

typedef enum __attribute__((packed)) {
  OFF = 0x00,
  MANUAL = 0x01,
  POWER_METER = 0x02,
} DisplayMode;

typedef enum __attribute__((packed)) {
  NORMAL = 0x00,
  FLIP = 0x01,
} DisplayFlipMode;

constexpr uint8_t MaxDisplayIntensity = 0xFF;
constexpr uint8_t MinDisplayIntensity = 0x00;
constexpr uint8_t NormalDisplayIntensity = 0x80;
