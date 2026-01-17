#ifndef SW3566_DATA_TYPES_H_
#define SW3566_DATA_TYPES_H_

#include <stdint.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include "internal/sw3566_command_types.h"  // IWYU pragma: export
#include "sdkconfig.h"

#define SW3566_IS_TYPE_A(addr) (addr == (CONFIG_SW3566_TYPE_A_PORT))
#define SW3566_MAX_POWER (CONFIG_SW3566_MAX_POWER)
#define SW3566_MIN_POWER (CONFIG_SW3566_MIN_POWER)
#define SW3566_MAX_CAP(addr) \
  (SW3566_IS_TYPE_A(addr) ? (CONFIG_SW3566_TYPE_A_MAX_POWER) : SW3566_MAX_POWER)
#define SW3566_MIN_CAP(addr) (SW3566_MIN_POWER)
// 1 * 8 (mV) * 3 (mA) * 1.023 (second) = 1uWs
#define SW3566_CHARGE_CONVERSION_FACTOR 24.552f

#endif
