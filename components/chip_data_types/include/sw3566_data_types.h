#ifndef SW3566_DATA_TYPES_H_
#define SW3566_DATA_TYPES_H_

#include <stdint.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include "sdkconfig.h"
#include "internal/sw3566_command_types.h"  // IWYU pragma: export

#define SW3566_IS_TYPE_A(addr) (addr == (CONFIG_SW3566_TYPE_A_PORT))
#define SW3566_MAX_POWER (CONFIG_SW3566_MAX_POWER)
#define SW3566_MIN_POWER (CONFIG_SW3566_MIN_POWER)
#define SW3566_MAX_CAP(addr) \
  (SW3566_IS_TYPE_A(addr) ? (CONFIG_SW3566_TYPE_A_MAX_POWER) : SW3566_MAX_POWER)
#define SW3566_MIN_CAP(addr) (SW3566_MIN_POWER)

#endif
