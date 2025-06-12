#ifndef ACDC_H
#define ACDC_H

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

void reset_acdc(void);
void adjust_voltage(uint8_t requester, uint16_t request_mv);
void request_low_voltage();

#ifdef __cplusplus
}
#endif

#endif  // ACDC_H
