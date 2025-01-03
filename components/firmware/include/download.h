#ifndef FIRMWARE_DOWNLOAD_H_
#define FIRMWARE_DOWNLOAD_H_

#include "esp_err.h"
#include "firmware.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t download_firmware(const char *url, const char *cert_pem,
                            const char *curr_ver, const char *new_ver,
                            FirmwareType type);

#ifdef __cplusplus
}
#endif

#endif
