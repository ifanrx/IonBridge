#ifndef H_HTTPS_OTA_
#define H_HTTPS_OTA_

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t https_ota(const char *url, const char *cert_pem, const char *curr_ver,
                    const char *new_ver);

#ifdef __cplusplus
}
#endif

#endif  // !H_HTTPS_OTA_
