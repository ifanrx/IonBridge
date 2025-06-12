#ifndef FIRMWARE_H_
#define FIRMWARE_H_

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "version.h"

enum FirmwareType : uint8_t {
  FIRMWARE_TYPE_FPGA = 1,
  FIRMWARE_TYPE_ESP32 = 2,
  FIRMWARE_TYPE_SW3566 = 3,
};

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t get_firmware_url(uint8_t type, const Version &version, char *url,
                           size_t url_len);
esp_err_t get_firmware_checksum_url(uint8_t type, const char *version,
                                    char *url, size_t url_len);
esp_err_t aes_cbc_decrypt_init();
void aes_cbc_decrypt_deinit();
esp_err_t aes_cbc_decrypt(const char *in, size_t in_len, char *out,
                          size_t *out_len, bool is_last = false);

#ifdef __cplusplus
}
#endif

#endif
