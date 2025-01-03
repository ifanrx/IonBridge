#include "firmware.h"

#include <cstdio>
#include <string>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "machine_info.h"
#include "mbedtls/aes.h"
#include "nvs_namespace.h"
#include "sdkconfig.h"
#include "version.h"

#define FIRMWARE_DOWNLOAD_ENDPOINT CONFIG_FIRMWARE_DOWNLOAD_ENDPOINT
#define AES_BLOCK_SIZE 16

static const char *TAG = "Firmware";

typedef struct {
  mbedtls_aes_context aes_ctx;
  uint8_t aes_iv[16];
} aes_decrypt_ctx;
static aes_decrypt_ctx decrypt_ctx;

static esp_err_t get_device_aes_key(uint8_t *key, size_t *length, uint8_t *iv,
                                    size_t *iv_length) {
  if (*length != 32 || *iv_length != 16) {
    return ESP_ERR_NVS_INVALID_LENGTH;
  }
  ESP_RETURN_ON_ERROR(DeviceNVSGet(key, length, NVSKey::DEVICE_AES_KEY), TAG,
                      "get_device_aes_key");
  // Use device psn as AES IV
  const std::string &psn = MachineInfo::GetInstance().GetPSN();
  memcpy(iv, psn.c_str(), *iv_length);
  return ESP_OK;
}

esp_err_t aes_cbc_decrypt_init() {
  uint8_t aes_key[32] = {0};
  size_t aes_key_length = sizeof(aes_key);
  size_t aes_iv_length = sizeof(decrypt_ctx.aes_iv);
  ESP_RETURN_ON_ERROR(get_device_aes_key(aes_key, &aes_key_length,
                                         decrypt_ctx.aes_iv, &aes_iv_length),
                      TAG, "get_device_aes_key");
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, aes_key, aes_key_length, ESP_LOG_DEBUG);
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, decrypt_ctx.aes_iv, aes_iv_length,
                           ESP_LOG_DEBUG);

  mbedtls_aes_init(&decrypt_ctx.aes_ctx);
  mbedtls_aes_setkey_dec(&decrypt_ctx.aes_ctx, aes_key, sizeof(aes_key) * 8);
  return ESP_OK;
}

void aes_cbc_decrypt_deinit() { mbedtls_aes_free(&decrypt_ctx.aes_ctx); }

esp_err_t aes_cbc_decrypt(const char *in, size_t in_len, char *out,
                          size_t *out_len, bool is_last) {
  if (in == NULL || out == NULL || out_len == NULL) {
    ESP_LOGE(TAG, "Invalid argument");
    return ESP_ERR_INVALID_ARG;
  }
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, decrypt_ctx.aes_iv, 16, ESP_LOG_DEBUG);

  size_t block_size = *out_len;
  if (block_size < in_len) {
    ESP_LOGE(TAG, "Invalid size");
    return ESP_ERR_INVALID_SIZE;
  }
  int ret = mbedtls_aes_crypt_cbc(
      &decrypt_ctx.aes_ctx, MBEDTLS_AES_DECRYPT, in_len, decrypt_ctx.aes_iv,
      (const unsigned char *)in, (unsigned char *)out);
  if (ret != 0) {
    ESP_LOGE(TAG, "mbedtls_aes_crypt_cbc: %d", ret);
    return ESP_FAIL;
  }
  if (is_last) {
    size_t padding_len = out[in_len - 1];
    if (padding_len > AES_BLOCK_SIZE) {
      ESP_LOGE(TAG, "Invalid padding length");
      return ESP_ERR_INVALID_SIZE;
    }
    block_size -= padding_len;
  }
  *out_len = block_size;
  return ESP_OK;
}

esp_err_t get_firmware_url(uint8_t type, const Version &version, char *url,
                           size_t url_len) {
  url[0] = '\0';
  return ESP_OK;
}
