#include "https_ota.h"

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_flash_partitions.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "firmware.h"
#include "ionbridge.h"
#include "sdkconfig.h"
#include "utils.h"

#define MAX_URL_LENGTH 256
#define OTA_RECV_TIMEOUT CONFIG_OTA_RECV_TIMEOUT
#define OTA_REBOOT_DELAY_MS CONFIG_OTA_REBOOT_DELAY_MS
#define OTA_FIRMWARE_TYPE 2
#define HASH_LEN 32

static const char *TAG = "OTA";
static bool in_progress = false, is_ota_timeout = false;
static esp_https_ota_handle_t https_ota_handle = NULL;
static esp_timer_handle_t ota_timer;  // Timer handle

// Abort OTA if it exceeds the time limit
void timer_callback(void *arg) {
  ESP_LOGE(TAG, "OTA timed out");
  is_ota_timeout = true;
  if (https_ota_handle != NULL) {
    esp_https_ota_abort(https_ota_handle);
  }
}

// Function to start the OTA timer
esp_err_t start_ota_timer(uint64_t timeout_ms) {
  esp_timer_create_args_t timer_args = {.callback = &timer_callback,
                                        .arg = NULL,
                                        .dispatch_method = ESP_TIMER_TASK,
                                        .name = "ota_timer",
                                        .skip_unhandled_events = true};
  // Create the timer
  esp_err_t ret = esp_timer_create(&timer_args, &ota_timer);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to create OTA timer");
  // Start the timer
  ESP_GOTO_ON_ERROR(esp_timer_start_once(ota_timer, timeout_ms * 1000), CLEANUP,
                    TAG, "Failed to start OTA timer");
  ESP_LOGI(TAG, "OTA timer started with timeout of %" PRIu32 " ms",
           (uint32_t)timeout_ms);
  return ESP_OK;
CLEANUP:
  esp_timer_delete(ota_timer);
  return ret;
}

// Function to stop the OTA timer
void stop_ota_timer() {
  if (ota_timer) {
    esp_timer_stop(ota_timer);
    esp_timer_delete(ota_timer);
    ESP_LOGI(TAG, "OTA timer stopped and deleted");
  }
}

esp_err_t ota_timer_reload(uint64_t timeout_ms) {
  ESP_RETURN_ON_FALSE(ota_timer != NULL, ESP_ERR_INVALID_STATE, TAG,
                      "OTA timer is not started.");

  ESP_RETURN_ON_ERROR(esp_timer_stop(ota_timer), TAG,
                      "Failed to stop the OTA timer.");

  ESP_RETURN_ON_ERROR(esp_timer_start_once(ota_timer, timeout_ms * 1000), TAG,
                      "Failed to restart the OTA timer.");

  ESP_LOGD(TAG, "OTA timer reloaded with a new timeout of %" PRIu32 " ms",
           static_cast<uint32_t>(timeout_ms));

  return ESP_OK;
}

static void get_sha256_of_partitions(void) {
  uint8_t sha256[HASH_LEN] = {0};
  esp_partition_t partition;

  // Calculate SHA-256 for bootloader
  partition.address = ESP_BOOTLOADER_OFFSET;
  partition.size = ESP_PARTITION_TABLE_OFFSET;
  partition.type = ESP_PARTITION_TYPE_APP;
  esp_partition_get_sha256(&partition, sha256);
  ESP_LOGI(TAG, "SHA-256 for bootloader:");
  ESP_LOG_BUFFER_HEX(TAG, sha256, HASH_LEN);

  // Calculate SHA-256 for running partition
  esp_partition_get_sha256(esp_ota_get_running_partition(), sha256);
  ESP_LOGI(TAG, "SHA-256 for running partition:");
  ESP_LOG_BUFFER_HEX(TAG, sha256, HASH_LEN);
}

static esp_err_t validate_image_header(esp_app_desc_t *new_app_info) {
  ESP_RETURN_ON_FALSE(new_app_info != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "Null argument");

  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_app_desc_t running_app_info;
  ESP_RETURN_ON_ERROR(
      esp_ota_get_partition_description(running, &running_app_info), TAG,
      "Error getting partition description");
  ESP_LOGI(TAG, "Current firmware version: %s", running_app_info.version);

  ESP_RETURN_ON_FALSE(memcmp(new_app_info->version, running_app_info.version,
                             sizeof(new_app_info->version)) != 0,
                      ESP_FAIL, TAG,
                      "Version unchanged. Update not proceeding.");
  ESP_LOGI(TAG, "New firmware version: %s", new_app_info->version);

  return ESP_OK;
}

static esp_err_t _decrypt_cb(decrypt_cb_arg_t *args, void *user_ctx) {
  static bool is_image_verified = false;
  ESP_RETURN_ON_FALSE(args != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "Invalid argument");

  args->data_out_len = args->data_in_len;
  args->data_out = (char *)malloc(args->data_out_len);
  memset(args->data_out, 0, args->data_out_len);

  ESP_RETURN_ON_ERROR(aes_cbc_decrypt(args->data_in, args->data_in_len,
                                      args->data_out, &args->data_out_len),
                      TAG, "aes_cbc_decrypt");

  ESP_LOG_BUFFER_HEX_LEVEL(TAG, args->data_out, args->data_out_len,
                           ESP_LOG_DEBUG);
  if (is_image_verified) {
    return ESP_OK;
  }

  is_image_verified = true;
  const int app_desc_offset =
      sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t);
  ESP_RETURN_ON_FALSE(
      args->data_out_len >= app_desc_offset + sizeof(esp_app_desc_t),
      ESP_ERR_OTA_VALIDATE_FAILED, TAG, "Invalid image");
  esp_app_desc_t *app_desc =
      (esp_app_desc_t *)(args->data_out + app_desc_offset);
  ESP_RETURN_ON_ERROR(validate_image_header(app_desc), TAG,
                      "validate_image_header");
  return ESP_OK;
}

esp_err_t https_ota(const char *url, const char *cert_pem, const char *curr_ver,
                    const char *new_ver) {
  int progress = 0, prev_progress = 0;
  esp_err_t ret;

  // Ensure no concurrent OTA updates
  ESP_RETURN_ON_FALSE(!in_progress, ESP_ERR_HTTPS_OTA_IN_PROGRESS, TAG,
                      "OTA in progress");

  in_progress = true;
  get_sha256_of_partitions();

  // Initialize AES CBC decryption
  ESP_RETURN_ON_ERROR(aes_cbc_decrypt_init(), TAG, "aes_cbc_decrypt_init");

  ESP_LOGI(TAG, "Starting OTA");
  esp_http_client_config_t config;
  memset(&config, 0, sizeof(config));
  config.url = url;
  config.cert_pem = cert_pem;
  config.timeout_ms = OTA_RECV_TIMEOUT;
  config.keep_alive_enable = true;
#ifdef CONFIG_SKIP_COMMON_NAME_CHECK
  config.skip_cert_common_name_check = true;
#endif

  esp_https_ota_config_t ota_config;
  memset(&ota_config, 0, sizeof(ota_config));
  ota_config.http_config = &config;
#ifdef CONFIG_ENABLE_PARTIAL_HTTP_DOWNLOAD
  ota_config.partial_http_download = true;
  ota_config.max_http_request_size = CONFIG_HTTP_REQUEST_SIZE;
#endif
#ifdef CONFIG_ESP_HTTPS_OTA_DECRYPT_CB
  ota_config.decrypt_cb = _decrypt_cb;
  ota_config.decrypt_user_ctx = NULL;
#endif

  ESP_LOGI(TAG, "Downloading firmware from %s", config.url);
  ESP_GOTO_ON_ERROR(esp_https_ota_begin(&ota_config, &https_ota_handle),
                    OTA_FAILED, TAG, "esp_https_ota_begin");
  ESP_GOTO_ON_ERROR(start_ota_timer(OTA_RECV_TIMEOUT), OTA_FAILED, TAG,
                    "start_ota_timer");

  is_ota_timeout = false;
  while (!is_ota_timeout) {
    ret = esp_https_ota_perform(https_ota_handle);
    if (ret != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
      // Exit loop on completion or error
      ESP_GOTO_ON_ERROR(ret, OTA_FAILED, TAG, "esp_https_ota_perform");
      break;
    }

    ESP_GOTO_ON_ERROR(ota_timer_reload(OTA_RECV_TIMEOUT), OTA_FAILED, TAG,
                      "ota_timer_reload");

    // Monitor download progress
    progress = esp_https_ota_get_image_len_read(https_ota_handle);
    if (std::abs(progress - prev_progress) > 100 * 1024) {
      ESP_LOGI(TAG, "Image bytes read: %d", progress);
      prev_progress = progress;
    }
  }

  if (is_ota_timeout) {
    ret = ESP_ERR_TIMEOUT;
    goto OTA_FAILED;
  }

  stop_ota_timer();
  ESP_LOGI(TAG, "Download complete, image bytes read: %d", progress);

  // Verify complete data receipt
  ESP_GOTO_ON_FALSE(esp_https_ota_is_complete_data_received(https_ota_handle),
                    ESP_ERR_NOT_FINISHED, OTA_FAILED, TAG,
                    "Full data was not received.");

  ESP_GOTO_ON_ERROR(esp_https_ota_finish(https_ota_handle), OTA_FAILED, TAG,
                    "esp_https_ota_finish");

  // Clean up and log successful completion
  aes_cbc_decrypt_deinit();
  ESP_LOGI(TAG, "HTTPS OTA upgrade successful");
  return ESP_OK;

OTA_FAILED:
  aes_cbc_decrypt_deinit();
  ESP_ERROR_COMPLAIN(ret, "OTA failed");
  log_memory_info();
  if (!(https_ota_handle == NULL || is_ota_timeout)) {
    esp_https_ota_abort(https_ota_handle);
  }
  return ret;
}
