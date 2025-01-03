#include "download.h"

#include <strings.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "download.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "firmware.h"
#include "sdkconfig.h"
#include "storage.h"

#define FIRMWARE_DOWNLOAD_RECV_TIMEOUT CONFIG_FIRMWARE_DOWNLOAD_RECV_TIMEOUT
#define FIRMWARE_DOWNLOAD_RECV_BUFFER_SIZE \
  CONFIG_FIRMWARE_DOWNLOAD_RECV_BUFFER_SIZE
#define DOWNLOAD_TASK_STACK_SIZE CONFIG_FIRMWARE_DOWNLOAD_TASK_STACK_SIZE
#define DOWNLOAD_TASK_PRIORITY CONFIG_FIRMWARE_DOWNLOAD_TASK_PRIORITY
#define DOWNLOAD_DATA_BUFFER_SIZE CONFIG_FIRMWARE_DOWNLOAD_DATA_BUFFER_SIZE

#define SW3566_FIRMWARE_FILE_PREFIX "SW3566"
#define FPGA_FIRMWARE_FILE_PREFIX "FPGA"

static const char *TAG = "OTADownload";
static bool download_completed = false;

typedef struct {
  const char *temp_path;
  const char *final_path;
  FirmwareType type;
} download_config_t;

typedef struct data_buffer {
  size_t length;
  size_t offset;
  bool full;
  char data[];
} *data_buffer_t;

data_buffer_t init_data_buffer(size_t length) {
  // Allocate memory for the structure plus the data array
  data_buffer_t buffer =
      (data_buffer_t)malloc(sizeof(struct data_buffer) + length * sizeof(char));
  if (buffer == NULL) {
    return NULL;  // Memory allocation failed
  }

  // Initialize the structure fields
  buffer->length = length;
  buffer->offset = 0;
  buffer->full = false;
  memset(buffer->data, 0,
         length);  // Optionally initialize the data array to zero

  return buffer;
}

void deinit_data_buffer(data_buffer_t buffer) {
  if (buffer != NULL) {
    free(buffer);
  }
}

size_t populate_data_buffer(data_buffer_t buffer, const char *data,
                            size_t data_length) {
  // Calculate the available space in the buffer
  size_t available_space = buffer->length - buffer->offset;

  // Determine how many bytes we can actually write to the buffer
  size_t bytes_to_write =
      data_length <= available_space ? data_length : available_space;

  // Copy the data to the buffer
  memcpy(buffer->data + buffer->offset, data, bytes_to_write);
  buffer->offset += bytes_to_write;

  // Determine if the buffer is full
  if (buffer->offset == buffer->length) {
    buffer->offset = 0;
    buffer->full = true;
  }

  // Calculate remaining bytes that couldn't be written
  return data_length - bytes_to_write;
}

esp_err_t decrypt_and_write_to_file(FILE *file, const char *data,
                                    size_t *length, bool is_final_block) {
  esp_err_t __attribute__((unused)) ret;
  size_t written = 0;

  if (file == NULL || data == NULL) {
    ESP_LOGE(TAG, "Invalid parameters");
    return ESP_ERR_INVALID_ARG;
  }

  // Decrypt the data block
  size_t decrypted_length = *length;
  char *output = (char *)calloc(decrypted_length, sizeof(char));
  if (output == NULL) {
    ESP_LOGE(TAG, "calloc failed");
    return ESP_ERR_NO_MEM;
  }

  ESP_GOTO_ON_ERROR(
      aes_cbc_decrypt(data, *length, output, &decrypted_length, is_final_block),
      FREE_AND_RETURN, TAG, "aes_cbc_decrypt");

  if (decrypted_length > 0) {
    written = fwrite(output, 1, decrypted_length, file);
  }
  free(output);
  if (written != decrypted_length) {
    ESP_LOGE(TAG, "Failed to write to file, written %d, expected %d", written,
             decrypted_length);
    return ESP_FAIL;
  }
  *length = decrypted_length;
  return ESP_OK;
FREE_AND_RETURN:
  free(output);
  return ESP_FAIL;
}

// Event handler for HTTP client
esp_err_t http_event_handler(esp_http_client_event_t *evt) {
  static FILE *file = NULL;
  static int total_bytes_written = 0, padding_length = 0, content_length = 0,
             status_code = -1;
  static data_buffer_t buffer = NULL;
  download_config_t *config = (download_config_t *)evt->user_data;

  switch (evt->event_id) {
    case HTTP_EVENT_ON_DATA: {
      if (status_code == -1) {
        status_code = esp_http_client_get_status_code(evt->client);
      }
      if (status_code != 200) {
        ESP_LOGE(TAG, "Non-200 status code received: %d", status_code);
        return ESP_FAIL;
      }
      ESP_LOGD(TAG, "Received data, length = %d", evt->data_len);
      // Populate the data buffer
      if (buffer == NULL) {
        buffer = init_data_buffer(DOWNLOAD_DATA_BUFFER_SIZE);
        if (buffer == NULL) {
          ESP_LOGE(TAG, "Memory allocation (%d) for data buffer failed.",
                   DOWNLOAD_DATA_BUFFER_SIZE);
          return ESP_ERR_NO_MEM;
        }
      }

      // Write the decrypted data block to the temporary file
      if (!file) {
        file = fopen(config->temp_path, "w");
        if (!file) {
          ESP_LOGE(TAG, "Failed to open file for writing: %s (%s)",
                   config->temp_path, strerror(errno));
          return ESP_FAIL;
        }
        total_bytes_written = 0;
      }

      // Because the server response doesn't contain the content length, we need
      // to check the block length to determine if it's the final block
      bool is_final_block = (total_bytes_written + buffer->offset +
                             evt->data_len) == content_length;
      size_t remaining_length = evt->data_len;
      const char *data_ptr = (const char *)evt->data;
      size_t written = 0;
      while (remaining_length > 0) {
        size_t new_remaining_length =
            populate_data_buffer(buffer, data_ptr, remaining_length);
        data_ptr += remaining_length - new_remaining_length;
        remaining_length = new_remaining_length;
        if (buffer->full) {
          written = buffer->length;
          ESP_RETURN_ON_ERROR(decrypt_and_write_to_file(
                                  file, buffer->data, &written,
                                  is_final_block && (remaining_length == 0)),
                              TAG, "decrypt_and_write_to_file");
          if (written < buffer->length) {
            padding_length = buffer->length - written;
          }
          total_bytes_written += written;
          buffer->full = false;
        }
      }

      if (is_final_block && buffer->offset > 0) {
        written = buffer->offset;
        ESP_RETURN_ON_ERROR(
            decrypt_and_write_to_file(file, buffer->data, &written, true), TAG,
            "decrypt_and_write_to_file");
        if (written < buffer->offset) {
          padding_length = buffer->offset - written;
        }
        total_bytes_written += written;
      }
      break;
    }
    case HTTP_EVENT_ON_FINISH: {
      if (file) {
        fclose(file);
        file = NULL;
      }
      if (status_code != 200) {
        break;
      }
      ESP_LOGI(TAG,
               "Download finished. Content length: %d, Padding length: %d, "
               "Total bytes written: %d",
               content_length, padding_length, total_bytes_written);
      download_completed =
          total_bytes_written == content_length - padding_length;
      break;
    }
    case HTTP_EVENT_DISCONNECTED:
      ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
      if (file) {
        fclose(file);
        file = NULL;
      }
      if (buffer != NULL) {
        deinit_data_buffer(buffer);
        buffer = NULL;
      }
      total_bytes_written = 0;
      padding_length = 0;
      content_length = 0;
      status_code = -1;
      break;
    case HTTP_EVENT_ON_HEADER:
      ESP_LOGD(TAG, "HTTP event on header: key=%s, value=%s", evt->header_key,
               evt->header_value);
      // Get HTTP header grpc-metadata-content-length
      if (strcasecmp(evt->header_key, "grpc-metadata-content-length") == 0) {
        content_length = atoi(evt->header_value);
      }
      break;
    default:
      break;
  }
  return ESP_OK;
}

// Download a file and save to SPIFFS
esp_err_t download_firmware(const char *url, const char *cert_pem,
                            const char *curr_ver, const char *new_ver,
                            FirmwareType type) {
  esp_err_t ret = ESP_OK;
  esp_http_client_config_t client_config;
  esp_http_client_handle_t client = NULL;
  int status_code;
  const char *file_prefix;
  char temp_path[32], temp_filename[16], path[32];
  download_config_t cfg;
  download_completed = false;

  // Check firmware type
  ESP_GOTO_ON_FALSE(type == FIRMWARE_TYPE_SW3566 || type == FIRMWARE_TYPE_FPGA,
                    ESP_ERR_INVALID_ARG, CLEANUP, TAG, "Invalid firmware type");

  // Construct the temp file path and the final path
  file_prefix = (type == FIRMWARE_TYPE_SW3566) ? SW3566_FIRMWARE_FILE_PREFIX
                                               : FPGA_FIRMWARE_FILE_PREFIX;
  snprintf(temp_filename, sizeof(temp_filename), "%s.bin", file_prefix);
  Storage::GetTempFilePath(temp_filename, temp_path);
  ESP_GOTO_ON_ERROR(
      Storage::GetFirmwarePath(file_prefix, new_ver, path, sizeof(path)),
      CLEANUP, TAG, "GetFirmwarePath: %s (%s)", file_prefix, new_ver);

  // Remove other version files before downloading
  ESP_GOTO_ON_ERROR(Storage::RemoveOldVersionFile(curr_ver, file_prefix),
                    CLEANUP, TAG, "RemoveOldVersionFile: %s (%s)", file_prefix,
                    curr_ver);

  // Initiate the download configuration
  cfg.temp_path = temp_path;
  cfg.final_path = path;
  cfg.type = type;

  memset(&client_config, 0, sizeof(esp_http_client_config_t));
  client_config.url = url;
  client_config.cert_pem = cert_pem;
  client_config.timeout_ms = FIRMWARE_DOWNLOAD_RECV_TIMEOUT;
  client_config.event_handler = http_event_handler;
  client_config.buffer_size = FIRMWARE_DOWNLOAD_RECV_BUFFER_SIZE;
  client_config.user_data = &cfg;
  client_config.keep_alive_enable = true;
#ifdef CONFIG_SKIP_DOWNLOAD_ENDPOINT_COMMON_NAME_CHECK
  config.skip_cert_common_name_check = true;
#endif  // !CONFIG_SKIP_DOWNLOAD_ENDPOINT_COMMON_NAME_CHECK

  client = esp_http_client_init(&client_config);
  ESP_GOTO_ON_FALSE(client != NULL, ESP_ERR_NO_MEM, CLEANUP, TAG,
                    "esp_http_client_init");

  ESP_GOTO_ON_ERROR(aes_cbc_decrypt_init(), CLEANUP, TAG,
                    "aes_cbc_decrypt_init");
  // Start the HTTP request
  ESP_LOGI(TAG, "GET %s", url);
  ESP_GOTO_ON_ERROR(esp_http_client_perform(client), CLEANUP, TAG,
                    "esp_http_client_perform");  // blocking until finished
  status_code = esp_http_client_get_status_code(client);
  ESP_GOTO_ON_FALSE(status_code, ESP_ERR_INVALID_RESPONSE, CLEANUP, TAG,
                    "HTTP status code %d", status_code);

  ret = ESP_ERR_NOT_FINISHED;
  if (download_completed) {
    ESP_LOGI(TAG, "Firmware %s downloaded",
             type == FIRMWARE_TYPE_SW3566 ? SW3566_FIRMWARE_FILE_PREFIX
                                          : FPGA_FIRMWARE_FILE_PREFIX);
    // Move the temporary file to the final path
    ESP_GOTO_ON_ERROR(Storage::MoveFile(cfg.temp_path, cfg.final_path, true),
                      CLEANUP, TAG, "MoveFile %s -> %s", cfg.temp_path,
                      cfg.final_path);
    ret = ESP_OK;
  }

CLEANUP:
  aes_cbc_decrypt_deinit();
  if (client != NULL) {
    esp_http_client_cleanup(client);
  }
  return ret;
}
