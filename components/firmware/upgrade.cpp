#include "upgrade.h"

#include <cstdio>
#include <cstring>

#include "display_manager.h"
#include "download.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "firmware.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "https_ota.h"
#include "machine_info.h"
#include "nvs_namespace.h"
#include "portmacro.h"
#include "sdkconfig.h"
#include "telemetry_task.h"
#include "version.h"

#define UPGRADE_TASK_STACK_SIZE CONFIG_UPGRADE_TASK_STACK_SIZE
#define UPGRADE_TASK_PRIORITY CONFIG_UPGRADE_TASK_PRIORITY
#define MAX_URL_LEN 256

static const char *TAG = "Upgrade";

#define GET_FIRMWARE_URL(type, version, url, goto_tag)                   \
  do {                                                                   \
    ESP_GOTO_ON_ERROR(get_firmware_url(type, version, url, sizeof(url)), \
                      goto_tag, TAG, "get_firmware_url");                \
  } while (0)

static bool upgrade_in_progress = false;
static bool download_in_progress = false;

void upgrade_task(void *arg);
void download_task(void *arg);

esp_err_t start_upgrade_task(const firmware_version *new_version) {
  return ESP_OK;
}

void upgrade_task(void *arg) {}

esp_err_t start_download_task(const firmware_version *new_version) {
  return ESP_OK;
}

void download_task(void *arg) {}