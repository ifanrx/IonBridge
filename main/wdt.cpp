#include "wdt.h"

#include <cinttypes>
#include <cstddef>

#include "FreeRTOSConfig.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "freertos/idf_additions.h"
#include "sdkconfig.h"
#include "utils.h"

static const char *TAG = "WDT";

#define LOW_MEMORY_THRESHOLD (6 * 1024)  // 6KB
#define WATCHDOG_TIMEOUT CONFIG_WATCHDOG_TIMEOUT

void feed_watchdog_task(void *arg);

void init_watchdog() {
#if !CONFIG_ESP_TASK_WDT_INIT
  // If the TWDT was not initialized automatically on startup,
  // manually initialize it now
  esp_task_wdt_config_t twdt_config = {
      .timeout_ms = WATCHDOG_TIMEOUT,
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // Bitmask of all cores
      .trigger_panic = true,
  };
  ESP_RETURN_VOID_ON_ERROR(esp_task_wdt_init(&twdt_config), TAG,
                           "esp_task_wdt_init");
  ESP_LOGI(TAG, "TWDT initialized");
#endif  // CONFIG_ESP_TASK_WDT_INIT
  xTaskCreate(feed_watchdog_task, "wdg", 1.5 * 1024, NULL,
              CONFIG_WATCHDOG_TASK_PRIORITY, nullptr);
}

void deinit_watchdog() {
#if !CONFIG_ESP_TASK_WDT_INIT
  // If we manually initialized the TWDT, deinitialize it now
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_task_wdt_deinit());
  ESP_LOGI(TAG, "TWDT deinitialized");
#endif  // !CONFIG_ESP_TASK_WDT_INIT
}

void feed_watchdog_task(void *arg) {
  ESP_ERROR_CHECK(esp_task_wdt_add(nullptr));     // Subscribe this task to TWDT
  ESP_ERROR_CHECK(esp_task_wdt_status(nullptr));  // Check if it is subscribed
  ESP_LOGI(TAG, "TWDT is subscribed");
  while (true) {
    if (esp_get_free_heap_size() < LOW_MEMORY_THRESHOLD) {
      // If the free heap size is less than 6KB, restart the system to free up
      // memory
      ESP_LOGW(TAG,
               "Free heap size is below 6KB: %" PRIu32 " - restarting system",
               esp_get_free_heap_size());
      esp_restart();
    }
    esp_task_wdt_reset();  // Feed watchdog timer
    DELAY_MS(50);
  }
  vTaskDelete(NULL);
}
