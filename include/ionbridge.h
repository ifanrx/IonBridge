#ifndef H_IONBRIDGE_
#define H_IONBRIDGE_
#include "esp_idf_version.h"

#define FORMAT_MAC(bssid)                                                   \
  ({                                                                        \
    static char str[18];                                                    \
    snprintf(str, sizeof(str), "%02x:%02x:%02x:%02x:%02x:%02x", (bssid)[0], \
             (bssid)[1], (bssid)[2], (bssid)[3], (bssid)[4], (bssid)[5]);   \
    (const char *)str;                                                      \
  })

#define ESP_ERROR_COMPLAIN(x, format, ...)                                  \
  do {                                                                      \
    esp_err_t err_rc_ = (x);                                                \
    if (unlikely(err_rc_ != ESP_OK)) {                                      \
      ESP_LOGE(TAG, "%s(%d) error 0x%04X: %s - " format, __FUNCTION__,      \
               __LINE__, err_rc_, esp_err_to_name(err_rc_), ##__VA_ARGS__); \
    }                                                                       \
  } while (0)

#define ESP_RETURN_FALSE_ON_ERROR(x, format, ...)                            \
  do {                                                                       \
    esp_err_t err_rc_ = (x);                                                 \
    if (unlikely(err_rc_ != ESP_OK)) {                                       \
      ESP_LOGE(TAG, "%s(%d) error %d: %s - " format, __FUNCTION__, __LINE__, \
               err_rc_, esp_err_to_name(err_rc_), ##__VA_ARGS__);            \
      return false;                                                          \
    }                                                                        \
  } while (0)

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 3, 0)
#define ESP_RETURN_VOID_ON_ERROR(x, log_tag, format, ...)                   \
  do {                                                                      \
    esp_err_t err_rc_ = (x);                                                \
    if (unlikely(err_rc_ != ESP_OK)) {                                      \
      ESP_LOGE(log_tag, "%s(%d) error %d: %s - " format, __FUNCTION__,      \
               __LINE__, err_rc_, esp_err_to_name(err_rc_), ##__VA_ARGS__); \
      return;                                                               \
    }                                                                       \
  } while (0)
#endif

#define WAIT_FOR_CONDITION(condition, delay_time_ms) \
  do {                                               \
    while (!(condition)) {                           \
      vTaskDelay(pdMS_TO_TICKS(delay_time_ms));      \
    }                                                \
  } while (0)

#define WAIT_FOR_CONDITION_OR_GOTO(max_times, condition, delay_time_ms, \
                                   goto_tag, format, ...)               \
  do {                                                                  \
    uint8_t _i;                                                         \
    for (_i = 0; _i < (max_times); _i++) {                              \
      if (condition) {                                                  \
        break;                                                          \
      }                                                                 \
      vTaskDelay(pdMS_TO_TICKS(delay_time_ms));                         \
    }                                                                   \
    if (_i == (max_times)) {                                            \
      ESP_LOGE(TAG, format, ##__VA_ARGS__);                             \
      ret = ESP_ERR_TIMEOUT;                                            \
      goto goto_tag;                                                    \
    }                                                                   \
  } while (0)

#define PERIODIC_LOG(interval_ms, tag, format, ...)                    \
  do {                                                                 \
    static volatile int64_t start_time = 0;                            \
    int64_t current_time = esp_timer_get_time();                       \
    if (start_time == 0) {                                             \
      start_time = current_time;                                       \
    }                                                                  \
    if ((esp_timer_get_time() - start_time) >= (interval_ms * 1000)) { \
      ESP_LOGI(tag, format, ##__VA_ARGS__);                            \
      start_time = current_time;                                       \
    }                                                                  \
  } while (0)

#endif
