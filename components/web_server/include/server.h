#ifndef H_WEB_SERVER_
#define H_WEB_SERVER_

#include "esp_err.h"
#include "esp_http_server.h"
#include "sdkconfig.h"

#if CONFIG_ENABLE_WEB_SERVER

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint16_t min_time;
  uint16_t max_time;
  uint16_t total_time;
  uint8_t received;
  uint8_t lost;
  EventGroupHandle_t event_group;  // For synchronization
} ping_stats_t;

#define PING_DONE_BIT BIT0
#define PING_COUNT 10

esp_err_t root_handler(httpd_req_t *req);
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err);

esp_err_t start_web_server(void);
esp_err_t stop_web_server(void);

#ifdef __cplusplus
}
#endif

#endif

#endif  // !H_WEB_SERVER_
