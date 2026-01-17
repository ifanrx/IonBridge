#ifndef H_WEB_SERVER_
#define H_WEB_SERVER_

#include "esp_err.h"
#include "esp_http_server.h"
#include "sdkconfig.h"

// #define ENABLE_COREDUMP_TEST

#if CONFIG_ENABLE_WEB_SERVER

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t root_handler(httpd_req_t *req);
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err);

esp_err_t start_web_server(void);
esp_err_t stop_web_server(void);

#ifdef __cplusplus
}
#endif

#endif

#endif  // !H_WEB_SERVER_
