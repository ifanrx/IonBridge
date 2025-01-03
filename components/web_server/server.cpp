#include "server.h"

#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_system.h"
#include "ionbridge.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "machine_info.h"
#include "ping/ping_sock.h"
#include "port_manager.h"

#define WEBSERVER_BUFDZ 512

static const char *TAG = "WebServer";

#if CONFIG_ENABLE_WEB_SERVER
// Define URI Handlers
static const httpd_uri_t uri_handlers[] = {
    {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_handler,
        .user_ctx = NULL,
    },
    {
        .uri = "/metrics",
        .method = HTTP_GET,
        .handler = metrics_handler,
        .user_ctx = NULL,
    },
    {
        .uri = "/procz",
        .method = HTTP_GET,
        .handler = procz_handler,
        .user_ctx = NULL,
    },
    {
        .uri = "/heapz",
        .method = HTTP_GET,
        .handler = heapz_handler,
        .user_ctx = NULL,
    },
    {
        .uri = "/infoz",
        .method = HTTP_GET,
        .handler = infoz_handler,
        .user_ctx = NULL,
    },
    {
        .uri = "/pingz",
        .method = HTTP_GET,
        .handler = pingz_handler,
        .user_ctx = NULL,
    },
};

// Root Handler Implementation
esp_err_t root_handler(httpd_req_t *req) {
  char buffer[WEBSERVER_BUFDZ];
  send_html_headers(req, buffer, "IonBridge Web Server");

  // Get chip information
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  const esp_app_desc_t *app_desc = esp_app_get_description();

  // System Information Table
  httpd_resp_sendstr_chunk(req,
                           "<h2>System Information</h2>"
                           "<table>"
                           "<tr><th>Property</th><th>Value</th></tr>");

  // Chip Info
  snprintf(buffer, sizeof(buffer),
           "<tr><td>Chip Model</td><td>%s</td></tr>"
           "<tr><td>Chip Cores</td><td>%d</td></tr>"
           "<tr><td>CPU Frequency</td><td>%d MHz</td></tr>"
           "<tr><td>IDF Version</td><td>%s</td></tr>"
           "<tr><td>Free Heap</td><td>%lu bytes</td></tr>"
           "<tr><td>App Version</td><td>%s</td></tr>"
           "<tr><td>App Compile Time</td><td>%s %s</td></tr>",
           CONFIG_IDF_TARGET, chip_info.cores, CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
           esp_get_idf_version(), esp_get_free_heap_size(), app_desc->version,
           app_desc->date, app_desc->time);
  httpd_resp_sendstr_chunk(req, buffer);
  httpd_resp_sendstr_chunk(req, "</table>");

  // URL List
  httpd_resp_sendstr_chunk(req,
                           "<h2>Available Endpoints</h2><table>"
                           "<tr><th>URL</th><th>Handler Address</th></tr>");

  for (size_t i = 0; i < sizeof(uri_handlers) / sizeof(uri_handlers[0]); ++i) {
    snprintf(buffer, sizeof(buffer),
             "<tr><td><a href='%s'>%s</a></td><td><tt>0x%08" PRIxPTR
             "</tt></td></tr>",
             uri_handlers[i].uri, uri_handlers[i].uri,
             (uintptr_t)uri_handlers[i].handler);
    httpd_resp_sendstr_chunk(req, buffer);
  }

  httpd_resp_sendstr_chunk(req, "</table></body></html>");
  httpd_resp_send_chunk(req, NULL, 0);

  return ESP_OK;
}

#endif