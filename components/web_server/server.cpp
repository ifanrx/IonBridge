#include "server.h"

#include <dirent.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "cJSON.h"
#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_chip_info.h"
#include "esp_core_dump.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wifi_types_generic.h"
#include "freertos/idf_additions.h"
#include "freertos/portable.h"
#include "freertos/projdefs.h"
#include "http_parser.h"
#include "ionbridge.h"
#include "logging.h"
#include "lwip/inet.h"
#include "lwip/ip_addr.h"
#include "lwip/netdb.h"
#include "machine_info.h"
#include "mqtt_app.h"
#include "multi_heap.h"
#include "nvs_default.h"
#include "nvs_namespace.h"
#include "ping/ping_sock.h"
#include "port.h"
#include "port_manager.h"
#include "portmacro.h"
#include "power_allocator.h"
#include "sdkconfig.h"
#include "strategy.h"
#include "wifi.h"

#if defined(CONFIG_DEVICE_HUMMING_BOARD)
#include "power_config.h"
#endif

#ifdef CONFIG_HEAP_TASK_TRACKING
#include "esp_heap_task_info.h"

#define HEAP_TASK_TRACKING
#define MAX_TASK_NUM 20   // Max number of per tasks info that it can store
#define MAX_BLOCK_NUM 20  // Max number of per block info that it can store

static size_t s_prepopulated_num = 0;
static heap_task_totals_t s_totals_arr[MAX_TASK_NUM];
static heap_task_block_t s_block_arr[MAX_BLOCK_NUM];
#endif

#if defined(CONFIG_ENABLE_ANIMATION_HTTP_TEST_API)
#include "display_animation.h"
#include "display_manager.h"
#endif

#ifdef CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
#define COREDUMP_CHUNK_SZ 1024
#endif

#define WEBSERVER_BUFSZ 512
#define MQTT_APP_URI CONFIG_MQTT_APP_URI

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
};

static httpd_handle_t server;
static SemaphoreHandle_t server_lock = NULL;

// Error Handler Implementation
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err) {
  httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
  return ESP_FAIL;
}

esp_err_t stop_web_server() {
  if (!server_lock) {
    ESP_LOGW(TAG, "Cannot stop Web server: server_lock is NULL");
    return ESP_FAIL;
  }

  if (!server) {
    ESP_LOGI(TAG, "Cannot stop Web server: server is NULL");
    return ESP_OK;
  }

  if (xSemaphoreTake(server_lock, pdMS_TO_TICKS(100)) != pdPASS) {
    ESP_LOGE(TAG, "Failed to take server lock");
    return ESP_FAIL;
  }

  esp_err_t ret = httpd_stop(server);
  if (ret == ESP_OK) {
    server = NULL;
    ESP_LOGI(TAG, "Web server stopped successfully");
  } else {
    ESP_LOGE(TAG, "Failed to stop Web server: %d", ret);
  }

  xSemaphoreGive(server_lock);
  return ret;
}

// Start Web Server
esp_err_t start_web_server(void) {
  if (server_lock == NULL) {
    server_lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(server_lock != NULL, ESP_ERR_NO_MEM, TAG,
                        "Failed to create server lock");
  }

  if (server) {
    ESP_RETURN_ON_ERROR(stop_web_server(), TAG, "Failed to stop Web server");
  }
  ESP_RETURN_ON_FALSE(xSemaphoreTake(server_lock, pdMS_TO_TICKS(100)) == pdPASS,
                      ESP_FAIL, TAG, "Failed to take server lock");

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = sizeof(uri_handlers) / sizeof(httpd_uri_t);
  config.max_open_sockets = 10;
  config.lru_purge_enable = true;
  config.stack_size = 4 * 1024;
  config.keep_alive_enable = true;

  ESP_RETURN_ON_ERROR(httpd_start(&server, &config), TAG,
                      "Unable to start Web server");

  for (size_t i = 0; i < sizeof(uri_handlers) / sizeof(httpd_uri_t); ++i) {
    httpd_register_uri_handler(server, &uri_handlers[i]);
  }
  httpd_register_err_handler(server, HTTPD_404_NOT_FOUND,
                             http_404_error_handler);
  uint8_t ipv4[4];
  get_ipv4_addr(ipv4);
  const std::string &hostname = MachineInfo::GetInstance().GetMDNSHostname();
  ESP_LOGI(TAG, "Web server is running at http://%s.local (http://%d.%d.%d.%d)",
           hostname.c_str(), ipv4[0], ipv4[1], ipv4[2], ipv4[3]);
  xSemaphoreGive(server_lock);
  return ESP_OK;
}

// Common Utilities
void send_html_headers(httpd_req_t *req, char *buf, const char *title) {
  snprintf(
      buf, WEBSERVER_BUFSZ,
      "<!DOCTYPE html><html><head>"
      "<meta name='viewport' content='width=device-width, initial-scale=1'/>"
      "<link rel='stylesheet' "
      "href='https://s3.cn-southeast-1.ifanrprod.com/"
      "public.cn-southeast-1.ifanrprod.com/cp-02/IonBridge.20250130.css'>"
      "<link rel='shortcut icon' "
      "href='https://s3.cn-southeast-1.ifanrprod.com/"
      "public.cn-southeast-1.ifanrprod.com/cp-02/candysign.ico'>"
      "<title>%s</title>"
      "<body><h1>%s</h1>",
      title, title);
  httpd_resp_sendstr_chunk(req, buf);
  httpd_resp_set_type(req, "text/html; charset=utf-8");
}

// Root Handler Implementation
esp_err_t root_handler(httpd_req_t *req) {
  char buffer[WEBSERVER_BUFSZ];
  static uint8_t fc_protocol, die_temperature;
  static uint16_t current, voltage;

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
           "<tr><td>Free Heap</td><td>%" PRIu32
           " bytes</td></tr>"
           "<tr><td>App Version</td><td>%s</td></tr>"
           "<tr><td>App Compile Time</td><td>%s %s</td></tr>",
           CONFIG_IDF_TARGET, chip_info.cores, CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
           esp_get_idf_version(), esp_get_free_heap_size(), app_desc->version,
           app_desc->date, app_desc->time);
  httpd_resp_sendstr_chunk(req, buffer);
  httpd_resp_sendstr_chunk(req, "</table>");

  // Port Metrics Table
  httpd_resp_sendstr_chunk(req,
                           "<h2>Port Metrics</h2>"
                           "<table>"
                           "<tr>"
                           "<th>Port ID</th>"
                           "<th>State</th>"
                           "<th>FC Protocol</th>"
                           "<th>Current</th>"
                           "<th>Voltage</th>"
                           "</tr>");

  PortManager &pm = PortManager::GetInstance();
  for (int i = 0; i < NUM_PORTS; i++) {
    Port *port = pm.GetPort(i);
    if (port == nullptr) {
      snprintf(buffer, sizeof(buffer),
               "<tr>"
               "<td>%d</td>"
               "<td>Inactive</td>"
               "<td colspan='3'>N/A</td>"
               "</tr>",
               i);
      httpd_resp_sendstr_chunk(req, buffer);
      continue;
    }

    port->GetData(&fc_protocol, &die_temperature, &current, &voltage);
    snprintf(buffer, sizeof(buffer),
             "<tr>"
             "<td>%d</td>"
             "<td>%s</td>"
             "<td>%u</td>"
             "<td>%u</td>"
             "<td>%u</td>"
             "</tr>",
             i, "Active", fc_protocol, current, voltage);
    httpd_resp_sendstr_chunk(req, buffer);
  }
  httpd_resp_sendstr_chunk(req, "</table>");

  wifi_ap_record_t ap_info;
  // Attempt to retrieve connected AP information
  if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
    httpd_resp_sendstr_chunk(
        req,
        "<h2>Wi-Fi Information</h2>"
        "<table>"
        "<tr><th>SSID</th><th>BSSID</th><th>Channel</th><th>RSSI</th></tr>");

    snprintf(buffer, sizeof(buffer),
             "<tr><td>%s</td><td>%s</td><td>%d</td><td>%d</td></tr>",
             ap_info.ssid,
             FORMAT_MAC(ap_info.bssid),  // Assumes FORMAT_MAC macro is defined
             ap_info.primary, ap_info.rssi);
    httpd_resp_sendstr_chunk(req, buffer);
    httpd_resp_sendstr_chunk(req, "</table>");
  }

  // URL List
  httpd_resp_sendstr_chunk(req,
                           "<h2>Available Endpoints</h2><table>"
                           "<tr><th>URL</th><th>Handler Address</th></tr>");

  for (size_t i = 0; i < (sizeof(uri_handlers) / sizeof(httpd_uri_t)) - 1;
       ++i) {
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
