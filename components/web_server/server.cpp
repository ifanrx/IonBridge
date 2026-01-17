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
#include "data_types.h"
#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_chip_info.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wifi_types_generic.h"
#include "feature_bits.h"
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
#include "nvs_key.h"
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
#include "fpga_data_types.h"
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
#include "esp_core_dump.h"
#include "esp_partition.h"

#define COREDUMP_CHUNK_SZ 1024
#endif

#define WEBSERVER_BUFSZ 1024
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

esp_err_t stop_web_server() {
  if (!server_lock) {
    ESP_LOGD(TAG, "Cannot stop Web server: server_lock is NULL");
    return ESP_OK;
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

  // Detailed Port Information Tables
  PortManager &pm = PortManager::GetInstance();
  for (int i = 0; i < NUM_PORTS; i++) {
    Port *port = pm.GetPort(i);
    if (port == nullptr) {
      snprintf(buffer, sizeof(buffer),
               "<h2>Port %d - Inactive</h2>"
               "<table>"
               "<tr><th>Status</th><td>Port not initialized</td></tr>"
               "</table>",
               i);
      httpd_resp_sendstr_chunk(req, buffer);
      continue;
    }

    // Get port data
    port->GetData(&fc_protocol, &die_temperature, &current, &voltage);

    // Get PD status
    ClientPDStatus pd_status = {};
    port->GetPDStatus(&pd_status);

    // Get port details
    PortDetails details = {};
    port->GetDetails(&details);

    // Port header
    snprintf(buffer, sizeof(buffer), "<h2>Port %d - %s</h2>", i,
             port->Attached() ? "Connected" : "Disconnected");
    httpd_resp_sendstr_chunk(req, buffer);

    // Main port table
    httpd_resp_sendstr_chunk(req,
                             "<table>"
                             "<tr><th>Property</th><th>Value</th></tr>");

    // Basic port information
    snprintf(buffer, sizeof(buffer),
             "<tr><td>State</td><td>%s</td></tr>"
             "<tr><td>Port Type</td><td>%s</td></tr>"
             "<tr><td>Initialized</td><td>%s</td></tr>",
             port->GetState().ToString().c_str(),
             port->IsTypeA() ? "Type-A" : "Type-C",
             port->IsInitialized() ? "Yes" : "No");
    httpd_resp_sendstr_chunk(req, buffer);

    // Power data (only show FC Protocol when attached)
    if (port->Attached()) {
      snprintf(buffer, sizeof(buffer),
               "<tr><td>FC Protocol</td><td>%u</td></tr>"
               "<tr><td>Current</td><td>%umA</td></tr>"
               "<tr><td>Voltage</td><td>%umV</td></tr>",
               fc_protocol, current, voltage);
      httpd_resp_sendstr_chunk(req, buffer);
    }

    // PD Status information (if available)
    if (port->Attached() && fc_protocol >= FC_PD_Fix5V) {
      httpd_resp_sendstr_chunk(
          req, "<tr><td colspan='2'><strong>PD Status</strong></td></tr>");

      snprintf(buffer, sizeof(buffer),
               "<tr><td>Battery VID</td><td>0x%04X</td></tr>"
               "<tr><td>Battery PID</td><td>0x%04X</td></tr>"
               "<tr><td>Battery Design Capacity</td><td>%" PRIu32
               "mWh</td></tr>"
               "<tr><td>Battery Last Full Capacity</td><td>%" PRIu32
               "mWh</td></tr>"
               "<tr><td>Battery Present Capacity</td><td>%" PRIu32
               "mWh</td></tr>"
               "<tr><td>Battery Present</td><td>%s</td></tr>"
               "<tr><td>Battery Status</td><td>%s</td></tr>"
               "<tr><td>Manufacturer VID</td><td>0x%04X</td></tr>"
               "<tr><td>Manufacturer PID</td><td>0x%04X</td></tr>"
               "<tr><td>Cable VID</td><td>0x%04X</td></tr>"
               "<tr><td>Cable PID</td><td>0x%04X</td></tr>"
               "<tr><td>Operating Current</td><td>%" PRIu32
               "mA</td></tr>"
               "<tr><td>Operating Voltage</td><td>%" PRIu32
               "mV</td></tr>"
               "<tr><td>PD Revision</td><td>%d.%d</td></tr>",
               pd_status.battery_vid, pd_status.battery_pid,
               (uint32_t)pd_status.battery_design_capacity * 100,
               (uint32_t)pd_status.battery_last_full_charge_capacity * 100,
               (uint32_t)pd_status.battery_present_capacity * 100,
               pd_status.battery_present ? "Yes" : "No",
               pd_status.battery_status == 0   ? "Charging"
               : pd_status.battery_status == 1 ? "Discharging"
               : pd_status.battery_status == 2 ? "Idle"
                                               : "Unknown",
               pd_status.manufacturer_vid, pd_status.manufacturer_pid,
               pd_status.cable_vid, pd_status.cable_pid,
               (uint32_t)pd_status.operating_current * 10,
               (uint32_t)pd_status.operating_voltage * 10,
               pd_status.pd_revision == 0   ? 1
               : pd_status.pd_revision == 1 ? 2
               : pd_status.pd_revision == 3 ? 3
                                            : 0,
               0);
      httpd_resp_sendstr_chunk(req, buffer);

      // Cable information
      snprintf(buffer, sizeof(buffer),
               "<tr><td>Cable Active</td><td>%s</td></tr>"
               "<tr><td>Cable EPR Mode Capable</td><td>%s</td></tr>"
               "<tr><td>Cable Max VBUS Voltage</td><td>%s</td></tr>"
               "<tr><td>Cable Max VBUS Current</td><td>%s</td></tr>"
               "<tr><td>Cable USB Highest Speed</td><td>%s</td></tr>"
               "<tr><td>Has eMarker</td><td>%s</td></tr>"
               "<tr><td>PPS Charging Supported</td><td>%s</td></tr>"
               "<tr><td>Has Battery</td><td>%s</td></tr>"
               "<tr><td>Dual Role Power</td><td>%s</td></tr>",
               pd_status.cable_is_active ? "Yes" : "No",
               pd_status.cable_epr_mode_capable ? "Yes" : "No",
               pd_status.cable_max_vbus_voltage == 0   ? "20V"
               : pd_status.cable_max_vbus_voltage == 1 ? "30V"
               : pd_status.cable_max_vbus_voltage == 2 ? "40V"
               : pd_status.cable_max_vbus_voltage == 3 ? "50V"
                                                       : "Unknown",
               pd_status.cable_max_vbus_current == 1   ? "3A"
               : pd_status.cable_max_vbus_current == 2 ? "5A"
                                                       : "Unknown",
               pd_status.cable_usb_highest_speed == 0   ? "USB 2.0"
               : pd_status.cable_usb_highest_speed == 1 ? "USB 3.2 Gen1"
               : pd_status.cable_usb_highest_speed == 2 ? "USB 3.2/4 Gen2"
               : pd_status.cable_usb_highest_speed == 3 ? "USB4 Gen3"
               : pd_status.cable_usb_highest_speed == 4 ? "USB4 Gen4"
                                                        : "Unknown",
               pd_status.has_emarker ? "Yes" : "No",
               pd_status.pps_charging_supported ? "Yes" : "No",
               pd_status.has_battery ? "Yes" : "No",
               pd_status.dual_role_power ? "Yes" : "No");
      httpd_resp_sendstr_chunk(req, buffer);
    }

    httpd_resp_sendstr_chunk(req, "</table>");
  }

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

// Error Handler Implementation
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err) {
  httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Resource not found");
  return ESP_OK;
}

#endif
