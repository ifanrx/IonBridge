#include "wifi.h"

#include <string.h>
#include <sys/param.h>

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>  // for std::memcpy
#include <map>

#include "animation.h"
#include "ble.h"
#include "controller.h"
#include "esp_bit_defs.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_event_base.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_netif_types.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_wifi_types_generic.h"
#include "freertos/event_groups.h"  // IWYU pragma: keep
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "ionbridge.h"
#include "lwip/netdb.h"
#include "machine_info.h"
#include "mqtt_app.h"
#include "sdkconfig.h"
#include "syslog.h"
#include "utils.h"
#include "wifi_manager.h"

#ifdef CONFIG_LWIP_IPV6
#include "lwip/ip6_addr.h"
#endif

#if CONFIG_ENABLE_WEB_SERVER
#define ENABLE_WEB_SERVER CONFIG_ENABLE_WEB_SERVER
#endif

#ifdef ENABLE_WEB_SERVER
#include "server.h"
#endif

#define WIFI_RECORD_COUNT CONFIG_WIFI_RECORD_COUNT

#define WIFI_STA_START_TIMEOUT CONFIG_ESP_WIFI_STA_START_TIMEOUT
#define WIFI_STA_MAXIMUM_RETRY CONFIG_ESP_WIFI_STA_MAXIMUM_RETRY
#define WIFI_SCAN_LIST_SIZE CONFIG_ESP_WIFI_SCAN_LIST_SIZE
#define WIFI_RSSI_LOG_TIMER_INTERVAL CONFIG_WIFI_RSSI_LOG_TIMER_INTERVAL
#define WIFI_DEFAULT_SSID CONFIG_IONBRIDGE_WIFI_DEFAULT_SSID
#define WIFI_DEFAULT_PASSWD CONFIG_IONBRIDGE_WIFI_DEFAULT_PASSWORD
#define DEFAULT_WIFI_CONNECTION_DELAY CONFIG_DEFAULT_WIFI_CONNECTION_DELAY
#define WIFI_RECONNECT_INTERVAL_MS CONFIG_WIFI_RECONNECT_INTERVAL_MS
#define WIFI_RECONNECT_LOOP_INTERVAL_MS CONFIG_WIFI_RECONNECT_LOOP_INTERVAL_MS
#define MQTT_APP_URI CONFIG_MQTT_APP_URI
#define MQTT_APP_HOST CONFIG_MQTT_APP_HOSTNAME
#define SYSLOG_HOST CONFIG_SYSLOG_HOST
#define WIFI_RETRY_CONNECT_DELAY_MS 10000
#define DISPLAY_FLASH_INTERVAL_MS CONFIG_DISPLAY_FLASH_INTERVAL_MS
#define FALLBACK_IP CONFIG_FALLBACK_IP

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define WIFI_SAE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define WIFI_SAE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define WIFI_SAE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif

#if CONFIG_ESP_WIFI_AUTH_OPEN
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

// Define constants for scan attempts and delays
#define MAX_SCAN_ATTEMPTS 5
#define SCAN_RETRY_DELAY_MS 500

static EventGroupHandle_t s_wifi_event_group;

#define WIFI_STA_STARTED_BIT BIT0

static const char *TAG = "WiFi";
static esp_ip4_addr_t local_ip;
#ifdef CONFIG_LWIP_IPV6
static esp_ip6_addr_t local_ip6;
#endif
static bool has_internet_access = false;

#define WAIT_FOR_NOTIFICATION(delay_ms, ssid, goto_tag)                        \
  if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(delay_ms)) == pdPASS) {        \
    ESP_LOGI(TAG, "Exiting Wi-Fi task for SSID: %s after notification", ssid); \
    goto goto_tag;                                                             \
  }

// static array of generate 204 urls
static const char *connectivity_check_endpoints_cn[] = {
    CONFIG_GENERATE_204_URL_DOMESTIC,
    CONFIG_GENERATE_204_URL_DOMESTIC_BACKUP,
    NULL,
};
static const char *connectivity_check_endpoints[] = {
    CONFIG_GENERATE_204_URL_GLOBAL,
    CONFIG_GENERATE_204_URL_GLOBAL_BACKUP,
    NULL,
};

static bool wifi_is_ready = false, wifi_is_connected_ = false,
            wifi_manually_disconnected = false;

static uint8_t retry_cnt = 0;
static uint16_t connection_start_time = 0;

TaskHandle_t wifi_task_handle = NULL;
bool wifi_task_running = false;
esp_event_handler_instance_t wifi_event_handler_instance,
    ip_event_handler_instance;

typedef struct {
  char ssid[WIFI_SSID_MAXIMUM];
  char passwd[WIFI_PASSWD_MAXIMUM];
  bool skip_reconnect;
} WiFiTaskParams;
static WiFiTaskParams wifi_task_params;

static TimerHandle_t wifi_log_rssi_timer = NULL;
static void wifi_log_rssi_timer_cb(TimerHandle_t xTimer);
static void start_wifi_log_rssi_timer();
static void stop_wifi_log_rssi_timer();

static TimerHandle_t wifi_reconnect_timer = NULL;
static void wifi_reconnect_timer_cb(TimerHandle_t);
static void start_wifi_reconnect_timer(int delay_ms);
static void stop_wifi_reconnect_timer();

static void handle_wifi_disconnect_event(void *event_data, uint32_t *delay_ms);

static bool check_network_connectivity();

static void ip_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data) {
  if (event_base != IP_EVENT) {
    ESP_LOGE(TAG, "Unhandled event base: %s", event_base);
    return;
  }
#ifdef ENABLE_WEB_SERVER
  static httpd_handle_t server = NULL;
#endif
  switch (event_id) {
#ifdef CONFIG_LWIP_IPV6
    case IP_EVENT_GOT_IP6: {
      ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
      local_ip6 = event->ip6_info.ip;
      const char *ipv6 = ip6addr_ntoa((ip6_addr_t *)&event->ip6_info.ip);
      ESP_LOGI(TAG, "Got IPv6: %s", ipv6);
      MQTTClient::GetInstance()->Start();
      break;
    }
#endif
    case IP_EVENT_STA_GOT_IP: {
      ble_adv_stop();
      ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
      local_ip = event->ip_info.ip;
      ESP_LOGI(TAG,
               "Got IP: " IPSTR ", Mask: " IPSTR ", GW: " IPSTR ", changed: %d",
               IP2STR(&local_ip), IP2STR(&event->ip_info.netmask),
               IP2STR(&event->ip_info.gw), event->ip_changed);
      if (!check_network_connectivity()) {
        ESP_LOGW(TAG, "No network connectivity");
        esp_wifi_disconnect();
        break;
      }
      WiFiManager::GetInstance().OnConnected();
      WiFiManager::GetInstance().AddWifiConnectionTime(
          esp_timer_get_time() / 1e6 - connection_start_time);
      connection_start_time = 0;
      retry_cnt = 0;
#ifdef CONFIG_ENABLE_GLOBAL_LOGGING_COLLECTOR
      ESP_LOGW(TAG, "Remote logging is enabled");
      SyslogClient &syslog = SyslogClient::GetInstance();
      ESP_ERROR_COMPLAIN(syslog.Initialize(), "Failed to initialize syslog");
      esp_log_set_vprintf(SyslogClient::Report);
#endif

      has_internet_access = true;
#ifdef ENABLE_WEB_SERVER
      if (server == NULL) {
        ESP_LOGI(TAG, "Starting web server");
      }
#endif
      break;
    }
    case IP_EVENT_STA_LOST_IP:
      ESP_LOGE(TAG, "IP address lost");
      has_internet_access = false;
      ble_adv_delay_start();
#ifdef ENABLE_WEB_SERVER
      if (server != NULL) {
        ESP_LOGI(TAG, "Stopping web server");
      }
#endif
      SyslogClient::GetInstance().Cleanup();
      break;
    default:
      ESP_LOGE(TAG, "Unhandled IP_EVENT: %" PRIu32, event_id);
      break;
  }
  return;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  static uint32_t delay_ms = WIFI_RETRY_CONNECT_DELAY_MS;
  if (event_base != WIFI_EVENT) {
    ESP_LOGE(TAG, "Unhandled event base: %s", event_base);
    return;
  }
  switch (event_id) {
    case WIFI_EVENT_STA_START:
      ESP_LOGI(TAG, "WIFI_EVENT_STA_START");
      xEventGroupSetBits(s_wifi_event_group, WIFI_STA_STARTED_BIT);
      ble_adv_stop();
      break;
    case WIFI_EVENT_STA_STOP:
      ESP_LOGI(TAG, "WIFI_EVENT_STA_STOP");
      ble_adv_start();
      break;
    case WIFI_EVENT_STA_CONNECTED:
      ESP_LOGI(TAG, "WIFI_EVENT_STA_CONNECTED");
      wifi_is_connected_ = true;
      connection_start_time = esp_timer_get_time() / 1e6;
      stop_wifi_reconnect_timer();
      start_wifi_log_rssi_timer();
      delay_ms = WIFI_RETRY_CONNECT_DELAY_MS;
      AnimationController::GetInstance().stop_animation(
          AnimationId::WIFI_CONNECTING);
      break;
    case WIFI_EVENT_SCAN_DONE:
      ESP_LOGD(TAG, "WIFI_EVENT_SCAN_DONE");
      break;
    case WIFI_EVENT_HOME_CHANNEL_CHANGE:
      ESP_LOGD(TAG, "WIFI_EVENT_HOME_CHANNEL_CHANGE");
      break;
    case WIFI_EVENT_AP_STADISCONNECTED: {
      /*
       * Every time a station is connected to ESP32-C3 AP, the
       * WIFI_EVENT_AP_STACONNECTED will arise. Upon receiving this event, the
       * event task will do nothing, and the application callback can also
       * ignore it. However, you may want to do something, for example, to get
       * the info of the connected STA.
       */
      ESP_LOGI(TAG, "WIFI_EVENT_AP_STADISCONNECTED");
    } break;
    case WIFI_EVENT_STA_DISCONNECTED: {
      wifi_is_connected_ = false;
      has_internet_access = false;
      connection_start_time = 0;
      ble_adv_start();
      WiFiManager::GetInstance().OnDisconnected();
      stop_wifi_log_rssi_timer();
      MQTTClient::GetInstance()->Stop();
      SyslogClient::GetInstance().Cleanup();
      handle_wifi_disconnect_event(event_data, &delay_ms);
    } break;
    case WIFI_EVENT_STA_BEACON_TIMEOUT:
      /*
       * For Station, If the station does not receive a beacon frame from the
       * connected SoftAP during the inactive time, disconnect from SoftAP.
       * Default 6s.
       */
      ESP_LOGI(TAG, "WIFI_EVENT_STA_BEACON_TIMEOUT");
      break;
    default:
      ESP_LOGE(TAG, "Unhandled WIFI_EVENT: %" PRIu32, event_id);
      break;
  }
}

esp_err_t wifi_scan_ap(ssid_info_t *aps, bool pause_reconnect_timer) {
  ESP_RETURN_ON_FALSE(wifi_is_ready, ESP_ERR_WIFI_NOT_INIT, TAG,
                      "Wi-Fi is not ready");
  // Temporarily pause the reconnect timer if required
  bool should_pause_reconnect =
      (wifi_reconnect_timer != NULL) && pause_reconnect_timer;
  uint16_t ap_count = 0;
  uint16_t scan_result_size = WIFI_SCAN_LIST_SIZE;
  uint8_t valid_ap_count = 0, max_count = 0;
  std::map<std::string, int> country_count;
  std::string most_common_country;

  if (should_pause_reconnect) {
    stop_wifi_reconnect_timer();  // Abort any pending connection attempts
  }

  esp_err_t ret = ESP_OK;
  wifi_ap_record_t ap_info[WIFI_SCAN_LIST_SIZE];
  memset(ap_info, 0, sizeof(ap_info));

  // Initialize scan configuration with default parameters
  wifi_scan_config_t scan_config = {
      .show_hidden = true,
      .scan_type = WIFI_SCAN_TYPE_ACTIVE,
      .scan_time =
          {
              .active =
                  {
                      .min = 100,
                      .max = 300,
                  },
          },
  };

  wifi_scan_default_params_t default_params;
  ESP_GOTO_ON_ERROR(esp_wifi_get_scan_parameters(&default_params), CLEANUP, TAG,
                    "esp_wifi_get_scan_parameters");

  scan_config.scan_time = default_params.scan_time;
  scan_config.home_chan_dwell_time = default_params.home_chan_dwell_time;

  // Attempt to start the scan with retries
  for (uint8_t attempt = 0; attempt < MAX_SCAN_ATTEMPTS; attempt++) {
    ret = esp_wifi_scan_start(&scan_config, true);
    if (ret == ESP_OK) {
      break;  // Scan started successfully
    } else if (ret == ESP_ERR_WIFI_STATE && attempt < (MAX_SCAN_ATTEMPTS - 1)) {
      ESP_LOGW(TAG,
               "Wi-Fi not in correct state for scanning, retrying (%d/%d)...",
               attempt + 1, MAX_SCAN_ATTEMPTS);
      vTaskDelay(pdMS_TO_TICKS(SCAN_RETRY_DELAY_MS));
      continue;
    } else {
      ESP_LOGE(TAG, "Failed to start Wi-Fi scan: %s", esp_err_to_name(ret));
      goto CLEANUP;
    }
  }

  ESP_GOTO_ON_ERROR(esp_wifi_scan_get_ap_num(&ap_count), CLEANUP, TAG,
                    "esp_wifi_scan_get_ap_num");

  // Ensure that scan_result_size does not exceed the actual number of APs
  if (scan_result_size > ap_count) {
    scan_result_size = ap_count;
  }

  ESP_GOTO_ON_ERROR(esp_wifi_scan_get_ap_records(&scan_result_size, ap_info),
                    CLEANUP, TAG, "esp_wifi_scan_get_ap_records");

  ESP_LOGI(TAG, "AP scan completed: %u/%u/%u AP(s) found", scan_result_size,
           WIFI_SCAN_LIST_SIZE, ap_count);

  // Process each AP and populate the aps array
  for (uint8_t i = 0;
       i < scan_result_size && valid_ap_count < WIFI_SCAN_LIST_SIZE; ++i) {
    const wifi_ap_record_t *ap = &ap_info[i];

    // Safely determine the SSID length
    size_t ssid_length = strnlen((const char *)ap->ssid, sizeof(ap->ssid));
    if (ssid_length == 0 || ssid_length >= WIFI_SSID_MAXIMUM) {
      continue;  // Skip invalid SSIDs
    }

    // Populate the aps array with AP details
    strncpy(aps[valid_ap_count].ssid, (const char *)ap->ssid, ssid_length);
    aps[valid_ap_count].size = ssid_length;
    aps[valid_ap_count].rssi = ap->rssi;
    aps[valid_ap_count].auth_mode = ap->authmode;
    valid_ap_count++;

    ESP_LOGI(TAG, "Found AP: SSID: %s, RSSI: %d, BSSID: %s, channel: %d",
             ap->ssid, ap->rssi, FORMAT_MAC(ap->bssid), ap->primary);

    std::string current_country_code(ap->country.cc, 2);
    country_count[current_country_code]++;

    // Determine the most common country code dynamically
    if (country_count[current_country_code] > max_count) {
      max_count = country_count[current_country_code];
      most_common_country = current_country_code;
    }
  }

  ESP_LOGI(TAG, "Current country is %s", most_common_country.c_str());

  MachineInfo::GetInstance().SetCountryCode(most_common_country);

CLEANUP:
  // Clear the scan results
  esp_wifi_clear_ap_list();

  // Restart the reconnect timer if it was paused
  if (should_pause_reconnect) {
    start_wifi_reconnect_timer(WIFI_RECONNECT_INTERVAL_MS);
  }

  return ret;
}

esp_err_t get_connected_ssid_detail(ssid_detail_t *ssid_detail) {
  ESP_RETURN_ON_FALSE(ssid_detail, ESP_ERR_INVALID_ARG, TAG, "ssid_detail");
  ESP_RETURN_ON_FALSE(wifi_is_ready, ESP_ERR_WIFI_NOT_INIT, TAG,
                      "! wifi_is_ready");
  ESP_RETURN_ON_FALSE(wifi_is_connected_, ESP_ERR_WIFI_NOT_CONNECT, TAG,
                      "! wifi_is_connected_");

  wifi_ap_record_t ap_info;

  ESP_RETURN_ON_ERROR(esp_wifi_sta_get_ap_info(&ap_info), TAG,
                      "esp_wifi_sta_get_ap_info");

  memcpy(ssid_detail->bssid, ap_info.bssid, 6);
  memcpy(ssid_detail->ssid, ap_info.ssid, 33);
  ssid_detail->channel = ap_info.primary;
  ssid_detail->rssi = ap_info.rssi;
  ssid_detail->phy_11b = ap_info.phy_11b;
  ssid_detail->phy_11g = ap_info.phy_11g;
  ssid_detail->phy_11n = ap_info.phy_11n;
  ssid_detail->phy_lr = ap_info.phy_lr;
  ssid_detail->phy_11ax = ap_info.phy_11ax;
  ssid_detail->wps = ap_info.wps;
  ssid_detail->ftm_responder = ap_info.ftm_responder;
  ssid_detail->ftm_initiator = ap_info.ftm_initiator;
  return ESP_OK;
}

size_t serialize_ssid_detail(const ssid_detail_t *ssid_detail, uint8_t *buf,
                             size_t buf_size) {
  if (buf_size < 11) {
    return 0;  // Ensuring buffer can at least hold bssid, channel, rssi, flags.
  }

  size_t written = 0;
  memcpy(buf, ssid_detail->bssid, sizeof(ssid_detail->bssid));
  written += sizeof(ssid_detail->bssid);

  buf[written++] = ssid_detail->channel;
  buf[written++] = ssid_detail->rssi;

  uint8_t flags = ssid_detail->phy_11b | (ssid_detail->phy_11g << 1) |
                  (ssid_detail->phy_11n << 2) | (ssid_detail->phy_lr << 3) |
                  (ssid_detail->phy_11ax << 4) | (ssid_detail->wps << 5) |
                  (ssid_detail->ftm_responder << 6) |
                  (ssid_detail->ftm_initiator << 7);
  buf[written++] = flags;

  // Copying ssid up to the first null terminator or end of ssid array.
  size_t ssid_len =
      strnlen((char *)ssid_detail->ssid, sizeof(ssid_detail->ssid));
  if (written + ssid_len > buf_size) {
    ssid_len = buf_size - written;  // Adjust length if exceeds buffer.
  }
  memcpy(buf + written, ssid_detail->ssid, ssid_len);
  written += ssid_len;

  return written;
}

void wifi_task(void *taskParams) {
  WiFiManager &wifi_manager = WiFiManager::GetInstance();

  wifi_config_t wifi_config;
  wifi_scan_threshold_t *scan_threshold;
  wifi_sta_config_t *sta_config;
  bool connecting = false;
  esp_err_t ret = ESP_OK;

  size_t ssid_length = strlen(wifi_task_params.ssid),
         passwd_length = strlen(wifi_task_params.passwd);

  ESP_GOTO_ON_FALSE(wifi_is_ready, ESP_ERR_WIFI_NOT_INIT, RETURN, TAG,
                    "Wi-Fi not ready");
  ESP_GOTO_ON_ERROR(esp_wifi_get_config(WIFI_IF_STA, &wifi_config), RETURN, TAG,
                    "esp_wifi_get_config");
  if (ssid_length > WIFI_SSID_MAXIMUM || passwd_length > WIFI_PASSWD_MAXIMUM) {
    ESP_LOGE(TAG, "Wi-Fi SSID(%d/%d) or password(%d/%d) is too long",
             ssid_length, WIFI_SSID_MAXIMUM, passwd_length,
             WIFI_PASSWD_MAXIMUM);
    goto RETURN;
  }
  if (ssid_length == 0 &&
      !wifi_manager.GetWiFi(wifi_task_params.ssid, wifi_task_params.passwd)) {
    ESP_LOGI(TAG, "No Wi-Fi credentials found, stopping Wi-Fi task");
    goto RETURN;
  }

  while (wifi_disconnect() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to disconnect from the AP, retrying.");
    WAIT_FOR_NOTIFICATION(50, wifi_task_params.ssid, RETURN);
  }

  // Wi-Fi configuration
  sta_config = &wifi_config.sta;
  scan_threshold = &sta_config->threshold;
  strncpy((char *)sta_config->ssid, wifi_task_params.ssid,
          sizeof(sta_config->ssid));
  strncpy((char *)sta_config->password, wifi_task_params.passwd,
          sizeof(sta_config->password));

  ESP_LOGD(TAG, "bssid_set default: %d", sta_config->bssid_set);
  sta_config->bssid_set = false;
  ESP_LOGD(TAG, "channel default: %d", sta_config->channel);
  sta_config->channel = 0;
  ESP_LOGD(TAG, "scan_method default: %d", sta_config->scan_method);
  sta_config->scan_method = WIFI_ALL_CHANNEL_SCAN;
  ESP_LOGD(TAG, "listen_interval default: %d", sta_config->listen_interval);
  sta_config->listen_interval = 0;
  ESP_LOGD(TAG, "sort_method default: %d", sta_config->sort_method);
  sta_config->sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
  ESP_LOGD(TAG, "rm_enabled default: %d", sta_config->rm_enabled);
  sta_config->rm_enabled = 1;
  ESP_LOGD(TAG, "btm_enabled default: %d", sta_config->btm_enabled);
  sta_config->btm_enabled = 1;
  ESP_LOGD(TAG, "mbo_enabled default: %d", sta_config->mbo_enabled);
  sta_config->mbo_enabled = 1;
  ESP_LOGD(TAG, "owe_enabled default: %d", sta_config->owe_enabled);
  sta_config->owe_enabled = 1;
  ESP_LOGD(TAG, "ft_enabled default: %d", sta_config->ft_enabled);
  sta_config->ft_enabled = 1;
  ESP_LOGD(TAG, "transition_disable default: %d",
           sta_config->transition_disable);
  sta_config->transition_disable = 1;
  ESP_LOGD(TAG, "failure_retry_cnt default: %d", sta_config->failure_retry_cnt);
  sta_config->failure_retry_cnt = CONFIG_ESP_WIFI_STA_MAXIMUM_RETRY;

  ESP_LOGD(TAG, "sae_pwe_h2e default: %d", sta_config->sae_pwe_h2e);
  sta_config->sae_pwe_h2e = WIFI_SAE_MODE;
  ESP_LOGD(TAG, "sae_pk_mode default: %d", sta_config->sae_pk_mode);
  sta_config->sae_pk_mode = WPA3_SAE_PK_MODE_AUTOMATIC;
  ESP_LOGD(TAG, "sae_h2e_identifier: %s", sta_config->sae_h2e_identifier);
  strncpy((char *)sta_config->sae_h2e_identifier, WIFI_SAE_H2E_IDENTIFIER,
          sizeof(sta_config->sae_h2e_identifier));

  ESP_LOGD(TAG, "threshold authmode default: %d", scan_threshold->authmode);
  scan_threshold->authmode = WIFI_SCAN_AUTH_MODE_THRESHOLD;
  ESP_LOGD(TAG, "threshold rssi default: %d", scan_threshold->rssi);
  scan_threshold->rssi = -127;
  ESP_GOTO_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), RETURN, TAG,
                    "esp_wifi_set_config");

  if (strcmp((char *)sta_config->ssid, WIFI_DEFAULT_SSID) == 0) {
    ble_adv_start();  // Allow BLE advertising if using default credentials
    // Check for a signal to delete this task
    WAIT_FOR_NOTIFICATION(DEFAULT_WIFI_CONNECTION_DELAY * 1000,
                          sta_config->ssid, RETURN);
    ESP_LOGI(TAG, "Using default Wi-Fi credentials: %s", sta_config->ssid);
  } else {
    DELAY_MS(3000);
    ble_adv_stop();  // Stop BLE advertising if using custom credentials
  }

  ESP_LOGI(TAG, "Connecting to the Access Point: %s, authmode: %d",
           sta_config->ssid, sta_config->threshold.authmode);
  ESP_GOTO_ON_ERROR(esp_wifi_connect(), RETURN, TAG, "esp_wifi_connect");
  connecting = true;
  wifi_manually_disconnected = false;
  ESP_LOGI(TAG, "esp_wifi_connect success, exiting Wi-Fi task");
RETURN:
  ESP_ERROR_COMPLAIN(ret, "wifi_task");
  if (!connecting) {
    ble_adv_start();
  }
  wifi_task_running = false;
  vTaskDelete(NULL);
}

bool start_wifi_task(const char *ssid, const char *passwd) {
  if (!wifi_is_ready) {
    ESP_LOGE(
        TAG,
        "Failed to start the Wi-Fi task because it has not been initialized.");
    return false;
  }

  // Allow only one Wi-Fi task to run at a time
  if (wifi_task_handle && wifi_task_running) {
    size_t connected_ssid_length = strlen(wifi_task_params.ssid);
    if (strncmp(wifi_task_params.ssid, ssid, connected_ssid_length) == 0) {
      ESP_LOGI(TAG, "Wi-Fi task is already running");
      return true;
    }
    if (strncmp(wifi_task_params.ssid, WIFI_DEFAULT_SSID,
                connected_ssid_length) == 0) {
      ESP_LOGI(TAG, "Stopping the default Wi-Fi task");
      xTaskNotifyGive(wifi_task_handle);
      DELAY_MS(500);  // Ensure task has time to exit and clean up
    }
    if (wifi_task_running) {
      ESP_LOGE(TAG, "Wi-Fi task is already running with SSID: %s",
               wifi_task_params.ssid);
      return false;
    }
  }

  AnimationController::GetInstance().set_animation(AnimationId::WIFI_CONNECTING,
                                                   LoopMode::FOREVER);
  if (ssid != NULL && passwd != NULL) {
    ESP_LOGI(TAG, "Using the provided SSID and password");
    strcpy(wifi_task_params.ssid, ssid);
    strcpy(wifi_task_params.passwd, passwd);
  }
  wifi_task_params.skip_reconnect = true;

  retry_cnt = 0;
  ESP_LOGI(TAG, "Creating the Wi-Fi task");
  BaseType_t res =
      xTaskCreate(wifi_task, "wifi", 4 * 1024, nullptr, 17, &wifi_task_handle);
  if (res != pdPASS) {
    ESP_LOGE(TAG, "Failed to create the Wi-Fi task");
    return false;
  }
  wifi_task_running = true;
  stop_wifi_reconnect_timer();
  return true;
}

esp_err_t wifi_initialize() {
  if (wifi_is_ready) {
    return ESP_OK;
  }

#if ENABLE_SNTP
  ntp_init();
#endif

  esp_err_t ret;
  int retries = 0;

  ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init");
  ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG,
                      "esp_event_loop_create_default");

  char hostname[22];
  const std::string &device_serial_number = MachineInfo::GetInstance().GetPSN();
  snprintf(hostname, sizeof(hostname), "cp02-%s", device_serial_number.c_str());
  esp_netif_t *netif = esp_netif_create_default_wifi_sta();
  ESP_RETURN_ON_ERROR(esp_netif_set_hostname(netif, hostname), TAG,
                      "esp_netif_set_hostname");

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "esp_wifi_init");
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG,
                      "esp_wifi_set_mode(WIFI_MODE_STA)");

  s_wifi_event_group = xEventGroupCreate();
  esp_event_handler_instance_t instance_started_id;
  ESP_RETURN_ON_ERROR(
      esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_START,
                                          &wifi_event_handler, NULL,
                                          &instance_started_id),
      TAG,
      "wifi_init: esp_event_handler_instance_register WIFI_EVENT_STA_START");

  while (retries < WIFI_RETRIES) {
    ret = esp_wifi_start();
    if (ret == ESP_OK) {
      break;
    }
    if (ret == ESP_ERR_NO_MEM) {
      ESP_LOGW(TAG, "Insufficient memory, delaying for %d milliseconds",
               100 << retries);
      DELAY_MS(100 << retries);
    }
    retries++;
    continue;
  }
  ESP_RETURN_ON_ERROR(ret, TAG, "esp_wifi_start");

  EventBits_t bits =
      xEventGroupWaitBits(s_wifi_event_group, WIFI_STA_STARTED_BIT, pdFALSE,
                          pdFALSE, pdMS_TO_TICKS(WIFI_STA_START_TIMEOUT));
  if (bits & WIFI_STA_STARTED_BIT) {
    wifi_is_ready = true;
  } else {
    ESP_LOGE(TAG, "Failed to start Wi-Fi station");
  }

  ESP_RETURN_ON_ERROR(
      esp_event_handler_instance_unregister(WIFI_EVENT, WIFI_EVENT_STA_START,
                                            instance_started_id),
      TAG, "esp_event_handler_instance_unregister WIFI_EVENT_STA_START");

  if (!wifi_is_ready) {
    return ESP_FAIL;
  }

  ESP_RETURN_ON_ERROR(
      esp_wifi_set_max_tx_power(CONFIG_IONBRIDGE_WIFI_MAX_TX_POWER), TAG,
      "esp_wifi_set_max_tx_power");
  ESP_RETURN_ON_ERROR(esp_wifi_set_inactive_time(WIFI_IF_STA, 12), TAG,
                      "esp_wifi_set_inactive_time");  // double default 6s
  ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
                          WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler,
                          NULL, &wifi_event_handler_instance),
                      TAG, "esp_event_handler_instance_register: WIFI_EVENT");
  ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
                          IP_EVENT, ESP_EVENT_ANY_ID, &ip_event_handler, NULL,
                          &ip_event_handler_instance),
                      TAG, "esp_event_handler_instance_register: IP_EVENT");
  return ESP_OK;
}

esp_err_t wifi_disconnect(bool reconnect) {
  wifi_manually_disconnected = !reconnect;
  // Try to disconnect from the AP if already connected
  ESP_RETURN_ON_ERROR(esp_wifi_disconnect(), TAG, "esp_wifi_disconnect");
  wifi_is_connected_ = false;
  ESP_LOGI(TAG, "Wi-Fi disconnected");
  return ESP_OK;
}

bool wifi_is_connected() { return wifi_is_connected_ && local_ip.addr != 0; }

void get_ipv4_addr(uint8_t *data) {
  if (!wifi_is_connected()) {
    return;
  }
  memcpy(data, &local_ip, 4);
}

#ifdef CONFIG_LWIP_IPV6
void get_ipv6_addr(uint8_t *data) {
  if (!wifi_is_connected()) {
    return;
  }
  memcpy(data, &local_ip6, 16);
}
#endif

esp_err_t wifi_close() {
  ESP_LOGI(TAG, "Closing the WiFi service");
  if (wifi_task_handle != NULL) {
    // A Wi-Fi task has been running; send a message to stop it
    xTaskNotifyGive(wifi_task_handle);
    // Wait a bit to ensure the task has time to exit and clean up
    DELAY_MS(500);
  }
  stop_wifi_log_rssi_timer();
  stop_wifi_reconnect_timer();
  esp_err_t err = esp_wifi_stop();
  if (err == ESP_ERR_WIFI_NOT_INIT) {
    err = ESP_OK;
  }

  if (s_wifi_event_group != NULL) {
    vEventGroupDelete(s_wifi_event_group);
    s_wifi_event_group = NULL;
  }

  return err;
}

void start_wifi_log_rssi_timer() {
  if (!wifi_is_connected_ || wifi_log_rssi_timer != NULL) {
    return;
  }
  wifi_log_rssi_timer = xTimerCreate(
      "wifi_log_timer", pdMS_TO_TICKS(WIFI_RSSI_LOG_TIMER_INTERVAL * 1000),
      pdTRUE, NULL, wifi_log_rssi_timer_cb);
  if (wifi_log_rssi_timer) {
    xTimerStart(wifi_log_rssi_timer, 0);
  }
}

void stop_wifi_log_rssi_timer() {
  if (wifi_log_rssi_timer == NULL) {
    return;
  }
  xTimerStop(wifi_log_rssi_timer, 0);
  xTimerDelete(wifi_log_rssi_timer, 0);
  wifi_log_rssi_timer = NULL;
}

void wifi_log_rssi_timer_cb(TimerHandle_t) {
  static char country_code[3];  // Buffer for null-terminated country code
  wifi_ap_record_t ap_info;

  // Check if Wi-Fi is connected before proceeding
  if (!wifi_is_connected_) {
    stop_wifi_log_rssi_timer();
    return;
  }

  // Fetch the current AP information and handle potential errors
  esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get AP info: %s", esp_err_to_name(err));
    return;
  }

  // Safely copy the country code and ensure null termination
  strncpy(country_code, ap_info.country.cc, 2);
  country_code[2] = '\0';

  // Log the current Wi-Fi access point details
  ESP_LOGI(TAG,
           "Current SSID: %s, RSSI: %d, BSSID: %s, Channel: %d, Country: %s",
           ap_info.ssid, ap_info.rssi, FORMAT_MAC(ap_info.bssid),
           ap_info.primary, country_code);
}

void wifi_reconnect_timer_cb(TimerHandle_t timer) {
  if (ble_is_connected()) {
    ESP_LOGI(TAG, "BLE is connected, not reconnecting");
    return;
  }
  WiFiManager &wifi_manager = WiFiManager::GetInstance();

  if (strlen(wifi_task_params.ssid) == 0) {
    ESP_LOGW(TAG, "No SSID provided; not reconnecting");
    return;
  }
  if (wifi_task_params.skip_reconnect) {
    ESP_LOGW(TAG, "Skipping Wi-Fi reconnection");
    return;
  }
  wifi_manager.GetWiFi(wifi_task_params.ssid, wifi_task_params.passwd);
  ESP_LOGI(TAG, "Reconnecting to Wi-Fi AP %s", wifi_task_params.ssid);
  if (!start_wifi_task(wifi_task_params.ssid, wifi_task_params.passwd)) {
    ESP_LOGW(TAG, "Failed to start Wi-Fi task; not reconnecting");
    return;
  }
  AnimationController::GetInstance().set_animation(AnimationId::WIFI_CONNECTING,
                                                   LoopMode::FOREVER);
}

void start_wifi_reconnect_timer(int interval_ms) {
  stop_wifi_reconnect_timer();
  AnimationController::GetInstance().stop_animation(
      AnimationId::WIFI_CONNECTING);

  if (WiFiManager::GetInstance().AllWiFiTried()) {
    interval_ms = WIFI_RECONNECT_LOOP_INTERVAL_MS;
  }

  // One-shot timer
  wifi_reconnect_timer =
      xTimerCreate("wifi_reconnect_timer", pdMS_TO_TICKS(interval_ms), pdFALSE,
                   nullptr, wifi_reconnect_timer_cb);
  if (wifi_reconnect_timer) {
    ESP_LOGI(TAG, "Starting Wi-Fi reconnect timer at %d ms", interval_ms);
    xTimerStart(wifi_reconnect_timer, 0);
  }
}

void stop_wifi_reconnect_timer() {
  if (wifi_reconnect_timer == NULL) {
    return;
  }
  xTimerStop(wifi_reconnect_timer, 0);
  xTimerDelete(wifi_reconnect_timer, 0);
  wifi_reconnect_timer = NULL;
}

void handle_wifi_disconnect_event(void *event_data, uint32_t *delay_ms) {
  bool retry_connect = !wifi_manually_disconnected;
  wifi_event_sta_disconnected_t *disconnected =
      (wifi_event_sta_disconnected_t *)event_data;
  ESP_LOGE(TAG, "Disconnected from %s (%s), RSSI: %d, reason: %d",
           disconnected->ssid, FORMAT_MAC(disconnected->bssid),
           disconnected->rssi, disconnected->reason);
  if (DeviceController::GetInstance()->is_upgrading()) {
    ESP_LOGW(TAG, "Wi-Fi disconnected while upgrading, rebooting");
    DeviceController::GetInstance()->reboot_after();
  }
  wifi_task_params.skip_reconnect =
      strcmp((char *)disconnected->ssid, wifi_task_params.ssid) != 0;
  switch (disconnected->reason) {
    case WIFI_REASON_AUTH_EXPIRE:
      ESP_LOGE(TAG, "WIFI_REASON_AUTH_EXPIRE");
      goto RECONNECT_TIMER;
      break;
    case WIFI_REASON_ASSOC_LEAVE:
      ESP_LOGI(TAG, "WIFI_REASON_ASSOC_LEAVE: %s", disconnected->ssid);
      break;
    case WIFI_REASON_AUTH_FAIL:
      ESP_LOGE(TAG, "WIFI_REASON_AUTH_FAIL: %s", disconnected->ssid);
      goto RECONNECT_TIMER;
      break;
    case WIFI_REASON_NO_AP_FOUND:
      ESP_LOGE(TAG, "WIFI_REASON_NO_AP_FOUND: %s, RSSI: %d", disconnected->ssid,
               disconnected->rssi);
      break;
    case WIFI_REASON_CONNECTION_FAIL:
      ESP_LOGE(TAG, "WIFI_REASON_CONNECTION_FAIL: %s", disconnected->ssid);
      break;
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
      // NOTE: maybe caused by wrong password, should pause retry
      ESP_LOGW(TAG, "WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: %s",
               disconnected->ssid);
      goto RECONNECT_TIMER;
      break;
    default:
      ESP_LOGE(TAG, "Unknown disconnect reason (%d)", disconnected->reason);
      break;
  }

  if (retry_connect) {
    retry_cnt++;
    if (retry_cnt > WIFI_STA_MAXIMUM_RETRY) {
      ESP_LOGW(TAG, "Maximum retry limit reached. Aborting connection to '%s'",
               disconnected->ssid);
      goto RECONNECT_TIMER;
    }
    ESP_LOGI(TAG, "Retrying to connect to '%s' in %" PRIu32 " ms",
             disconnected->ssid, *delay_ms);
    DELAY_MS(*delay_ms);
    if (ble_is_connected()) {
      ESP_LOGW(TAG, "BLE is connected. Aborting connection to '%s'",
               disconnected->ssid);
      goto RECONNECT_TIMER;
    }
    ble_adv_stop();
    AnimationController::GetInstance().set_animation(
        AnimationId::WIFI_CONNECTING, LoopMode::FOREVER);
    esp_wifi_connect();
    *delay_ms = MIN((*delay_ms) * 2, WIFI_EVT_MAX_DELAY_MS_);
    return;
  }
  return;
RECONNECT_TIMER:
  wifi_manually_disconnected = true;
  retry_cnt = 0;
  *delay_ms = WIFI_RETRY_CONNECT_DELAY_MS;
  start_wifi_reconnect_timer(WIFI_RECONNECT_INTERVAL_MS);
  return;
}

static bool resolve_address(const char *hostname) {
  // Initialize addrinfo hints using C++ initializer list for clarity
  struct addrinfo hints = {
      .ai_family = AF_INET,
      .ai_socktype = SOCK_STREAM,
  };
  struct addrinfo *res = nullptr;

  // Perform DNS lookup
  int err = getaddrinfo(hostname, nullptr, &hints, &res);

  if (err != 0 || res == nullptr) {
    ESP_LOGE(TAG, "DNS lookup failed for %s: %d", hostname, err);
    if (res) {
      freeaddrinfo(res);
    }
    return false;
  }

  // DNS lookup succeeded
  freeaddrinfo(res);  // Free the allocated memory
  return true;
}

static bool check_connectivity() {
  esp_http_client_config_t config = {
      .method = HTTP_METHOD_GET,
      .timeout_ms = 3000,
  };
  const char **urls;
  if (MachineInfo::GetInstance().IsChina()) {
    urls = connectivity_check_endpoints_cn;
  } else {
    urls = connectivity_check_endpoints;
  }

  for (int i = 0; urls[i] != nullptr; ++i) {
    config.url = urls[i];
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
      ESP_LOGE(TAG, "Failed to initialize HTTP client for %s", config.url);
      continue;
    }

    // Perform the HTTP request
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
      int status = esp_http_client_get_status_code(client);
      if (status >= 200 && status < 300) {
        ESP_LOGI(TAG, "Connectivity check passed: %s", config.url);
        esp_http_client_cleanup(client);
        return true;
      }
      ESP_LOGE(TAG, "Unexpected HTTP status %s: %d", config.url, status);
    } else {
      ESP_ERROR_COMPLAIN(err, "HTTP request failed: %s", config.url);
    }

    esp_http_client_cleanup(client);
  }

  // Return false if none of the URLs returned a 204 status
  return false;
}

static bool check_network_connectivity() {
  // Check external network access
  if (!check_connectivity()) {
    ESP_LOGE(TAG, "Failed to access connectivity check URL");
    WiFiManager::GetInstance().AddConnectivityFailure();
    return false;
  }

  // Check if MQTT broker can be resolved
  if (!resolve_address(MQTT_APP_HOST)) {
    ESP_LOGE(TAG, "Failed to resolve MQTT broker: %s", MQTT_APP_HOST);
    WiFiManager::GetInstance().AddDNSResolutionFailure();
    MQTTClient::GetInstance()->SetFallbackIP();
  }

  // Check if syslog server can be resolved
  if (!resolve_address(SYSLOG_HOST)) {
    ESP_LOGE(TAG, "Failed to resolve SysLog server: %s", SYSLOG_HOST);
    WiFiManager::GetInstance().AddDNSResolutionFailure();
  }

  // All checks passed
  return true;
}

bool is_internet_accessible() { return has_internet_access; }
