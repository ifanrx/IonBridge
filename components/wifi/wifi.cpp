#include "wifi.h"

#include <string.h>
#include <sys/param.h>

#include <algorithm>
#include <cctype>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>  // for std::memcpy
#include <string>

#include "ble.h"
#include "controller.h"
#include "esp_bit_defs.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_event_base.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_netif_types.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_wifi_types_generic.h"
#include "freertos/event_groups.h"  // IWYU pragma: keep
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "ionbridge.h"
#include "machine_info.h"
#include "mqtt_app.h"
#include "sdkconfig.h"
#include "syslog.h"
#include "utils.h"
#include "wifi_manager.h"
#include "wifi_state.h"

#ifdef CONFIG_LWIP_IPV6
#include "lwip/ip6_addr.h"
#endif

#define WIFI_RECORD_COUNT CONFIG_WIFI_RECORD_COUNT

#define WIFI_STA_START_TIMEOUT CONFIG_ESP_WIFI_STA_START_TIMEOUT

static EventGroupHandle_t s_wifi_event_group;

#define WIFI_STA_STARTED_BIT BIT0

static const char *TAG = "WiFi";
static esp_ip4_addr_t local_ip;
#ifdef CONFIG_LWIP_IPV6
static esp_ip6_addr_t local_ip6;
#endif

#define WAIT_FOR_NOTIFICATION(delay_ms, ssid, goto_tag)                        \
  if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(delay_ms)) == pdPASS) {        \
    ESP_LOGI(TAG, "Exiting Wi-Fi task for SSID: %s after notification", ssid); \
    goto goto_tag;                                                             \
  }

static bool wifi_is_ready = false, wifi_is_connected_ = false;

TaskHandle_t wifi_task_handle = NULL;
esp_event_handler_instance_t wifi_event_handler_instance,
    ip_event_handler_instance;

static void ip_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data) {
  if (event_base != IP_EVENT) {
    ESP_LOGE(TAG, "Unhandled event base: %s", event_base);
    return;
  }
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
      wifi_controller.Notify(WiFiEventType::CHECKING_CONN);
      break;
    }
    case IP_EVENT_STA_LOST_IP:
      ESP_LOGE(TAG, "IP address lost");
      ble_adv_delay_start();
      SyslogClient::GetInstance().Cleanup();
      wifi_controller.Notify(WiFiEventType::STOP_WEB_SERVER);
      break;
    default:
      ESP_LOGE(TAG, "Unhandled IP_EVENT: %" PRIu32, event_id);
      break;
  }
  return;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base != WIFI_EVENT) {
    ESP_LOGE(TAG, "Unhandled event base: %s", event_base);
    return;
  }
  WiFiManager &wifi_manager = WiFiManager::GetInstance();
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
      wifi_controller.OnConnected(event_data);
      break;
    case WIFI_EVENT_SCAN_DONE:
      ESP_LOGD(TAG, "WIFI_EVENT_SCAN_DONE");
      ESP_ERROR_COMPLAIN(wifi_manager.GetScanResult(), "GetScanResult");
      wifi_controller.Notify(WiFiEventType::SCAN_DONE, false);
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
      wifi_controller.OnDisconnected(event_data);
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
      esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
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

esp_err_t wifi_disconnect(bool reconnect_next_wifi) {
  // Try to disconnect from the AP if already connected
  ESP_RETURN_ON_ERROR(esp_wifi_disconnect(), TAG, "esp_wifi_disconnect");
  wifi_controller.SetReconnect(!reconnect_next_wifi, reconnect_next_wifi);
  wifi_is_connected_ = false;
  ESP_LOGI(TAG, "Wi-Fi disconnected");
  return ESP_OK;
}

bool wifi_is_connected() { return wifi_is_connected_ && local_ip.addr != 0; }

void get_ipv4_addr(uint8_t *data) {
  if (!wifi_is_connected()) {
    return;
  }
  // 假设 local_ip 已经是网络字节序的 uint32_t
  memcpy(data, &local_ip, 4);
}

#ifdef CONFIG_LWIP_IPV6
void get_ipv6_addr(uint8_t *data) {
  if (!wifi_is_connected()) {
    return;
  }
  // 假设 local_ip6 已经是网络字节序的 16 字节地址
  memcpy(data, &local_ip6, 16);
}
#endif

esp_err_t wifi_close() {
  ESP_LOGI(TAG, "Closing the WiFi service");
  if (wifi_task_handle != NULL) {
    // Wait a bit to ensure the task has time to exit and clean up
    DELAY_MS(500);
  }
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

void WiFiController::StartTask() {
  ESP_LOGI(TAG, "Creating the Wi-Fi task");
  BaseType_t res = xTaskCreate(WiFiController::Run, "wifi_task", 4 * 1024,
                               nullptr, 17, &wifi_task_handle);
  if (res != pdPASS) {
    ESP_LOGE(TAG, "Failed to create the Wi-Fi task");
    return;
  }
}

void WiFiController::Run(void *taskParams) { wifi_controller.Loop(); }

void WiFiController::SetState(WiFiStateType type) {
  switch (type) {
    case WiFiStateType::CONFIGURATION:
      SetState(&WiFiConfigurationState::GetInstance());
      break;
    case WiFiStateType::SCAN_AND_CONNECT:
      SetState(&WiFiScanAndConnectState::GetInstance());
      break;
    case WiFiStateType::CONNECTING:
      SetState(&WiFiConnectingState::GetInstance());
      break;
    case WiFiStateType::CONNECTED:
      SetState(&WiFiConnectedState::GetInstance());
      break;
    case WiFiStateType::CHECKING_CONN:
      SetState(&WiFiCheckingConnState::GetInstance());
      break;
    case WiFiStateType::STARTING_MQTT:
      SetState(&WiFiStartingMQTTState::GetInstance());
      break;
    case WiFiStateType::IDLE:
      SetState(&WiFiIdleState::GetInstance());
      break;
    case WiFiStateType::DISCONNECTED:
      SetState(&WiFiDisconnectedState::GetInstance());
      break;
    case WiFiStateType::RECONNECTING:
      SetState(&WiFiReconnectingState::GetInstance());
      break;
    case WiFiStateType::WAITING_FOR_NEXT:
      SetState(&WiFiWaitingForNextState::GetInstance());
      break;
    case WiFiStateType::SCANNING:
      SetState(&WiFiScanningState::GetInstance());
      break;
    case WiFiStateType::CONNECTION_ABORT:
      SetState(&WiFiConnectionAbortState::GetInstance());
      break;
    case WiFiStateType::WAITING_FOR_DEFAULT:
      SetState(&WiFiWaitingForDefaultState::GetInstance());
      break;
    default:
      ESP_LOGE(TAG, "Unknown WiFiStateType: %d", static_cast<int>(type));
      break;
  }
}

void WiFiController::Loop() {
  ctx_ = &WiFiStateContext::GetInstance();
  ctx_->Reset();

  BaseType_t xResult;
  uint32_t val;
  WiFiEventType event = WiFiEventType::NONE;
  state_ = &WiFiConfigurationState::GetInstance();
  state_->Entering(ctx_);
  while (true) {
    event = WiFiEventType::NONE;
    xResult = xTaskNotifyWait(pdTRUE, pdTRUE, &val, pdMS_TO_TICKS(10));
    if (xResult == pdPASS) {
      event = (WiFiEventType)val;
      ESP_LOGD(TAG, "Receive WiFi event: %s", WiFiEventTypeToString(event));
    }
    state_->Handle(ctx_, event);
  }
}

void WiFiController::SetState(WiFiState *state) {
  if (state_->Type() == state->Type()) {
    return;
  }
  ESP_LOGI(TAG, "WiFi state change: %s -> %s", state_->ToString().c_str(),
           state->ToString().c_str());
  ctx_->SetLastState(state_->Type());
  state_->Exiting(ctx_);
  state_ = state;
  state_->Entering(ctx_);
}

void WiFiController::Notify(WiFiEventType event, bool overwrite) {
  xTaskNotify(wifi_task_handle, static_cast<uint32_t>(event),
              overwrite ? eSetValueWithOverwrite : eSetValueWithoutOverwrite);
}

bool WiFiController::SwitchWiFi(const char *ssid, const char *passwd) {
  if (!wifi_is_ready) {
    ESP_LOGE(TAG,
             "Failed to switch the Wi-Fi because it has not been initialized.");
    return false;
  }
  wifi_config_t &wifi_config = ctx_->GetWiFiConfig();
  char *connect_wifi_ssid = reinterpret_cast<char *>(wifi_config.sta.ssid);
  size_t connect_ssid_length = strlen(connect_wifi_ssid);
  if (IsConnectingOrConnected() && connect_ssid_length > 0 &&
      strncmp(connect_wifi_ssid, ssid, connect_ssid_length) == 0) {
    ESP_LOGI(TAG, "WiFi %s is connecting or connected", ssid);
    return true;
  }

  if (ssid != NULL && passwd != NULL) {
    ESP_LOGI(TAG, "Using the provided SSID and password");
    ctx_->Reset();
    ctx_->SetSwitchingWiFi(true);
    ctx_->SetWiFi(ssid, passwd);
  }
  Notify(WiFiEventType::SWITCHING);
  return true;
}

void WiFiController::OnConnected(void *event_data) {
  Notify(WiFiEventType::CONNECTED);
}

void WiFiController::OnDisconnected(void *event_data) {
  WiFiManager::GetInstance().OnDisconnected();
  MQTTClient::GetInstance()->Stop();
  SyslogClient::GetInstance().Cleanup();
  wifi_event_sta_disconnected_t *disconnected =
      (wifi_event_sta_disconnected_t *)event_data;
  ESP_LOGE(TAG, "Disconnected from %s (%s), RSSI: %d, reason: %d",
           disconnected->ssid, FORMAT_MAC(disconnected->bssid),
           disconnected->rssi, disconnected->reason);
  if (DeviceController::GetInstance()->is_upgrading()) {
    ESP_LOGW(TAG, "Wi-Fi disconnected while upgrading, rebooting");
    DeviceController::GetInstance()->reboot_after();
  }
  ctx_->SetDisconnectdSSID(reinterpret_cast<const char *>(disconnected->ssid));
  switch (disconnected->reason) {
    case WIFI_REASON_AUTH_EXPIRE:
      ESP_LOGE(TAG, "WIFI_REASON_AUTH_EXPIRE");
      goto RECONNECT_NEXT_WIFI;
      break;
    case WIFI_REASON_ASSOC_LEAVE:
      ESP_LOGI(TAG, "WIFI_REASON_ASSOC_LEAVE: %s", disconnected->ssid);
      break;
    case WIFI_REASON_AUTH_FAIL:
      ESP_LOGE(TAG, "WIFI_REASON_AUTH_FAIL: %s", disconnected->ssid);
      goto RECONNECT_NEXT_WIFI;
      break;
    case WIFI_REASON_NO_AP_FOUND:
      ESP_LOGE(TAG, "WIFI_REASON_NO_AP_FOUND: %s, RSSI: %d", disconnected->ssid,
               disconnected->rssi);
      ctx_->ResetWiFiBSSID();
      break;
    case WIFI_REASON_CONNECTION_FAIL:
      ESP_LOGE(TAG, "WIFI_REASON_CONNECTION_FAIL: %s", disconnected->ssid);
      break;
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
      // NOTE: maybe caused by wrong password, should pause retry
      ESP_LOGW(TAG, "WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: %s",
               disconnected->ssid);
      goto RECONNECT_NEXT_WIFI;
      break;
    case WIFI_REASON_SA_QUERY_TIMEOUT:
      ESP_LOGE(TAG, "WIFI_REASON_SA_QUERY_TIMEOUT: %s", disconnected->ssid);
      break;
    default:
      ESP_LOGE(TAG, "Unknown disconnect reason (%d)", disconnected->reason);
      break;
  }
  SetReconnect(false, false);
  Notify(WiFiEventType::DISCONNECTED);
  return;
RECONNECT_NEXT_WIFI:
  SetReconnect(false, true);
  Notify(WiFiEventType::DISCONNECTED);
}
WiFiController wifi_controller;
