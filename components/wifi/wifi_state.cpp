#include "wifi_state.h"

#include <string.h>
#include <sys/param.h>

#include <cinttypes>
#include <cstdint>
#include <cstring>  // for std::memcpy
#include <memory>

#include "animation.h"
#include "app.h"
#include "ble.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_wifi_types_generic.h"
#include "freertos/event_groups.h"  // IWYU pragma: keep
#include "ionbridge.h"
#include "lwip/netdb.h"
#include "machine_info.h"
#include "mqtt_app.h"
#include "sdkconfig.h"
#include "syslog.h"
#include "utils.h"
#include "wifi.h"
#include "wifi_manager.h"
#include "wifi_state.h"

#ifdef CONFIG_ENABLE_WEB_SERVER
#include "server.h"

#define ENABLE_WEB_SERVER
#endif

#define WIFI_STA_MAXIMUM_RETRY CONFIG_ESP_WIFI_STA_MAXIMUM_RETRY
#define WIFI_RSSI_LOG_TIMER_INTERVAL CONFIG_WIFI_RSSI_LOG_TIMER_INTERVAL
#define WIFI_DEFAULT_SSID CONFIG_IONBRIDGE_WIFI_DEFAULT_SSID
#define WIFI_DEFAULT_PASSWD CONFIG_IONBRIDGE_WIFI_DEFAULT_PASSWORD
#define DEFAULT_WIFI_CONNECTION_DELAY CONFIG_DEFAULT_WIFI_CONNECTION_DELAY
#define WIFI_RECONNECT_INTERVAL_MS CONFIG_WIFI_RECONNECT_INTERVAL_MS
#define WIFI_RECONNECT_LOOP_INTERVAL_MS CONFIG_WIFI_RECONNECT_LOOP_INTERVAL_MS
#define WIFI_ABORT_WAIT_MS CONFIG_WIFI_ABORT_WAIT_MS

#define WIFI_CONNECTING_MAX_RETRY 10

#ifdef CONFIG_WIFI_CONNECT_SET_BSSID
#define WIFI_CONNECT_SET_BSSID
#endif

#define MQTT_APP_URI CONFIG_MQTT_APP_URI
#define MQTT_APP_HOST CONFIG_MQTT_APP_HOSTNAME
#define SYSLOG_HOST CONFIG_SYSLOG_HOST
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

static const char *TAG = "WiFiState";

static bool check_network_connectivity();

const char *WiFiEventTypeToString(WiFiEventType event) {
  switch (event) {
    case WiFiEventType::NONE:
      return "NONE";
    case WiFiEventType::SCANNING:
      return "SCANNING";
    case WiFiEventType::SCAN_DONE:
      return "SCAN_DONE";
    case WiFiEventType::SWITCHING:
      return "SWITCHING";
    case WiFiEventType::CONNECTED:
      return "CONNECTED";
    case WiFiEventType::DISCONNECTED:
      return "DISCONNECTED";
    case WiFiEventType::CHECKING_CONN:
      return "CHECKING_CONN";
    case WiFiEventType::MQTT_CONNECTED:
      return "MQTT_CONNECTED";
    case WiFiEventType::MQTT_DISCONNECTED:
      return "MQTT_DISCONNECTED";
    case WiFiEventType::ABORT:
      return "ABORT";
    case WiFiEventType::START_WEB_SERVER:
      return "START_WEB_SERVER";
    case WiFiEventType::STOP_WEB_SERVER:
      return "STOP_WEB_SERVER";
  }
  return "UNKNOWN";
}

void WiFiStateContext::Initialize() {
  wifi_scan_threshold_t *scan_threshold;
  wifi_sta_config_t *sta_config;
  wifi_config_ = std::make_unique<wifi_config_t>();

  ESP_RETURN_VOID_ON_ERROR(esp_wifi_get_config(WIFI_IF_STA, wifi_config_.get()),
                           TAG, "esp_wifi_get_config");

  // Wi-Fi configuration
  sta_config = &wifi_config_->sta;
  scan_threshold = &sta_config->threshold;

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
  ResetWiFiSSID();
}

void WiFiStateContext::SetWiFi(const char *ssid, const char *passwd) {
  strcpy(reinterpret_cast<char *>(wifi_config_->sta.ssid), ssid);
  strcpy(reinterpret_cast<char *>(wifi_config_->sta.password), passwd);
}

void WiFiStateContext::ResetWiFiSSID() {
  if (wifi_config_ == nullptr) {
    return;
  }
  memset(wifi_config_->sta.ssid, 0, sizeof(wifi_config_->sta.ssid));
}

void WiFiStateContext::ResetWiFiBSSID() {
  if (wifi_config_ == nullptr) {
    return;
  }
  wifi_config_->sta.bssid_set = false;
}

void WiFiStateContext::SetDisconnectdSSID(const char *ssid) {
  strcpy(disconnected_ssid_, ssid);
}

void WiFiStateContext::Reset() {
  skip_reconnect_ = false;
  reconnect_next_wifi_ = false;
  switching_wifi_ = false;
  connection_start_time_ = 0;
  ResetWiFiSSID();
  ResetWiFiBSSID();
}

void WiFiConfigurationState::Entering(WiFiStateContext *ctx) {
  ctx->Initialize();
}

void WiFiConfigurationState::Handle(WiFiStateContext *ctx,
                                    WiFiEventType event) {
  wifi_controller.SetState(WiFiStateType::SCAN_AND_CONNECT);
}

void WiFiScanAndConnectState::Entering(WiFiStateContext *ctx) {
  WiFiManager &wifi_manager = WiFiManager::GetInstance();
  scan_failed_ = wifi_manager.ScanWiFi() != ESP_OK;
}

void WiFiScanAndConnectState::Handle(WiFiStateContext *ctx,
                                     WiFiEventType event) {
  if (scan_failed_) {
    ESP_LOGE(TAG, "Failed to scan wifi to connect");
    wifi_controller.SetState(WiFiStateType::WAITING_FOR_NEXT);
    return;
  }
  WiFiManager &wifi_manager = WiFiManager::GetInstance();
  switch (event) {
    case WiFiEventType::SCAN_DONE: {
      break;
    }
    case WiFiEventType::SWITCHING:
      wifi_controller.SetState(WiFiStateType::CONNECTING);
      return;
    default:
      ESP_LOGD(TAG, "%s ignore %s", ToString().c_str(),
               WiFiEventTypeToString(event));
      return;
  }
  char ssid[WIFI_SSID_MAXIMUM] = {};
  char passwd[WIFI_PASSWD_MAXIMUM] = {};

  if (!wifi_manager.GetWiFi(ssid, passwd)) {
    wifi_controller.SetState(WiFiStateType::WAITING_FOR_NEXT);
    return;
  }

  ctx->SetWiFi(ssid, passwd);
  if (strcmp(ssid, WIFI_DEFAULT_SSID) == 0) {
    wifi_controller.SetState(WiFiStateType::WAITING_FOR_DEFAULT);
    return;
  }

#ifdef WIFI_CONNECT_SET_BSSID
  wifi_config_t &wifi_config = ctx->GetWiFiConfig();
  wifi_config.sta.bssid_set = wifi_manager.GetBSSIDBySSID(
      reinterpret_cast<const char *>(wifi_config.sta.ssid),
      wifi_config.sta.bssid);
#endif
  wifi_controller.SetState(WiFiStateType::CONNECTING);
}

esp_err_t start_connect_wifi(wifi_config_t &wifi_config) {
  const wifi_sta_config_t *sta_config = &wifi_config.sta;
  ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), TAG,
                      "esp_wifi_set_config");
  ble_adv_stop();  // Stop BLE advertising if using custom credentials
  DELAY_MS(100);   // delay to avoid wifi animation conflict with ble animation
  if (sta_config->bssid_set) {
    ESP_LOGI(TAG, "Connecting to the Access Point: %s, bssid: %s, authmode: %d",
             sta_config->ssid, FORMAT_MAC(sta_config->bssid),
             sta_config->threshold.authmode);
  } else {
    ESP_LOGI(TAG, "Connecting to the Access Point: %s, authmode: %d",
             sta_config->ssid, sta_config->threshold.authmode);
  }
  ESP_RETURN_ON_ERROR(esp_wifi_connect(), TAG, "esp_wifi_connect");
  AnimationController::GetInstance().StartAnimation(
      AnimationId::WIFI_CONNECTING, true);
  return ESP_OK;
}

void WiFiConnectingState::StartConnect(WiFiStateContext *ctx) {
  retry_cnt_++;
  if (!disconnected_) {
    ESP_LOGW(TAG, "disconnect wifi before connect");
    ESP_RETURN_VOID_ON_ERROR(
        esp_wifi_disconnect(), TAG,
        "Failed to disconnect from WiFi before connecting: %d", err_rc_);
    disconnected_ = true;
    DELAY_MS(1000);
  }
  if (!connecting_) {
    connecting_ = start_connect_wifi(ctx->GetWiFiConfig()) == ESP_OK;
  }
}

void WiFiConnectingState::Entering(WiFiStateContext *ctx) {
  disconnected_ = false;
  connecting_ = false;
  retry_cnt_ = 0;
  StartConnect(ctx);
}

void WiFiConnectingState::Handle(WiFiStateContext *ctx, WiFiEventType event) {
  switch (event) {
    case WiFiEventType::CONNECTED:
      wifi_controller.SetState(WiFiStateType::CONNECTED);
      return;
    case WiFiEventType::DISCONNECTED: {
      if (strcmp(ctx->GetDisconnectdSSID(), ctx->GetWiFiSSID()) == 0) {
        wifi_controller.SetState(WiFiStateType::DISCONNECTED);
      }
      return;
    }
    case WiFiEventType::SWITCHING: {
      ESP_LOGI(TAG, "%s switching wifi", ToString().c_str());
      disconnected_ = false;
      connecting_ = false;
      break;
    }
    case WiFiEventType::ABORT:
      wifi_controller.SetState(WiFiStateType::CONNECTION_ABORT);
      return;
    default:
      ESP_LOGD(TAG, "%s ignore %s", ToString().c_str(),
               WiFiEventTypeToString(event));
      break;
  }
  if (strlen(ctx->GetWiFiSSID()) == 0) {
    ESP_LOGE(TAG, "No SSID to connect");
    wifi_controller.SetState(WiFiStateType::WAITING_FOR_NEXT);
    return;
  }
  if (retry_cnt_ >= WIFI_CONNECTING_MAX_RETRY) {
    ESP_LOGE(TAG, "Failed to connect to Wi-Fi");
    wifi_controller.SetState(WiFiStateType::CONNECTION_ABORT);
    return;
  }
  if (!connecting_) {
    StartConnect(ctx);
  }
}

void WiFiConnectingState::Exiting(WiFiStateContext *ctx) {
  AnimationController::GetInstance().StopAnimation(
      AnimationId::WIFI_CONNECTING);
}

void WiFiConnectedState::Entering(WiFiStateContext *ctx) {
  ctx->connection_start_time_ = 0;
}

void WiFiConnectedState::Handle(WiFiStateContext *ctx, WiFiEventType event) {
  switch (event) {
    case WiFiEventType::SCANNING:
      wifi_controller.SetState(WiFiStateType::SCANNING);
      return;
    case WiFiEventType::SWITCHING:
      wifi_controller.SetState(WiFiStateType::CONNECTING);
      return;
    case WiFiEventType::CHECKING_CONN:
      wifi_controller.SetState(WiFiStateType::CHECKING_CONN);
      return;
    case WiFiEventType::DISCONNECTED:
      wifi_controller.SetState(WiFiStateType::DISCONNECTED);
      return;
    case WiFiEventType::ABORT:
      wifi_controller.SetState(WiFiStateType::CONNECTION_ABORT);
      return;
    default:
      ESP_LOGD(TAG, "%s ignore %s", ToString().c_str(),
               WiFiEventTypeToString(event));
      break;
  }
}

void WiFiCheckingConnState::Handle(WiFiStateContext *ctx, WiFiEventType event) {
  if (check_network_connectivity()) {
    WiFiManager::GetInstance().OnConnected();
    WiFiManager::GetInstance().AddWifiConnectionTime(
        esp_timer_get_time() / 1e6 - ctx->connection_start_time_);
    ctx->connection_start_time_ = 0;
#ifdef CONFIG_ENABLE_GLOBAL_LOGGING_COLLECTOR
    ESP_LOGW(TAG, "Remote logging is enabled");
    SyslogClient &syslog = SyslogClient::GetInstance();
    ESP_ERROR_COMPLAIN(syslog.Initialize(), "Failed to initialize syslog");
    syslog.Register();
#endif

    wifi_controller.SetState(WiFiStateType::STARTING_MQTT);
  } else {
    wifi_controller.SetState(WiFiStateType::WAITING_FOR_NEXT);
  }
}

void WiFiStartingMQTTState::Entering(WiFiStateContext *ctx) {
  client_started_ = false;
  if (App::GetInstance().IsInitialized()) {
    client_started_ = true;
    MQTTClient::GetInstance()->Start();
  }
}

void WiFiStartingMQTTState::Handle(WiFiStateContext *ctx, WiFiEventType event) {
  switch (event) {
    case WiFiEventType::SWITCHING:
      wifi_controller.SetState(WiFiStateType::CONNECTING);
      return;
    case WiFiEventType::MQTT_CONNECTED:
      wifi_controller.SetState(WiFiStateType::IDLE);
      return;
    case WiFiEventType::MQTT_DISCONNECTED:
      wifi_controller.SetState(WiFiStateType::WAITING_FOR_NEXT);
      return;
    case WiFiEventType::ABORT:
      wifi_controller.SetState(WiFiStateType::CONNECTION_ABORT);
      return;
    case WiFiEventType::DISCONNECTED:
      wifi_controller.SetState(WiFiStateType::DISCONNECTED);
      return;
    case WiFiEventType::CONNECTED:
      client_started_ = false;
      break;
    default:
      ESP_LOGD(TAG, "%s ignore %s", ToString().c_str(),
               WiFiEventTypeToString(event));
      break;
  }
  if (!client_started_ && App::GetInstance().IsInitialized()) {
    client_started_ = true;
    MQTTClient::GetInstance()->Start();
  }
}

void WiFiIdleState::Entering(WiFiStateContext *ctx) {
  ctx->SetReconnect(false, false);
  ctx->SetSwitchingWiFi(false);
  next_log_at_ = 0;
}

void WiFiIdleState::Exiting(WiFiStateContext *ctx) {}

void WiFiIdleState::Handle(WiFiStateContext *ctx, WiFiEventType event) {
  switch (event) {
    case WiFiEventType::SCANNING:
      wifi_controller.SetState(WiFiStateType::SCANNING);
      break;
    case WiFiEventType::SWITCHING:
      wifi_controller.SetState(WiFiStateType::CONNECTING);
      break;
    case WiFiEventType::DISCONNECTED:
      wifi_controller.SetState(WiFiStateType::DISCONNECTED);
      break;
    case WiFiEventType::ABORT:
      wifi_controller.SetState(WiFiStateType::CONNECTION_ABORT);
      return;
    case WiFiEventType::START_WEB_SERVER:
      break;
    case WiFiEventType::STOP_WEB_SERVER:
      break;
    default:
      ESP_LOGD(TAG, "%s ignore %s", ToString().c_str(),
               WiFiEventTypeToString(event));
      break;
  }
  int64_t ts = esp_timer_get_time();
  if (ts > next_log_at_) {
    LogRSSI();
    next_log_at_ = ts + WIFI_RSSI_LOG_TIMER_INTERVAL * 1e6;
  }
}

void WiFiIdleState::LogRSSI() {
  char country_code[3];  // Buffer for null-terminated country code
  wifi_ap_record_t ap_info;

  // Fetch the current AP information and handle potential errors
  ESP_RETURN_VOID_ON_ERROR(esp_wifi_sta_get_ap_info(&ap_info), TAG,
                           "esp_wifi_sta_get_ap_info");

  // Safely copy the country code and ensure null termination
  strncpy(country_code, ap_info.country.cc, 2);
  country_code[2] = '\0';

  // Log the current Wi-Fi access point details
  ESP_LOGI(TAG,
           "Current SSID: %s, RSSI: %d, BSSID: %s, Channel: %d, Country: %s, "
           "Current Country: %s",
           ap_info.ssid, ap_info.rssi, FORMAT_MAC(ap_info.bssid),
           ap_info.primary, country_code,
           MachineInfo::GetInstance().GetCountryCode().c_str());
}

void WiFiDisconnectedState::Handle(WiFiStateContext *ctx, WiFiEventType event) {
  switch (event) {
    case WiFiEventType::SCANNING:
      wifi_controller.SetState(WiFiStateType::SCANNING);
      return;
    case WiFiEventType::SWITCHING:
      wifi_controller.SetState(WiFiStateType::CONNECTING);
      return;
    default:
      ESP_LOGD(TAG, "%s ignore %s", ToString().c_str(),
               WiFiEventTypeToString(event));
      break;
  }
  if (ctx->IsSkipReconnect()) {
    wifi_controller.SetState(WiFiStateType::CONNECTION_ABORT);
  } else if (ctx->IsReconnectNextWiFi()) {
    wifi_controller.SetState(WiFiStateType::WAITING_FOR_NEXT);
  } else {
    wifi_controller.SetState(WiFiStateType::RECONNECTING);
  }
}

void WiFiReconnectingState::Entering(WiFiStateContext *ctx) {
  connecting_ = false;
  retry_cnt_ = 0;
  delay_ms_ = WIFI_RETRY_CONNECT_DELAY_MS;
  if (ctx->IsSwitchingWiFi()) {
    delay_ms_ = 0;
  }
  retry_cnt_++;
  reconnect_at_ = esp_timer_get_time() + delay_ms_ * 1e3;
  ESP_LOGI(TAG, "Retrying %d times to connect to '%s' in %" PRIu32 " ms",
           retry_cnt_, ctx->GetWiFiSSID(), delay_ms_);
}

void WiFiReconnectingState::Handle(WiFiStateContext *ctx, WiFiEventType event) {
  switch (event) {
    case WiFiEventType::SWITCHING:
      wifi_controller.SetState(WiFiStateType::CONNECTING);
      return;
    case WiFiEventType::SCANNING: {
      if (connecting_) {
        ESP_LOGE(TAG, "Cannot scan while reconnecting to WiFi");
      } else {
        wifi_controller.SetState(WiFiStateType::SCANNING);
      }
      return;
    }
    case WiFiEventType::CONNECTED:
      wifi_controller.SetState(WiFiStateType::CONNECTED);
      return;
    case WiFiEventType::DISCONNECTED: {
      // some disconnections will skip the reconnection process, such as
      // auth_fail
      if (ctx->IsSkipReconnect()) {
        wifi_controller.SetState(WiFiStateType::WAITING_FOR_NEXT);
        return;
      }
      connecting_ = false;
      retry_cnt_++;
      if (retry_cnt_ > WIFI_STA_MAXIMUM_RETRY) {
        ESP_LOGW(TAG, "Maximum retry limit reached");
        wifi_controller.SetState(WiFiStateType::WAITING_FOR_NEXT);
        return;
      }
      delay_ms_ = MIN(delay_ms_ * 2, WIFI_EVT_MAX_DELAY_MS_);
      reconnect_at_ = esp_timer_get_time() + delay_ms_ * 1e3;
      ESP_LOGI(TAG, "Retrying %d times to connect to '%s' in %" PRIu32 " ms",
               retry_cnt_, ctx->GetWiFiSSID(), delay_ms_);
      break;
    }
    case WiFiEventType::ABORT:
      wifi_controller.SetState(WiFiStateType::CONNECTION_ABORT);
      return;
    default: {
      if (!connecting_ && esp_timer_get_time() > reconnect_at_) {
        connecting_ = start_connect_wifi(ctx->GetWiFiConfig()) == ESP_OK;
      }
      break;
    }
  }
}

void WiFiReconnectingState::Exiting(WiFiStateContext *ctx) {
  AnimationController::GetInstance().StopAnimation(
      AnimationId::WIFI_CONNECTING);
}

void WiFiWaitingForNextState::UpdateConnectAt() {
  uint32_t interval_ms = WIFI_RECONNECT_INTERVAL_MS;
  if (WiFiManager::GetInstance().AllWiFiTried()) {
    interval_ms = WIFI_RECONNECT_LOOP_INTERVAL_MS;
  }
  ESP_LOGI(TAG, "Retrying to connect next wifi in %" PRIu32 " ms", interval_ms);
  connect_at_ = esp_timer_get_time() + interval_ms * 1e3;
}

void WiFiWaitingForNextState::Entering(WiFiStateContext *ctx) {
  ESP_ERROR_COMPLAIN(
      esp_wifi_disconnect(),
      "Failed to disconnect from WiFi before attempting the next connection");
  ble_adv_start();
  ctx->Reset();
  UpdateConnectAt();
}

void WiFiWaitingForNextState::Handle(WiFiStateContext *ctx,
                                     WiFiEventType event) {
  switch (event) {
    case WiFiEventType::SCANNING:
      wifi_controller.SetState(WiFiStateType::SCANNING);
      break;
    case WiFiEventType::SWITCHING:
      wifi_controller.SetState(WiFiStateType::CONNECTING);
      break;
    case WiFiEventType::ABORT:
      wifi_controller.SetState(WiFiStateType::CONNECTION_ABORT);
      return;
    default:
      ESP_LOGD(TAG, "%s ignore %s", ToString().c_str(),
               WiFiEventTypeToString(event));
      break;
  }
  if (esp_timer_get_time() > connect_at_) {
    if (ble_is_connected()) {
      ESP_LOGW(TAG, "BLE is connected, skipping connection to the next WiFi");
      UpdateConnectAt();
      return;
    }
    WiFiManager &wifi_manager = WiFiManager::GetInstance();
    if (wifi_manager.AllWiFiTried()) {
      wifi_controller.SetState(WiFiStateType::SCAN_AND_CONNECT);
      return;
    }
    char ssid[WIFI_SSID_MAXIMUM] = {};
    char passwd[WIFI_PASSWD_MAXIMUM] = {};
    if (wifi_manager.GetWiFi(ssid, passwd)) {
      ctx->SetWiFi(ssid, passwd);
      wifi_controller.SetState(WiFiStateType::CONNECTING);
    } else {
      ESP_LOGW(TAG, "Unable to retrieve the next WiFi to connect");
      UpdateConnectAt();
    }
  }
}

void WiFiScanningState::Entering(WiFiStateContext *ctx) {
  done_ = false;
  WiFiManager &wifi_manager = WiFiManager::GetInstance();
  ESP_ERROR_COMPLAIN(wifi_manager.ScanWiFi(), "Failed to scan WiFi");
}

void WiFiScanningState::Handle(WiFiStateContext *ctx, WiFiEventType event) {
  switch (event) {
    case WiFiEventType::SCAN_DONE: {
      wifi_controller.SetState(ctx->GetLastState());
      break;
    }
    case WiFiEventType::DISCONNECTED:
      wifi_controller.SetState(WiFiStateType::DISCONNECTED);
      break;
    default: {
      ESP_LOGD(TAG, "%s ignore %s", ToString().c_str(),
               WiFiEventTypeToString(event));
      break;
    }
  }
}

void WiFiConnectionAbortState::Entering(WiFiStateContext *ctx) {
  ble_adv_start();
  disconnected_ = esp_wifi_disconnect() == ESP_OK;
  until_at_ = esp_timer_get_time() + WIFI_ABORT_WAIT_MS * 1e3;
}

void WiFiConnectionAbortState::Handle(WiFiStateContext *ctx,
                                      WiFiEventType event) {
  switch (event) {
    case WiFiEventType::SCANNING:
      wifi_controller.SetState(WiFiStateType::SCANNING);
      return;
    case WiFiEventType::SWITCHING:
      wifi_controller.SetState(WiFiStateType::CONNECTING);
      return;
    default:
      ESP_LOGD(TAG, "%s ignore %s", ToString().c_str(),
               WiFiEventTypeToString(event));
      break;
  }
  if (disconnected_ && esp_timer_get_time() > until_at_) {
    if (strlen(ctx->GetWiFiSSID()) == 0) {
      wifi_controller.SetState(WiFiStateType::WAITING_FOR_NEXT);
    } else {
      wifi_controller.SetState(WiFiStateType::RECONNECTING);
    }
  } else {
    disconnected_ = esp_wifi_disconnect() == ESP_OK;
  }
}

void WiFiWaitingForDefaultState::Entering(WiFiStateContext *ctx) {
  ble_adv_start();
  until_at_ = esp_timer_get_time() + DEFAULT_WIFI_CONNECTION_DELAY * 1e6;
}

void WiFiWaitingForDefaultState::Handle(WiFiStateContext *ctx,
                                        WiFiEventType event) {
  switch (event) {
    case WiFiEventType::SCANNING:
      wifi_controller.SetState(WiFiStateType::SCANNING);
      return;
    case WiFiEventType::SWITCHING:
      wifi_controller.SetState(WiFiStateType::CONNECTING);
      return;
    default:
      ESP_LOGD(TAG, "%s ignore %s", ToString().c_str(),
               WiFiEventTypeToString(event));
      break;
  }
  if (esp_timer_get_time() >= until_at_) {
    wifi_controller.SetState(WiFiStateType::CONNECTING);
  }
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
    ESP_LOGI(TAG, "Accessing %s", config.url);
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
    MQTTClient::GetInstance()->SetFallbackConfig();
  }

  // Check if syslog server can be resolved
  if (!resolve_address(SYSLOG_HOST)) {
    ESP_LOGE(TAG, "Failed to resolve SysLog server: %s", SYSLOG_HOST);
    WiFiManager::GetInstance().AddDNSResolutionFailure();
  }

  // All checks passed
  return true;
}
