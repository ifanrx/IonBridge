#include "wifi_state.h"

#include <string.h>
#include <sys/param.h>

#include <cinttypes>
#include <cstdint>
#include <cstring>  // for std::memcpy
#include <memory>

#include "ble.h"
#include "controller.h"
#include "display_manager.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_wifi_types_generic.h"
#include "freertos/event_groups.h"  // IWYU pragma: keep
#include "freertos/idf_additions.h"
#include "ionbridge.h"
#include "machine_info.h"
#include "mqtt_app.h"
#include "portmacro.h"
#include "sdkconfig.h"
#include "utils.h"
#include "wifi.h"
#include "wifi_manager.h"
#include "wifi_state.h"

#ifdef CONFIG_ENABLE_WEB_SERVER
#include "server.h"

#define ENABLE_WEB_SERVER
#endif

#define MQTT_MAX_RECONNECT_ATTEMPTS CONFIG_MQTT_MAX_RECONNECT_ATTEMPTS
#define MQTT_APP_STABLE_CONNECTION_DURATION 10 * 60 * 1e6

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
#define FALLBACK_IP CONFIG_FALLBACK_IP

#ifdef CONFIG_ENABLE_REPORT_SYSLOG
#define SYSLOG_HOST CONFIG_SYSLOG_HOST
#endif

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

static bool network_accessible = false, check_conn_task_running = false;
static void check_network_connectivity(void *);

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
    case WiFiEventType::ABORT:
      return "ABORT";
    case WiFiEventType::START_WEB_SERVER:
      return "START_WEB_SERVER";
    case WiFiEventType::STOP_WEB_SERVER:
      return "STOP_WEB_SERVER";
    case WiFiEventType::LOST_IP:
      return "LOST_IP";
    case WiFiEventType::STOP:
      return "STOP";
  }
  return "UNKNOWN";
}

void WiFiStateContext::Initialize() {
  wifi_config_ = std::make_unique<wifi_config_t>();

  esp_err_t err = esp_wifi_get_config(WIFI_IF_STA, wifi_config_.get());
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_wifi_get_config failed: %s", esp_err_to_name(err));
    // The unique_ptr will clean up automatically when leaving scope
    wifi_config_.reset();
    return;
  }

  // Wi-Fi configuration
  wifi_sta_config_t *sta_config = &wifi_config_->sta;
  wifi_scan_threshold_t *scan_threshold = &sta_config->threshold;

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
  strlcpy(reinterpret_cast<char *>(wifi_config_->sta.ssid), ssid,
          sizeof(wifi_config_->sta.ssid));
  strlcpy(reinterpret_cast<char *>(wifi_config_->sta.password), passwd,
          sizeof(wifi_config_->sta.password));
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
  DisplayManager::GetInstance().SetAnimation(AnimationType::WIFI_CONNECTING,
                                             true);
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
  DisplayManager::GetInstance().SetAnimation(AnimationType::IDLE_ANIMATION,
                                             true);
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
    case WiFiEventType::LOST_IP:
      wifi_controller.SetState(WiFiStateType::WAITING_FOR_NEXT);
      return;
    default:
      ESP_LOGD(TAG, "%s ignore %s", ToString().c_str(),
               WiFiEventTypeToString(event));
      break;
  }
}

void WiFiCheckingConnState::Entering(WiFiStateContext *ctx) {
  StartCheckConnTask();
}

void WiFiCheckingConnState::Handle(WiFiStateContext *ctx, WiFiEventType event) {
  switch (event) {
    case WiFiEventType::DISCONNECTED: {
      wifi_controller.SetState(WiFiStateType::DISCONNECTED);
      return;
    } break;
    case WiFiEventType::ABORT: {
      wifi_controller.SetState(WiFiStateType::CONNECTION_ABORT);
      return;
    } break;
    default:
      break;
  }
  if (network_accessible) {
    WiFiManager::GetInstance().OnConnected();
    WiFiManager::GetInstance().AddWifiConnectionTime(
        esp_timer_get_time() / 1e6 - ctx->connection_start_time_);
    ctx->connection_start_time_ = 0;
    wifi_controller.SetState(WiFiStateType::IDLE);
    return;
  }
  if (!check_conn_task_running) {
    wifi_controller.SetState(WiFiStateType::WAITING_FOR_NEXT);
    return;
  }
  ExecuteCheckConnTask();
}

void WiFiCheckingConnState::Exiting(WiFiStateContext *ctx) {
  StopCheckConnTask();
}

void WiFiCheckingConnState::StartCheckConnTask() {
  StopCheckConnTask();
  ESP_LOGI(TAG, "Starting network connectivity check task");
  xTaskCreate(check_network_connectivity, "checking_conn", 1024 * 2, nullptr, 5,
              &task_handle_);
}

void WiFiCheckingConnState::StopCheckConnTask() {
  if (task_handle_ != nullptr) {
    ESP_LOGI(TAG, "Stopping network connectivity check task");
    while (check_conn_task_running) {
      xTaskNotify(task_handle_, (uint32_t)CheckConnState::EXIT,
                  eSetValueWithOverwrite);
      DELAY_MS(50);
    }
    task_handle_ = nullptr;
  }
}

void WiFiCheckingConnState::ExecuteCheckConnTask() {
  if (task_handle_ != nullptr && check_conn_task_running) {
    ESP_LOGD(TAG, "Executing network connectivity check task");
    xTaskNotify(task_handle_, (uint32_t)CheckConnState::CHECKING,
                eSetValueWithoutOverwrite);
  }
}

void WiFiIdleState::Entering(WiFiStateContext *ctx) {
  temporary_exiting_ = false;
  client_started_ = false;
  if (DeviceController::GetInstance().is_normally_booted() &&
      MQTTClient::Initialize()) {
    client_started_ = MQTTClient::GetInstance()->Start();
  }
  ctx->SetReconnect(false, false);
  ctx->SetSwitchingWiFi(false);
  next_log_at_ = 0;
}

void WiFiIdleState::Exiting(WiFiStateContext *ctx) {
  if (temporary_exiting_) {
    return;
  }
  MQTTClient::DestroyInstance();
#ifdef ENABLE_WEB_SERVER
  ESP_ERROR_COMPLAIN(stop_web_server(),
                     "Failed to stop web server while exiting idle state");
#endif
}

void WiFiIdleState::Handle(WiFiStateContext *ctx, WiFiEventType event) {
  switch (event) {
    case WiFiEventType::SCANNING:
      temporary_exiting_ = true;
      wifi_controller.SetState(WiFiStateType::SCANNING);
      break;
    case WiFiEventType::SWITCHING:
      wifi_controller.SetState(WiFiStateType::CONNECTING);
      return;
    case WiFiEventType::DISCONNECTED:
      wifi_controller.SetState(WiFiStateType::DISCONNECTED);
      return;
    case WiFiEventType::ABORT:
      wifi_controller.SetState(WiFiStateType::CONNECTION_ABORT);
      return;
    case WiFiEventType::START_WEB_SERVER:
#ifdef ENABLE_WEB_SERVER
      ESP_ERROR_COMPLAIN(start_web_server(),
                         "Failed to start web server during idle state");
#endif
      break;
    case WiFiEventType::LOST_IP:
    case WiFiEventType::STOP_WEB_SERVER:
#ifdef ENABLE_WEB_SERVER
      ESP_ERROR_COMPLAIN(stop_web_server(),
                         "Failed to stop web server during idle state");
#endif
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
  if (!client_started_ &&
      DeviceController::GetInstance().is_normally_booted() &&
      MQTTClient::Initialize()) {
    client_started_ = MQTTClient::GetInstance()->Start();
    return;
  }

  if (MQTTClient::AttemptReconnection() == ESP_FAIL) {
    ESP_LOGW(TAG, "Current Wi-Fi connection is unstable, finding next Wi-Fi");
    wifi_controller.SetState(WiFiStateType::WAITING_FOR_NEXT);
  }
}

void WiFiIdleState::LogRSSI() {
  wifi_ap_record_t ap_info;

  // Fetch the current AP information and handle potential errors
  ESP_RETURN_VOID_ON_ERROR(esp_wifi_sta_get_ap_info(&ap_info), TAG,
                           "esp_wifi_sta_get_ap_info");
  std::string ap_cc(ap_info.country.cc, 2);

  if (!MachineInfo::GetInstance().IsValidCountryCode()) {
    MachineInfo::GetInstance().SetCountryCode(ap_cc);
  }

  // Log the current Wi-Fi access point details
  ESP_LOGI(TAG,
           "Current SSID: %s, RSSI: %d, BSSID: %s, Channel: %d, Country: %s, "
           "Current Country: %s",
           ap_info.ssid, ap_info.rssi, FORMAT_MAC(ap_info.bssid),
           ap_info.primary, ap_cc.c_str(),
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
  DisplayManager::GetInstance().SetAnimation(AnimationType::IDLE_ANIMATION,
                                             true);
}

void WiFiWaitingForNextState::UpdateConnectAt() {
  uint32_t interval_ms = WIFI_RECONNECT_LOOP_INTERVAL_MS;
  if (ssid_available_) {
    interval_ms = WIFI_RECONNECT_INTERVAL_MS;
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
  WiFiManager &wifi_manager = WiFiManager::GetInstance();
  char ssid[WIFI_SSID_MAXIMUM] = {};
  char passwd[WIFI_PASSWD_MAXIMUM] = {};
  ssid_available_ = false;
  if (wifi_manager.GetWiFi(ssid, passwd) &&
      strcmp(ssid, WIFI_DEFAULT_SSID) != 0) {
    ctx->SetWiFi(ssid, passwd);
    ssid_available_ = true;
  }
  UpdateConnectAt();
}

void WiFiWaitingForNextState::Handle(WiFiStateContext *ctx,
                                     WiFiEventType event) {
  switch (event) {
    case WiFiEventType::SCANNING:
      wifi_controller.SetState(WiFiStateType::SCANNING);
      return;
    case WiFiEventType::SWITCHING:
      wifi_controller.SetState(WiFiStateType::CONNECTING);
      return;
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
    if (ssid_available_) {
      wifi_controller.SetState(WiFiStateType::CONNECTING);
    } else {
      wifi_controller.SetState(WiFiStateType::SCAN_AND_CONNECT);
    }
  }
}

void WiFiScanningState::Entering(WiFiStateContext *ctx) {
  WiFiManager &wifi_manager = WiFiManager::GetInstance();
  scan_failed_ = wifi_manager.ScanWiFi() != ESP_OK;
}

void WiFiScanningState::Handle(WiFiStateContext *ctx, WiFiEventType event) {
  if (scan_failed_) {
    ESP_LOGE(TAG, "Failed to scan WiFi");
    wifi_controller.SetState(ctx->GetLastState());
    return;
  }
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
  if (MachineInfo::GetInstance().IsInChina()) {
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

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err == ESP_OK && status >= 200 && status < 300) {
      ESP_LOGI(TAG, "Connectivity check passed: %s", config.url);
      return true;
    } else if (err == ESP_OK) {
      ESP_LOGE(TAG, "Unexpected HTTP status for %s: %d", config.url, status);
    } else {
      ESP_ERROR_COMPLAIN(err, "HTTP request failed for %s", config.url);
    }
  }

  return false;
}

void check_network_connectivity(void *arg) {
  uint8_t retry_count = 0;
  check_conn_task_running = true;
  WiFiCheckingConnState::CheckConnState state;
  network_accessible = false;
  while (retry_count++ < 2 && !network_accessible) {
    if (xTaskNotifyWait(0, 0, (uint32_t *)&state, portMAX_DELAY) != pdTRUE) {
      ESP_LOGW(TAG, "Failed to notify task: CHECKING_CONN");
      continue;
    }
    if (state == WiFiCheckingConnState::CheckConnState::EXIT) {
      ESP_LOGI(TAG, "Exiting network connectivity check task");
      break;
    }
    // Check external network access
    if (!check_connectivity()) {
      ESP_LOGE(TAG, "Failed to access connectivity check URL");
      WiFiManager::GetInstance().AddConnectivityFailure();
      network_accessible = false;
      break;
    }

    // Check if MQTT broker can be resolved
    if (!wifi_controller.ResolveAddress(MQTT_APP_HOST)) {
      ESP_LOGE(TAG, "Failed to resolve MQTT broker: %s", MQTT_APP_HOST);
      WiFiManager::GetInstance().AddDNSResolutionFailure();
      MQTTClient *mqtt = MQTTClient::GetInstance();
      if (mqtt) {
        mqtt->SetFallbackConfig();
      }
    }

#ifdef SYSLOG_HOST
    // Check if syslog server can be resolved
    if (!wifi_controller.ResolveAddress(SYSLOG_HOST)) {
      ESP_LOGE(TAG, "Failed to resolve SysLog server: %s", SYSLOG_HOST);
      WiFiManager::GetInstance().AddDNSResolutionFailure();
    }
#endif

    // All checks passed
    network_accessible = true;
    break;
  }

  check_conn_task_running = false;
  vTaskDelete(NULL);
}

#ifdef IONBRIDGE_WIFI_HOST_TEST
void set_network_accessible(bool accessible) {
  network_accessible = accessible;
}
#endif
