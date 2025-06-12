#ifndef H_WIFI_
#define H_WIFI_

#include <stddef.h>

#include <cstdint>

#include "esp_err.h"
#include "sdkconfig.h"
#include "wifi_state.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_WIFI_CONNECT_SET_BSSID
#define WIFI_CONNECT_SET_BSSID
#endif
#define WIFI_SSID_MAXIMUM 32
#define WIFI_PASSWD_MAXIMUM 64
#define WIFI_RETRIES 6
#define WIFI_EVT_MAX_DELAY_MS_ 24000
#define WIFI_RETRY_CONNECT_DELAY_MS 10000

typedef struct {
  char ssid[WIFI_SSID_MAXIMUM];
  size_t size;
  int8_t rssi;
  uint8_t auth_mode;
#ifdef WIFI_CONNECT_SET_BSSID
  uint8_t bssid[6];
#endif
} ssid_info_t;

typedef struct {
  uint8_t bssid[6];
  uint8_t channel;
  int8_t rssi;
  // clang-format off
  uint8_t phy_11b : 1; /**< bit: 0 flag to identify if 11b mode is enabled or not */
  uint8_t phy_11g : 1; /**< bit: 1 flag to identify if 11g mode is enabled or not */
  uint8_t phy_11n : 1; /**< bit: 2 flag to identify if 11n mode is enabled or not */
  uint8_t phy_lr : 1; /**< bit: 3 flag to identify if low rate is enabled or not */
  uint8_t phy_11ax : 1; /**< bit: 4 flag to identify if 11ax mode is enabled or not */
  uint8_t wps : 1; /**< bit: 5 flag to identify if WPS is supported or not */
  uint8_t ftm_responder : 1; /**< bit: 6 flag to identify if FTM is supported in responder mode */
  uint8_t ftm_initiator : 1; /**< bit: 7 flag to identify if FTM is supported in initiator mode */
  uint8_t ssid[33];
  // clang-format on
} ssid_detail_t;

esp_err_t wifi_initialize();
esp_err_t wifi_disconnect(bool reconnect_next_wifi = false);
esp_err_t wifi_scan_ap(ssid_info_t *aps, bool pause_reconnect_timer);
esp_err_t get_connected_ssid_detail(ssid_detail_t *ssid_detail);
size_t serialize_ssid_detail(const ssid_detail_t *ssid_detail, uint8_t *buf,
                             size_t buf_size);
bool wifi_is_connected();
esp_err_t wifi_close();

void get_ipv4_addr(uint8_t *data);
#ifdef CONFIG_LWIP_IPV6
void get_ipv6_addr(uint8_t *data);
#endif

#ifdef __cplusplus
}
#endif

class WiFiController {
  WiFiState *state_;
  WiFiStateContext *ctx_;

  void SetState(WiFiState *state);

 public:
  static void StartTask();
  static void Run(void *taskParams);
  void SetState(WiFiStateType type);
  WiFiStateType GetStateType() { return state_->Type(); }
  const WiFiState *GetState() { return state_; }
  std::string GetStateString() { return state_->ToString(); }
  void Loop();
  void Notify(WiFiEventType event, bool overwrite = true);

  bool SwitchWiFi(const char *ssid, const char *passwd);
  void StartScan() { Notify(WiFiEventType::SCANNING); };
  void SetReconnect(bool skip_reconnect, bool reconnect_next_wifi) {
    ctx_->SetReconnect(skip_reconnect, reconnect_next_wifi);
  }
  void Abort() { Notify(WiFiEventType::ABORT); }

  void OnConnected(void *event_data);
  void OnDisconnected(void *event_data);

  bool IsConnectingOrConnected() {
    switch (state_->Type()) {
      case WiFiStateType::CONNECTING:
      case WiFiStateType::CONNECTED:
      case WiFiStateType::CHECKING_CONN:
      case WiFiStateType::IDLE:
        return true;
      default:
        return false;
    }
  };

  bool ResolveAddress(const char *hostname);
};

extern WiFiController wifi_controller;

#endif  // !H_WIFI_
