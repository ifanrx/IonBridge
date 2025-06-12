#ifndef H_WIFI_STATE_
#define H_WIFI_STATE_

#include <cstdint>
#include <memory>
#include <string>

#include "esp_wifi_types_generic.h"
#include "freertos/idf_additions.h"
#include "singleton.h"

#define WIFI_SSID_MAXIMUM 32

enum class WiFiEventType {
  NONE = 0,
  SCANNING,
  SCAN_DONE,
  SWITCHING,
  CONNECTED,
  DISCONNECTED,
  CHECKING_CONN,
  ABORT,
  START_WEB_SERVER,
  STOP_WEB_SERVER,
  LOST_IP,
  STOP,
};

const char *WiFiEventTypeToString(WiFiEventType event);

enum class WiFiStateType {
  CONFIGURATION = 0,
  SCAN_AND_CONNECT,
  CONNECTING,
  CONNECTED,
  CHECKING_CONN,
  IDLE,
  DISCONNECTED,
  RECONNECTING,
  WAITING_FOR_NEXT,
  SCANNING,
  CONNECTION_ABORT,
  WAITING_FOR_DEFAULT,
  COUNT,
};

class WiFiStateContext : public Singleton<WiFiStateContext> {
  WiFiStateType last_state_ = WiFiStateType::CONFIGURATION;
  bool skip_reconnect_ =
      false;  // If true, skip all Wi-Fi reconnection attempts.
  bool reconnect_next_wifi_ =
      false;  // If true, skip reconnecting to the current Wi-Fi and attempt to
              // connect to the next available network.
  bool switching_wifi_ = false;
  std::unique_ptr<wifi_config_t> wifi_config_;
  char disconnected_ssid_[WIFI_SSID_MAXIMUM];

 public:
  ~WiFiStateContext() {
    if (wifi_config_) {
      wifi_config_.reset();
    }
  }
  void Initialize();
  uint16_t connection_start_time_ = 0;

  WiFiStateType GetLastState() { return last_state_; }
  void SetLastState(WiFiStateType state) { last_state_ = state; }

  bool IsSkipReconnect() { return skip_reconnect_; }
  bool IsReconnectNextWiFi() { return reconnect_next_wifi_; }
  void SetReconnect(bool skip_reconnect, bool reconnect_next_wifi) {
    skip_reconnect_ = skip_reconnect;
    reconnect_next_wifi_ = reconnect_next_wifi;
  }
  bool IsSwitchingWiFi() { return switching_wifi_; }
  void SetSwitchingWiFi(bool switching_wifi) {
    switching_wifi_ = switching_wifi;
  }

  void SetWiFi(const char *ssid, const char *passwd);
  wifi_config_t &GetWiFiConfig() { return *wifi_config_; };
  const char *GetWiFiSSID() {
    return reinterpret_cast<const char *>(wifi_config_->sta.ssid);
  };
  void ResetWiFiSSID();
  void ResetWiFiBSSID();

  void SetDisconnectdSSID(const char *ssid);
  const char *GetDisconnectdSSID() { return disconnected_ssid_; };

  void Reset();
};

class WiFiState {
 protected:
  WiFiStateType type_;

 public:
  virtual ~WiFiState() = default;
  virtual void Handle(WiFiStateContext *ctx, WiFiEventType event) = 0;
  virtual void Entering(WiFiStateContext *ctx) {}
  virtual void Exiting(WiFiStateContext *ctx) {}

  explicit WiFiState(WiFiStateType type) : type_(type) {}
  WiFiStateType Type() const { return type_; }
  std::string ToString() const {
    switch (type_) {
      case WiFiStateType::CONFIGURATION:
        return "CONFIGURATION";
      case WiFiStateType::SCAN_AND_CONNECT:
        return "SCAN_AND_CONNECT";
      case WiFiStateType::CONNECTING:
        return "CONNECTING";
      case WiFiStateType::CONNECTED:
        return "CONNECTED";
      case WiFiStateType::CHECKING_CONN:
        return "CHECKING_CONN";
      case WiFiStateType::IDLE:
        return "IDLE";
      case WiFiStateType::RECONNECTING:
        return "RECONNECTING";
      case WiFiStateType::DISCONNECTED:
        return "DISCONNECTED";
      case WiFiStateType::SCANNING:
        return "SCANNING";
      case WiFiStateType::CONNECTION_ABORT:
        return "CONNECTION_ABORT";
      case WiFiStateType::WAITING_FOR_NEXT:
        return "WAITING_FOR_NEXT";
      case WiFiStateType::WAITING_FOR_DEFAULT:
        return "WAITING_FOR_DEFAULT";
      default:
        return "UNKNOWN";
    }
  }
};

class WiFiConfigurationState : public WiFiState,
                               public Singleton<WiFiConfigurationState> {
 public:
  void Entering(WiFiStateContext *ctx) override;
  void Handle(WiFiStateContext *ctx, WiFiEventType event) override;

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<WiFiConfigurationState>;
  // Private constructor to prevent external instantiation
  WiFiConfigurationState() : WiFiState(WiFiStateType::CONFIGURATION) {}
};

class WiFiScanAndConnectState : public WiFiState,
                                public Singleton<WiFiScanAndConnectState> {
 public:
  void Entering(WiFiStateContext *ctx) override;
  void Handle(WiFiStateContext *ctx, WiFiEventType event) override;

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<WiFiScanAndConnectState>;
  // Private constructor to prevent external instantiation
  WiFiScanAndConnectState() : WiFiState(WiFiStateType::SCAN_AND_CONNECT) {}

  bool scan_failed_ = false;
};

class WiFiConnectingState : public WiFiState,
                            public Singleton<WiFiConnectingState> {
 public:
  void Entering(WiFiStateContext *ctx) override;
  void Handle(WiFiStateContext *ctx, WiFiEventType event) override;
  void Exiting(WiFiStateContext *ctx) override;

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<WiFiConnectingState>;
  // Private constructor to prevent external instantiation
  WiFiConnectingState() : WiFiState(WiFiStateType::CONNECTING) {}
  void StartConnect(WiFiStateContext *ctx);

  bool disconnected_ = false;
  bool connecting_ = false;
  uint8_t retry_cnt_ = 0;
};

class WiFiConnectedState : public WiFiState,
                           public Singleton<WiFiConnectedState> {
 public:
  void Entering(WiFiStateContext *ctx) override;
  void Handle(WiFiStateContext *ctx, WiFiEventType event) override;

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<WiFiConnectedState>;
  // Private constructor to prevent external instantiation
  WiFiConnectedState() : WiFiState(WiFiStateType::CONNECTED) {}
};

class WiFiCheckingConnState : public WiFiState,
                              public Singleton<WiFiCheckingConnState> {
 public:
  void Entering(WiFiStateContext *ctx) override;
  void Exiting(WiFiStateContext *ctx) override;
  void Handle(WiFiStateContext *ctx, WiFiEventType event) override;

  enum class CheckConnState {
    EXIT = 0,
    CHECKING,
  };

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<WiFiCheckingConnState>;
  // Private constructor to prevent external instantiation
  WiFiCheckingConnState() : WiFiState(WiFiStateType::CHECKING_CONN) {}

  TaskHandle_t task_handle_ = nullptr;

  void StartCheckConnTask();
  void StopCheckConnTask();
  void ExecuteCheckConnTask();
};

class WiFiIdleState : public WiFiState, public Singleton<WiFiIdleState> {
 public:
  void Entering(WiFiStateContext *ctx) override;
  void Exiting(WiFiStateContext *ctx) override;
  void Handle(WiFiStateContext *ctx, WiFiEventType event) override;

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<WiFiIdleState>;
  // Private constructor to prevent external instantiation
  WiFiIdleState() : WiFiState(WiFiStateType::IDLE) {}
  void LogRSSI();

  int64_t next_log_at_ = 0;
  bool client_started_ = false;
  bool temporary_exiting_ = false;
};

class WiFiDisconnectedState : public WiFiState,
                              public Singleton<WiFiDisconnectedState> {
 public:
  void Handle(WiFiStateContext *ctx, WiFiEventType event) override;

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<WiFiDisconnectedState>;
  // Private constructor to prevent external instantiation
  WiFiDisconnectedState() : WiFiState(WiFiStateType::DISCONNECTED) {}
};

class WiFiReconnectingState : public WiFiState,
                              public Singleton<WiFiReconnectingState> {
 public:
  void Entering(WiFiStateContext *ctx) override;
  void Exiting(WiFiStateContext *ctx) override;
  void Handle(WiFiStateContext *ctx, WiFiEventType event) override;

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<WiFiReconnectingState>;
  // Private constructor to prevent external instantiation
  WiFiReconnectingState() : WiFiState(WiFiStateType::RECONNECTING) {}

  uint32_t delay_ms_ = 0;
  int retry_cnt_ = 0;
  int64_t reconnect_at_;
  bool connecting_ = false;
};

class WiFiWaitingForNextState : public WiFiState,
                                public Singleton<WiFiWaitingForNextState> {
 public:
  void Entering(WiFiStateContext *ctx) override;
  void Handle(WiFiStateContext *ctx, WiFiEventType event) override;

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<WiFiWaitingForNextState>;
  // Private constructor to prevent external instantiation
  WiFiWaitingForNextState() : WiFiState(WiFiStateType::WAITING_FOR_NEXT) {}

  void UpdateConnectAt();

  int64_t connect_at_;
  bool ssid_available_;
};

class WiFiScanningState : public WiFiState,
                          public Singleton<WiFiScanningState> {
 public:
  void Entering(WiFiStateContext *ctx) override;
  void Handle(WiFiStateContext *ctx, WiFiEventType event) override;

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<WiFiScanningState>;
  // Private constructor to prevent external instantiation
  WiFiScanningState() : WiFiState(WiFiStateType::SCANNING) {}

  bool scan_failed_ = false;
};

class WiFiConnectionAbortState : public WiFiState,
                                 public Singleton<WiFiConnectionAbortState> {
 public:
  void Entering(WiFiStateContext *ctx) override;
  void Handle(WiFiStateContext *ctx, WiFiEventType event) override;

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<WiFiConnectionAbortState>;
  // Private constructor to prevent external instantiation
  WiFiConnectionAbortState() : WiFiState(WiFiStateType::CONNECTION_ABORT) {}

  int64_t until_at_ = 0;
  bool disconnected_ = false;
};

class WiFiWaitingForDefaultState
    : public WiFiState,
      public Singleton<WiFiWaitingForDefaultState> {
 public:
  void Entering(WiFiStateContext *ctx) override;
  void Handle(WiFiStateContext *ctx, WiFiEventType event) override;

 private:
  // Allow Singleton to access the constructor
  friend class Singleton<WiFiWaitingForDefaultState>;
  // Private constructor to prevent external instantiation
  WiFiWaitingForDefaultState()
      : WiFiState(WiFiStateType::WAITING_FOR_DEFAULT) {}

  int64_t until_at_ = 0;
};

#ifdef IONBRIDGE_WIFI_HOST_TEST
void set_network_accessible(bool accessible);
#endif

#endif /* H_WIFI_STATE_ */
