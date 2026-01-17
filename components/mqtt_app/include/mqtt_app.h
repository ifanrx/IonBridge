#ifndef MQTT_APP_H_
#define MQTT_APP_H_

#include <cstdint>
#include <string>
#include <vector>

#include "admin.pb.h"
#include "command.pb.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "mqtt_client.h"
#include "pb.h"
#include "service.h"
#include "telemetry.pb.h"
#include "topic_parser.h"

#define CHECK_MQTT_CONNECTED()                                       \
  ({                                                                 \
    MQTTClient *mqtt = MQTTClient::GetInstance();                    \
    if (!(mqtt && mqtt->Connected())) {                              \
      ESP_LOGD(TAG, "MQTT client is not connected yet, skipping %s", \
               __FUNCTION__);                                        \
      return;                                                        \
    }                                                                \
  })

typedef struct {
  MqttTopicParser::Command::CommandInfo cmd;
  std::vector<uint8_t> data;
} MQTTMsg;

enum class MQTTMsgTaskEvent : uint32_t {
  RECEIVED = 0,
  EXIT = 1,
};

class MQTTClient {
  static MQTTClient *m_instance;  // pointer to singleton instance
  static SemaphoreHandle_t m_instance_mutex;
  static bool m_initialized;

  esp_mqtt_client_config_t m_config;
  esp_mqtt_client_handle_t m_client = nullptr;
  char m_uri[128];
  char m_key[2048];
  char m_cert[2048];
  char m_ca_crt[4096];

  bool m_started = false;
  bool m_connected = false;
  int64_t m_connected_at = 0;
  int64_t m_first_disconnected_at = 0;

  bool m_low_memory = false, m_ble_deinit = false;
  bool m_custom_broker = false;

  TaskHandle_t m_task = nullptr;

  MQTTClient();
  ~MQTTClient();
  esp_err_t PublishBytes(const char *topic, const std::vector<uint8_t> &data,
                         int qos = 0);
  void StopInternal(bool permanently = false);

  template <MqttTopicParser::Command::CommandSource ExpectedSource,
            typename CmdEnumT, typename PbRequestT,
            const pb_msgdesc_t *PbRequestFields, typename PbResponseT>
  void ProcessCommandMsg(MQTTMsg &msg);
  void ProcessEnduserMsg(MQTTMsg &msg) {
    ProcessCommandMsg<MqttTopicParser::Command::CommandSource::ENDUSER,
                      ServiceCommand, pb_CommandRequest,
                      pb_CommandRequest_fields, pb_CommandResponse>(msg);
  }
  void ProcessAdminMsg(MQTTMsg &msg) {
    ProcessCommandMsg<MqttTopicParser::Command::CommandSource::ADMIN,
                      AdminCommand, pb_AdminCommandRequest,
                      pb_AdminCommandRequest_fields, pb_AdminCommandResponse>(
        msg);
  }

 public:
  MQTTClient(MQTTClient const &) = delete;      // Delete copy constructor
  void operator=(MQTTClient const &) = delete;  // Delete assignment operator

  static bool Initialize();
  static MQTTClient *GetInstance();  // Singleton
  static void DestroyInstance();
  void MarkForDestroy();
  bool IsCustomBroker() { return m_custom_broker; }

  void TaskLoop();

  /**
   * @brief Attempts to establish or re-establish an MQTT connection
   *
   * This method handles the complete MQTT connection lifecycle:
   * 1. Initializes the MQTT client if needed
   * 2. Attempts to start the client if it's not already running
   * 3. Handles reconnection if the existing connection has failed
   * 4. Tracks consecutive failed connection attempts
   *
   * The method maintains a static counter to track consecutive failed
   * reconnection attempts. If a connection was stable before disconnection,
   * this counter is reset.
   *
   * @note This method may block while acquiring mutex locks
   *
   * @return
   *     - ESP_OK: Client connected successfully or is already connected
   *     - ESP_FAIL: Maximum reconnection attempts reached
   *     - ESP_ERR_NOT_FINISHED: Connection attempt failed, retry later
   *     - ESP_ERR_INVALID_STATE: Client initialization failed or GetInstance
   * returned null
   */
  static esp_err_t AttemptReconnection();

  bool Start();
  bool ForceStart();
  void Stop(bool permanently = false);
  bool Started() { return m_started; }
  bool Connected() { return m_started && m_connected; }

  void GetBrokerURI(char *uri, size_t &uri_len);
  uint16_t GetConnectionTime() const { return connection_time_; }
  uint16_t GetConnectionCount() const { return connection_count_; }
  uint32_t GetMessageTxCount() const { return message_tx_count_; }
  uint32_t GetMessageRxCount() const { return message_rx_count_; }
  void SetLowMemoryMark() { m_low_memory = true; }
  void SetFallbackConfig();
  uint64_t GetConnectionDuration() {
    if (m_connected_at == 0) {
      return 0;
    }
    return esp_timer_get_time() - m_connected_at;
  }

  esp_err_t Publish(std::string &topic, const std::vector<uint8_t> &data,
                    int qos = 0) {
    return PublishBytes(topic.c_str(), data, qos);
  }
  esp_err_t Publish(ServiceCommand cmd, const pb_CommandResponse &resp,
                    int qos = 0);
  esp_err_t Publish(AdminCommand cmd, const pb_AdminCommandResponse &resp,
                    int qos = 0);
  esp_err_t Publish(TelemetryCommand cmd, const pb_TelemetryData &data,
                    int qos = 0);
  esp_err_t Subscribe();

  // MQTT Event callbacks
  void on_mqtt_event_connected(esp_mqtt_event_handle_t event);
  void on_mqtt_event_disconnected(esp_mqtt_event_handle_t event);
  void on_mqtt_event_published(esp_mqtt_event_handle_t event);
  void on_mqtt_event_data(esp_mqtt_event_handle_t event);
  void on_mqtt_event_error(esp_mqtt_event_handle_t event);
  void on_mqtt_event_subscribed(esp_mqtt_event_handle_t event);

 private:
  bool stop_permanently = false;
  uint16_t connection_time_ = 0;
  uint16_t connection_count_ = 0;
  uint32_t message_tx_count_ = 0;
  uint32_t message_rx_count_ = 0;
};

#include "mqtt_app.tpp"  // IWYU pragma: keep

#endif
