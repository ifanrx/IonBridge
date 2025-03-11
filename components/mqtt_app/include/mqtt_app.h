#ifndef MQTT_APP_H_
#define MQTT_APP_H_

#include <cstdint>
#include <vector>

#include "esp_err.h"
#include "mqtt_client.h"

#ifdef __cplusplus
extern "C" {
#endif

class MQTTClient {
  static MQTTClient *m_instance;
  esp_mqtt_client_config_t m_config;
  esp_mqtt_client_handle_t m_client;
  char m_uri[128];
  char m_key[2048];
  char m_cert[2048];
  char m_ca_crt[4096];

  char m_telemetry_topic[40];
  char m_command_topic[40];

  bool m_started = false;
  bool m_connected = false;
  bool m_need_destroy = false;
  uint8_t m_reconnect_attempts = 0;
  int64_t m_connected_at = 0;
  int64_t m_first_disconnected_at = 0;

  bool m_low_memory = false, m_ble_deinit = false;
  uint8_t m_publishing = 0;
  bool m_custom_broker = false;

  void init_topics();
  esp_err_t PublishBytes(const char *topic, const std::vector<uint8_t> &data,
                         int qos = 0);

 public:
  MQTTClient(MQTTClient const &) = delete;
  void operator=(MQTTClient const &) = delete;

  static MQTTClient *GetInstance();
  static void DestroyInstance() {
    if (m_instance != nullptr) {
      m_instance->Stop();
      delete m_instance;
      m_instance = nullptr;
    }
  }
  bool NeedDestroy() { return m_need_destroy; }

  MQTTClient();
  void Start();
  void ForceStart();
  void Stop(bool permanently = false);
  bool Started() { return m_started; }
  bool Connected() { return m_started && m_connected; }
  bool Published() { return m_publishing == 0; }

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

  esp_err_t Publish(const std::vector<uint8_t> &data, int qos = 0);

  // MQTT Event callbacks
  void on_mqtt_event_connected(esp_mqtt_event_handle_t event);
  void on_mqtt_event_disconnected(esp_mqtt_event_handle_t event);
  void on_mqtt_event_published(esp_mqtt_event_handle_t event);
  void on_mqtt_event_data(esp_mqtt_event_handle_t event);
  void on_mqtt_event_error(esp_mqtt_event_handle_t event);

 private:
  bool stop_permanently = false;
  uint16_t connection_time_ = 0;
  uint16_t connection_count_ = 0;
  uint32_t message_tx_count_ = 0;
  uint32_t message_rx_count_ = 0;
};

#ifdef __cplusplus
}
#endif

#endif
