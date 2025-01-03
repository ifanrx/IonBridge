#include "mqtt_app.h"

#include <sys/stat.h>

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "app.h"
#include "ble.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event_base.h"
#include "esp_log.h"
#include "ionbridge.h"
#include "machine_info.h"
#include "mqtt_client.h"
#include "nvs_default.h"
#include "nvs_namespace.h"
#include "sdkconfig.h"
#include "telemetry_task.h"
#include "wifi.h"

static const char *TAG = "MQTTApp";

MQTTClient *MQTTClient::m_instance = nullptr;

#ifdef CONFIG_ENABLE_MQTT
#define MQTT_APP_URI CONFIG_MQTT_APP_URI
#define MQTT_MAX_RECONNECT_ATTEMPTS CONFIG_MQTT_MAX_RECONNECT_ATTEMPTS
#define MQTT_FALLBACK_IP CONFIG_MQTT_FALLBACK_IP
#endif

#ifdef CONFIG_ENABLE_MQTT

static uint16_t connection_start_time = 0;

void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                        int32_t event_id, void *event_data) {
  ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32,
           base, event_id);
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

  switch (event_id) {
    case MQTT_EVENT_CONNECTED: {
      ESP_LOGD(TAG, "MQTT_EVENT_CONNECTED");
      MQTTClient::GetInstance()->on_mqtt_event_connected(event);
      break;
    }
    case MQTT_EVENT_DISCONNECTED:
      ESP_LOGD(TAG, "MQTT_EVENT_DISCONNECTED");
      MQTTClient::GetInstance()->on_mqtt_event_disconnected(event);
      break;
    case MQTT_EVENT_SUBSCRIBED:
      ESP_LOGD(TAG, "MQTT_EVENT_SUBSCRIBED");
      break;
    case MQTT_EVENT_PUBLISHED:
      ESP_LOGD(TAG, "MQTT_EVENT_PUBLISHED");
      MQTTClient::GetInstance()->on_mqtt_event_published(event);
      break;
    case MQTT_EVENT_DATA:
      ESP_LOGD(TAG, "MQTT_EVENT_DATA");
      MQTTClient::GetInstance()->on_mqtt_event_data(event);
      break;
    case MQTT_EVENT_ERROR:
      ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
      break;
    case MQTT_EVENT_BEFORE_CONNECT:
      ESP_LOGD(TAG, "MQTT_EVENT_BEFORE_CONNECT");
      ble_adv_stop();
      connection_start_time = esp_timer_get_time() / 1e6;
      break;
    case MQTT_EVENT_DELETED:
      connection_start_time = 0;
      ESP_LOGW(TAG, "MQTT_EVENT_DELETED: msg_id=%d", event->msg_id);
      break;
    default:
      ESP_LOGW(TAG, "Other event id: %" PRIi32, event_id);
      break;
  }
}
#endif

MQTTClient::MQTTClient() {
  m_started = false;
  m_config = {};
  esp_err_t ret;
#ifdef CONFIG_ENABLE_MQTT
  size_t length = sizeof(m_uri);
  ret = MQTTNVSGet(m_uri, &length, NVSKey::MQTT_BROKER);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to get MQTT broker address: %s (%d)",
             esp_err_to_name(ret), ret);
    m_config.broker.address.uri = MQTT_APP_URI;
  } else {
    m_config.broker.address.uri = m_uri;
  }
  /* MQTT keepalive, default is 120 seconds
   * When configuring this value, keep in mind that the client attempts to
   * communicate with the broker at half the interval that is actually set.
   * This conservative approach allows for more attempts before the broker's
   * timeout occurs
   */
  m_config.session.keepalive = 40;
  m_config.session.disable_clean_session = false;
#ifdef CONFIG_MQTT_PROTOCOL_V_3_1
  m_config.session.protocol_ver = MQTT_PROTOCOL_V_3_1;
#elif defined(CONFIG_MQTT_PROTOCOL_V_3_1_1)
  m_config.session.protocol_ver = MQTT_PROTOCOL_V_3_1_1;
#elif defined(CONFIG_MQTT_PROTOCOL_V_5)
  m_config.session.protocol_ver = MQTT_PROTOCOL_V_5;
#endif
#endif

#ifdef CONFIG_ENABLE_MQTT_TLS
  // Load CA
  length = sizeof(m_ca_crt);
  ESP_GOTO_ON_ERROR(MQTTNVSGet(m_ca_crt, &length, NVSKey::MQTT_CA_CERT), ERROR,
                    TAG, "Failed to get CA certificate");
  m_config.broker.verification.certificate = m_ca_crt;

  // Load client cert
  length = sizeof(m_cert);
  ESP_GOTO_ON_ERROR(MQTTNVSGet(m_cert, &length, NVSKey::MQTT_CERT), ERROR, TAG,
                    "Failed to get client certificate");
  m_config.credentials.authentication.certificate = m_cert;

  // Load client key
  length = sizeof(m_key);
  ESP_GOTO_ON_ERROR(MQTTNVSGet(m_key, &length, NVSKey::MQTT_KEY), ERROR, TAG,
                    "Failed to get client key");
  m_config.credentials.authentication.key = m_key;
  return;

ERROR:
  m_config.broker.verification.certificate = nullptr;
  m_config.credentials.authentication.certificate = nullptr;
  m_config.credentials.authentication.key = nullptr;
#endif
}

// The MQTT Client needs to be used by multiple modules, such as Wi-Fi and App,
// so it requires a singleton pattern.
MQTTClient *MQTTClient::GetInstance() {
  if (m_instance == nullptr) {
    m_instance = new MQTTClient();
  }
  return m_instance;
}

void MQTTClient::SetFallbackIP() {
  const char *uri = "mqtt://" MQTT_FALLBACK_IP ":1883";
  bool stopped = false;
  if (m_started) {
    Stop(true);
    stopped = true;
  }
  GetInstance()->m_config.broker.address.uri = uri;
  GetInstance()->m_config.broker.verification.certificate = nullptr;
  GetInstance()->m_config.credentials.authentication.certificate = nullptr;
  GetInstance()->m_config.credentials.authentication.key = nullptr;
  ESP_LOGI(TAG, "Set fallback mqtt broker: %s",
           GetInstance()->m_config.broker.address.uri);
  if (stopped) {
    Start();
  }
}

void MQTTClient::ForceStart() {
  stop_permanently = false;
#ifdef CONFIG_ENABLE_MQTT
  if (m_started) {
    return;
  }
  ESP_LOGI(TAG, "Starting MQTT client");
  m_client = esp_mqtt_client_init(&m_config);
  /* The last argument may be used to pass data to the event handler, in this
   * example mqtt_event_handler */
  esp_err_t err = esp_mqtt_client_register_event(
      m_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler,
      NULL);
  ESP_ERROR_COMPLAIN(err, "esp_mqtt_client_register_event");
  err = esp_mqtt_client_start(m_client);
  ESP_ERROR_COMPLAIN(err, "esp_mqtt_client_start");
  m_started = err == ESP_OK;
  init_topics();
#else
  ESP_LOGI(TAG, "MQTT is disabled");
#endif
}

void MQTTClient::Start() {
  if (!stop_permanently) {
    ForceStart();
  }
}

// Called after Wi-Fi disconnection to execute the Stop Wi-Fi module
// implementation.
void MQTTClient::Stop(bool permanently) {
  stop_permanently = permanently;
  if (permanently) {
    ESP_LOGI(TAG, "Stopping MQTT client permanently");
  }

#ifdef CONFIG_ENABLE_MQTT
  if (!m_started) {
    return;
  }

#ifdef CONFIG_MQTT_MEMORY_DEBUG
  // Debugging memory usage before stopping the MQTT client
  uint32_t memory_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
  size_t free_memory_size = heap_caps_get_free_size(memory_caps);
  size_t largest_free_block = heap_caps_get_largest_free_block(memory_caps);
  ESP_LOGW(TAG,
           "Before stopping MQTT, %" PRIu32
           " capabilities (free: %d, largest block: %d).",
           memory_caps, static_cast<int>(free_memory_size),
           static_cast<int>(largest_free_block));
#endif

  ESP_LOGI(TAG, "Stopping MQTT client");
  m_started = false;
  m_connected = false;
  WAIT_FOR_CONDITION(m_publishing == 0, 10);
  ESP_ERROR_COMPLAIN(esp_mqtt_client_stop(m_client),
                     "Failed to stop MQTT client");
  ESP_ERROR_COMPLAIN(
      esp_mqtt_client_unregister_event(
          m_client, static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID),
          mqtt_event_handler),
      "Failed to unregister MQTT event handler");
  ESP_ERROR_COMPLAIN(esp_mqtt_client_destroy(m_client),
                     "Failed to destroy MQTT client");

#ifdef CONFIG_MQTT_MEMORY_DEBUG
  // Debugging memory usage after stopping the MQTT client
  free_memory_size = heap_caps_get_free_size(memory_caps);
  largest_free_block = heap_caps_get_largest_free_block(memory_caps);
  ESP_LOGW(TAG,
           "After stopping MQTT, %" PRIu32
           " capabilities (free: %d, largest block: %d).",
           memory_caps, static_cast<int>(free_memory_size),
           static_cast<int>(largest_free_block));
#endif

#else
  ESP_LOGI(TAG, "MQTT functionality is disabled");
#endif
}

esp_err_t MQTTClient::PublishBytes(const char *topic,
                                   const std::vector<uint8_t> &data, int qos) {
#ifdef CONFIG_ENABLE_MQTT
  if (data.empty()) {
    ESP_LOGW(TAG, "Attempted to publish empty data");
    return ESP_FAIL;
  }

  ESP_RETURN_ON_FALSE(
      qos != 0 || m_connected, ESP_ERR_INVALID_STATE, TAG,
      "Attempting to publish with QoS %d while MQTT is not connected", qos);

  m_publishing++;
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, data.data(), data.size(), ESP_LOG_DEBUG);

  int message_id = esp_mqtt_client_publish(
      m_client, topic, reinterpret_cast<const char *>(data.data()), data.size(),
      qos, 0);
  if (message_id < 0) {
    ESP_LOGE(TAG, "Failed to publish message");
    m_publishing--;
    return ESP_FAIL;
  }

  ESP_LOGD(TAG, "Message published successfully, Topic: %s, Message ID: %d",
           topic, message_id);

  if (qos == 0) {
    m_publishing--;
  }
#endif

  return ESP_OK;
}

esp_err_t MQTTClient::Publish(const std::vector<uint8_t> &data, int qos) {
  message_tx_count_++;
  return PublishBytes(m_telemetry_topic, data, qos);
}

void MQTTClient::on_mqtt_event_connected(esp_mqtt_event_handle_t event) {
  static bool is_first_attempt = true;

  esp_mqtt_client_subscribe(m_client, m_command_topic, 2);
  ESP_LOGI(TAG, "MQTT connected and subscribed to topic: %s", m_command_topic);

  ble_adv_delay_start();
  m_connected = true;
  m_low_memory = false;

  connection_count_++;
  connection_time_ += (esp_timer_get_time() / 1e6) - connection_start_time;
  connection_start_time = 0;

  TelemetryTask *task = TelemetryTask::GetInstance();
  if (task != nullptr) {
    if (is_first_attempt) {
      is_first_attempt = false;
      task->ReportMqttConnectionTime();
    }
    task->RequestShutdownBle();
  }
}

void MQTTClient::on_mqtt_event_disconnected(esp_mqtt_event_handle_t event) {
  m_connected = false;
  ble_adv_delay_start_limited_duration();

  if (m_disconnected_at != 0) {
    int64_t duration = esp_timer_get_time() - m_disconnected_at;
    if (duration > 15 * 60 * 1e6) {  // More than 15 minutes
      m_disconnected_at = esp_timer_get_time();
      m_reconnect_attempts = 0;
    }
  } else {
    m_disconnected_at = esp_timer_get_time();
  }

  m_reconnect_attempts++;
  ESP_LOGI(TAG, "MQTT reconnect attempts: %d", m_reconnect_attempts);

  if (m_reconnect_attempts >= MQTT_MAX_RECONNECT_ATTEMPTS) {
    if (!m_low_memory) {
      ESP_LOGW(TAG, "MQTT reconnect attempts exceeded, stopping MQTT");
      m_disconnected_at = 0;
      m_reconnect_attempts = 0;
      ble_adv_start();
      wifi_disconnect(true);
      return;
    }

    if (m_ble_deinit) {
      ESP_LOGE(TAG,
               "MQTT reconnect attempts exceeded and low memory, rebooting");
      esp_restart();
      return;  // Should not reach here
    }

    // Deinitialize BLE to free memory and attempt reconnection
    m_ble_deinit = (ble_deinit() == ESP_OK);
    m_reconnect_attempts = MQTT_MAX_RECONNECT_ATTEMPTS - 1;
  }
}

void MQTTClient::on_mqtt_event_published(esp_mqtt_event_handle_t event) {
  if (m_publishing != 0) {
    m_publishing--;
  }
  ESP_LOGD(TAG, "MQTT message published, msg_id: %d, publishing: %d",
           event->msg_id, m_publishing);
}

void MQTTClient::on_mqtt_event_data(esp_mqtt_event_handle_t event) {
  message_rx_count_++;
  App::GetInstance()->ProcessMQTTMessage(event->data, event->data_len);
}

void MQTTClient::init_topics() {
  const std::string &psn = MachineInfo::GetInstance().GetPSN();
  snprintf(m_telemetry_topic, sizeof(m_telemetry_topic), "device/%s/telemetry",
           psn.c_str());
  snprintf(m_command_topic, sizeof(m_command_topic), "device/%s/command",
           psn.c_str());
}
