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
#include "esp_timer.h"
#include "ionbridge.h"
#include "machine_info.h"
#include "mqtt_client.h"
#include "nvs_default.h"
#include "nvs_namespace.h"
#include "sdkconfig.h"
#include "telemetry_task.h"
#include "utils.h"
#include "wifi.h"
#include "wifi_state.h"

static const char *TAG = "MQTTApp";

MQTTClient *MQTTClient::m_instance = nullptr;

#ifdef CONFIG_ENABLE_MQTT
#define MQTT_APP_URI CONFIG_MQTT_APP_URI
#define MQTT_MAX_RECONNECT_ATTEMPTS CONFIG_MQTT_MAX_RECONNECT_ATTEMPTS
#define MQTT_APP_FALLBACK_URI CONFIG_MQTT_APP_FALLBACK_URI
#define MQTT_APP_HOSTNAME CONFIG_MQTT_APP_HOSTNAME
#define MQTT_APP_STABLE_CONNECTION_DURATION 10 * 60 * 1e6
#define MQTT_APP_MONITOR_DURATION 15 * 60 * 1e6
#endif

#ifdef CONFIG_ENABLE_MQTT

static uint16_t connection_start_time = 0;

// 这个是 MQTT 的事件处理函数，在这里分发事件到 App 来处理
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
      ESP_LOGD(TAG, "MQTT_EVENT_ERROR");
      MQTTClient::GetInstance()->on_mqtt_event_error(event);
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
  ret = UserMQTTNVSGet(m_uri, &length, NVSKey::MQTT_CUSTOM_BROKER);
  if (ret == ESP_OK && strlen(m_uri) > 0) {
    // use custom broker
    m_config.broker.address.uri = m_uri;
    m_custom_broker = true;
  } else {
    ret = MQTTNVSGet(m_uri, &length, NVSKey::MQTT_BROKER);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Failed to get MQTT broker address: %s (%d)",
               esp_err_to_name(ret), ret);
      m_config.broker.address.uri = MQTT_APP_URI;
    } else {
      m_config.broker.address.uri = m_uri;
    }
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

  // handle reconnect manually
  m_config.network.disable_auto_reconnect = true;

#ifdef CONFIG_ENABLE_MQTT_TLS
  if (m_custom_broker) {
    return;
  }
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

void MQTTClient::ForceStart() {
  stop_permanently = false;
#ifdef CONFIG_ENABLE_MQTT
  if (m_started) {
    return;
  }

  ESP_LOGI(TAG, "Starting MQTT client, Broker: %s",
           m_config.broker.address.uri);
  m_client = esp_mqtt_client_init(&m_config);

  /* The last argument may be used to pass data to the event handler, in
   * this example mqtt_event_handler */
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
  __attribute__((unused)) esp_err_t ret = ESP_OK;
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
  ESP_LOGI(TAG, "Stoppped MQTT client");
  ESP_ERROR_COMPLAIN(
      esp_mqtt_client_unregister_event(
          m_client, static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID),
          mqtt_event_handler),
      "Failed to unregister MQTT event handler");
  ESP_ERROR_COMPLAIN(esp_mqtt_client_destroy(m_client),
                     "Failed to destroy MQTT client");
  ESP_LOGI(TAG, "Destroyed MQTT client");

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

void MQTTClient::SetFallbackConfig() {
#ifdef CONFIG_ENABLE_MQTT
  if (m_custom_broker) {
    return;
  }
  m_config.broker.address.uri = MQTT_APP_FALLBACK_URI;
  m_config.broker.verification.common_name = MQTT_APP_HOSTNAME;
#endif
}

esp_err_t MQTTClient::PublishBytes(const char *topic,
                                   const std::vector<uint8_t> &data, int qos) {
#ifdef CONFIG_ENABLE_MQTT
  if (data.empty()) {
    ESP_LOGW(TAG, "Attempted to publish empty data");
    return ESP_FAIL;
  }

  if (qos == 0 && !m_connected) {
    // Drop the message if QoS is 0 and MQTT is not connected
    return ESP_OK;
  }

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

  wifi_controller.Notify(WiFiEventType::MQTT_CONNECTED);
  m_connected = true;
  m_low_memory = false;
  m_connected_at = esp_timer_get_time();

  connection_count_++;
  connection_time_ += (esp_timer_get_time() / 1e6) - connection_start_time;
  connection_start_time = 0;

  TelemetryTask *task = TelemetryTask::GetInstance();
  if (task != nullptr && !m_custom_broker) {
    if (is_first_attempt) {
      is_first_attempt = false;
      task->ReportMqttConnectionTime();
    }
    ble_adv_delay_start();
    task->RequestShutdownBle();
  }
  if (m_custom_broker) {
    DELAY_MS(100);
    ESP_LOGI(TAG, "MQTT connected to custom broker, starting web server");
    wifi_controller.Notify(WiFiEventType::START_WEB_SERVER);
  }
}

void MQTTClient::on_mqtt_event_disconnected(esp_mqtt_event_handle_t event) {
  m_connected = false;
  ble_adv_delay_start_limited_duration();
  m_connected_at = 0;
  wifi_controller.Notify(WiFiEventType::MQTT_DISCONNECTED);

  if (!m_low_memory) {
    ESP_LOGI(TAG, "MQTT disconnected, destroying client");
    m_need_destroy = true;
    return;
  }

  ESP_LOGE(TAG, "MQTT reconnect attempts exceeded and low memory, rebooting");
  DELAY_MS(200);  // ensure log is printed
  esp_restart();
  return;  // Should not reach here
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
  App::GetInstance().ProcessMQTTMessage(event->data, event->data_len);
}

void MQTTClient::on_mqtt_event_error(esp_mqtt_event_handle_t event) {
  esp_mqtt_error_codes_t *error_code = event->error_handle;
  ESP_LOGW(TAG, "last esp_err code reported from esp-tls component: 0x%04X",
           error_code->esp_tls_last_esp_err);
  ESP_LOGW(TAG, "tls specific error code reported from underlying tls tack: %d",
           error_code->esp_tls_stack_err);
  ESP_LOGW(TAG,
           "tls flags reported from underlying tls stack during certificate "
           "verification: %d",
           error_code->esp_tls_cert_verify_flags);
  switch (error_code->error_type) {
    case MQTT_ERROR_TYPE_NONE:
      ESP_LOGE(TAG, "MQTT_ERROR_TYPE_NONE");
      break;
    case MQTT_ERROR_TYPE_TCP_TRANSPORT:
      ESP_LOGE(TAG, "MQTT_ERROR_TYPE_TCP_TRANSPORT");
      break;
    case MQTT_ERROR_TYPE_CONNECTION_REFUSED:
      ESP_LOGE(TAG, "MQTT_ERROR_TYPE_CONNECTION_REFUSED");
      break;
    case MQTT_ERROR_TYPE_SUBSCRIBE_FAILED:
      ESP_LOGE(TAG, "MQTT_ERROR_TYPE_SUBSCRIBE_FAILED");
      break;
    default:
      ESP_LOGE(TAG, "MQTT_EVENT_ERROR: %d", event->error_handle->error_type);
      break;
  }
  switch (error_code->connect_return_code) {
    case MQTT_CONNECTION_ACCEPTED:
      ESP_LOGW(TAG, "Connection accepted");
      break;
    case MQTT_CONNECTION_REFUSE_PROTOCOL:
      ESP_LOGW(TAG, "Wrong protocol");
      break;
    case MQTT_CONNECTION_REFUSE_ID_REJECTED:
      ESP_LOGW(TAG, "ID rejected");
      break;
    case MQTT_CONNECTION_REFUSE_SERVER_UNAVAILABLE:
      ESP_LOGW(TAG, "Server unavailable");
      break;
    case MQTT_CONNECTION_REFUSE_BAD_USERNAME:
      ESP_LOGW(TAG, "Wrong user");
      break;
    case MQTT_CONNECTION_REFUSE_NOT_AUTHORIZED:
      ESP_LOGW(TAG, "Wrong username or password");
      break;
  }
  ESP_LOGW(TAG, "errno from the underlying socket: %d",
           error_code->esp_transport_sock_errno);
}

void MQTTClient::init_topics() {
  const std::string &psn = MachineInfo::GetInstance().GetPSN();
  snprintf(m_telemetry_topic, sizeof(m_telemetry_topic), "device/%s/telemetry",
           psn.c_str());
  snprintf(m_command_topic, sizeof(m_command_topic), "device/%s/command",
           psn.c_str());
}
