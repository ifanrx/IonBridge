#include "mqtt_app.h"

#include <string.h>
#include <sys/stat.h>

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <queue>
#include <string>
#include <vector>

#include "admin.pb.h"
#include "ble.h"
#include "command.pb.h"
#include "display_animation.h"
#include "display_manager.h"
#ifndef CONFIG_ENABLE_FACTORY_MODE
#include "config_sync_manager.h"
#endif
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event_base.h"
#include "esp_log.h"
#include "esp_log_buffer.h"
#include "esp_log_level.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "mqtt_client.h"
#include "nvs_key.h"
#include "nvs_namespace.h"
#include "pb_utils.h"
#include "portmacro.h"
#include "sdkconfig.h"
#include "service.h"
#include "telemetry.pb.h"
#include "telemetry_task.h"
#include "topic_parser.h"
#include "utils.h"

using namespace MqttTopicParser;
using namespace nvs_namespace;

static const char *TAG = "MQTTApp";

typedef std::queue<MQTTMsg> MQTTMsgQueue;
static MQTTMsgQueue kMQTTMsgQueue;

MQTTClient *MQTTClient::m_instance = nullptr;
SemaphoreHandle_t MQTTClient::m_instance_mutex = nullptr;
bool MQTTClient::m_initialized = false;

#define MQTT_APP_URI CONFIG_MQTT_APP_URI
#define MQTT_MAX_RECONNECT_ATTEMPTS CONFIG_MQTT_MAX_RECONNECT_ATTEMPTS
#define MQTT_APP_FALLBACK_URI CONFIG_MQTT_APP_FALLBACK_URI
#define MQTT_APP_HOSTNAME CONFIG_MQTT_APP_HOSTNAME
#define MQTT_APP_STABLE_CONNECTION_DURATION 10 * 60 * 1e6
#define MQTT_APP_MONITOR_DURATION 15 * 60 * 1e6

static uint16_t connection_start_time = 0;
static bool client_need_destroy = false;

// Dispatching MQTT events to the MQTT client
void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                        int32_t event_id, void *event_data) {
  ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32,
           base, event_id);

  if (client_need_destroy) {
    // MQTT client is destroying, ignore all events
    return;
  }

  if (!MQTTClient::Initialize()) {
    // MQTT client initialization failed, ignore all events
    return;
  }

  MQTTClient *client = MQTTClient::GetInstance();
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
  switch (event_id) {
    case MQTT_EVENT_CONNECTED: {
      ESP_LOGD(TAG, "MQTT_EVENT_CONNECTED");
      client->on_mqtt_event_connected(event);
      break;
    }
    case MQTT_EVENT_DISCONNECTED:
      ESP_LOGD(TAG, "MQTT_EVENT_DISCONNECTED");
      client->on_mqtt_event_disconnected(event);
      break;
    case MQTT_EVENT_SUBSCRIBED:
      ESP_LOGD(TAG, "MQTT_EVENT_SUBSCRIBED");
      client->on_mqtt_event_subscribed(event);
      break;
    case MQTT_EVENT_PUBLISHED:
      ESP_LOGD(TAG, "MQTT_EVENT_PUBLISHED");
      client->on_mqtt_event_published(event);
      break;
    case MQTT_EVENT_DATA:
      ESP_LOGD(TAG, "MQTT_EVENT_DATA");
      client->on_mqtt_event_data(event);
      break;
    case MQTT_EVENT_ERROR:
      ESP_LOGD(TAG, "MQTT_EVENT_ERROR");
      client->on_mqtt_event_error(event);
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

static void RunMQTTMessageTask(void *arg) {
  MQTTClient *mqtt = static_cast<MQTTClient *>(arg);
  mqtt->TaskLoop();
}

bool MQTTClient::Initialize() {
  if (m_initialized) {
    return true;
  }

  m_instance_mutex = xSemaphoreCreateMutex();
  if (m_instance_mutex == nullptr) {
    ESP_LOGE(TAG, "Failed to create MQTTClient mutex");
    return false;
  }

  m_initialized = true;
  return true;
}

// The MQTT Client needs to be used by multiple modules, such as Wi-Fi and App,
// so it requires a singleton pattern.
MQTTClient *MQTTClient::GetInstance() {
  if (!m_initialized) {
    ESP_LOGE(TAG, "MQTTClient not initialized");
    return nullptr;
  }

  xSemaphoreTake(m_instance_mutex, portMAX_DELAY);

  if (m_instance == nullptr) {
    m_instance = new MQTTClient();
    if (m_instance == nullptr) {
      // Memory allocation failed
      ESP_LOGE(TAG, "Failed to allocate memory for MQTT client");
      return nullptr;
    }
    client_need_destroy = false;
  }

  xSemaphoreGive(m_instance_mutex);
  return m_instance;
}

void MQTTClient::DestroyInstance() {
  if (!m_initialized) {
    return;
  }

  if (xSemaphoreTake(m_instance_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    // destroying at somewhere else, abort current destroying
    return;
  }

  if (m_instance != nullptr) {
    m_instance->Stop();
    delete m_instance;
    m_instance = nullptr;
    client_need_destroy = false;
  }

  xSemaphoreGive(m_instance_mutex);
}

esp_err_t MQTTClient::AttemptReconnection() {
  static uint8_t reconnect_count = 0;
  static bool custom_broker_logged = false;

  // 1. Ensure MQTT client is initialized
  if (!m_initialized && !Initialize()) {
    ESP_LOGE(TAG, "MQTT client initialization failed");
    return ESP_ERR_INVALID_STATE;
  }

  // 2. Get current instance
  MQTTClient *mqtt = GetInstance();
  if (!mqtt) {
    return ESP_ERR_INVALID_STATE;
  }

  // 3. Skip auto-reconnect mechanism for custom brokers
  if (mqtt->IsCustomBroker()) {
    if (!custom_broker_logged) {
      ESP_LOGW(TAG,
               "Custom broker detected, skipping auto-reconnect mechanism");
      custom_broker_logged = true;
    }
    client_need_destroy = false;  // Reset flag to prevent destruction
    return ESP_OK;
  }

  // Reset custom broker log flag when using default broker
  custom_broker_logged = false;

  // 4. Early exit if client does not need to be destroyed
  if (!client_need_destroy) {
    return ESP_OK;
  }

  // 5. Handle reconnection when client needs to be destroyed
  bool was_stable =
      mqtt->GetConnectionDuration() > MQTT_APP_STABLE_CONNECTION_DURATION;
  ESP_LOGI(TAG, "Destroying MQTT client, was stable: %d", was_stable);
  DestroyInstance();
  ESP_LOGI(TAG, "MQTT client destroyed");

  if (was_stable) {
    reconnect_count = 0;
  } else {
    reconnect_count++;
    ESP_LOGI(TAG, "MQTT reconnect attempt %d of %d", reconnect_count,
             MQTT_MAX_RECONNECT_ATTEMPTS);
  }

  if (reconnect_count >= MQTT_MAX_RECONNECT_ATTEMPTS) {
    ESP_LOGE(TAG, "Maximum MQTT reconnection attempts reached");
    reconnect_count = 0;
    return ESP_FAIL;
  }

  // 6. Create a new instance and start it
  mqtt = GetInstance();
  if (!mqtt) {
    return ESP_ERR_INVALID_STATE;
  }
  if (!mqtt->Start()) {
    ESP_LOGE(TAG, "Failed to restart MQTT client");
    mqtt->MarkForDestroy();
    return ESP_ERR_NOT_FINISHED;
  }
  return ESP_OK;
}

void MQTTClient::MarkForDestroy() { client_need_destroy = true; }

MQTTClient::MQTTClient() : m_started(false), m_custom_broker(false) {
  using protected_data::MQTT;

  m_config = {};
  esp_err_t ret __attribute__((unused));
  size_t length = sizeof(m_uri);

  this->GetBrokerURI(m_uri, length);
  m_config.broker.address.uri = m_uri;

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

  // handle reconnect manually
  m_config.network.disable_auto_reconnect = true;

  if (m_custom_broker) {
    return;
  }

#ifdef CONFIG_ENABLE_MQTT_TLS
  // Load CA
  length = sizeof(m_ca_crt);
  ESP_GOTO_ON_ERROR(MQTT.Get(nvs_keys::MQTT_CA_CERT, m_ca_crt, length), ERROR,
                    TAG, "Failed to get CA certificate");
  m_config.broker.verification.certificate = m_ca_crt;

  // Load client cert
  length = sizeof(m_cert);
  ESP_GOTO_ON_ERROR(MQTT.Get(nvs_keys::MQTT_CERT, m_cert, length), ERROR, TAG,
                    "Failed to get client certificate");
  m_config.credentials.authentication.certificate = m_cert;

  // Load client key
  length = sizeof(m_key);
  ESP_GOTO_ON_ERROR(MQTT.Get(nvs_keys::MQTT_KEY, m_key, length), ERROR, TAG,
                    "Failed to get client key");
  m_config.credentials.authentication.key = m_key;
  return;

ERROR:
  m_config.broker.verification.certificate = nullptr;
  m_config.credentials.authentication.certificate = nullptr;
  m_config.credentials.authentication.key = nullptr;
#endif
}

MQTTClient::~MQTTClient() {
  if (m_client != nullptr) {
    esp_mqtt_client_destroy(m_client);
    m_client = nullptr;
  }
  if (m_task != nullptr) {
    // exit task safely
    xTaskNotify(m_task, static_cast<uint32_t>(MQTTMsgTaskEvent::EXIT),
                eSetValueWithOverwrite);
    // Wait briefly to allow task to start cleanup
    DELAY_MS(10);
    m_task = nullptr;
  }
}

void MQTTClient::GetBrokerURI(char *uri, size_t &length) {
  bool broker_found = false;

  size_t uri_length = length;
  esp_err_t custom_ret =
      user_data::MQTT_CUSTOM.Get(nvs_keys::MQTT_CUSTOM_BROKER, uri, uri_length);

  if (custom_ret == ESP_OK && strlen(uri) > 0) {
    m_custom_broker = true;
    broker_found = true;
  } else {
    // Try protected_data
    uri_length = length;
    esp_err_t prot_ret =
        protected_data::MQTT.Get(nvs_keys::MQTT_BROKER, uri, uri_length);

    if (prot_ret == ESP_OK && strlen(uri) > 0) {
      m_custom_broker = false;
      broker_found = true;
    }
  }

  if (!broker_found) {
    ESP_LOGW(TAG, "No broker found, using default: %s", MQTT_APP_URI);
    std::strncpy(uri, MQTT_APP_URI, length - 1);
    uri[length - 1] = '\0';  // Ensure null-termination
    m_custom_broker = false;
  }

  length = strlen(uri);
}

bool MQTTClient::ForceStart() {
  stop_permanently = false;
  if (m_started) {
    return true;
  }

  ESP_LOGI(TAG, "Starting MQTT client, Broker: %s",
           m_config.broker.address.uri);

  // resolve address
  if (std::strcmp(m_config.broker.address.uri, MQTT_APP_FALLBACK_URI) != 0 &&
      !resolve_hostname(MQTT_APP_HOSTNAME)) {
    ESP_LOGE(TAG, "Failed to resolve MQTT broker: %s",
             m_config.broker.address.uri);
    SetFallbackConfig();
  }

  // start MQTT message task
  if (xTaskCreate(RunMQTTMessageTask, "mqtt_msg", 1024 * 4, (void *)this, 5,
                  &m_task) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create MQTT task");
    return false;
  }

  m_client = esp_mqtt_client_init(&m_config);
  if (!m_client) {
    ESP_LOGE(TAG, "Failed to initialize MQTT client");
    return false;
  }

  /* The last argument may be used to pass data to the event handler, in
   * this example mqtt_event_handler */
  esp_err_t err = esp_mqtt_client_register_event(
      m_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler,
      NULL);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register MQTT event handler: %s",
             esp_err_to_name(err));
    esp_mqtt_client_destroy(m_client);
    m_client = nullptr;
    return false;
  }

  err = esp_mqtt_client_start(m_client);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
    esp_mqtt_client_destroy(m_client);
    m_client = nullptr;
    return false;
  }

  m_started = true;
  return m_started;
}

bool MQTTClient::Start() {
  if (!stop_permanently) {
    return ForceStart();
  }
  return false;
}

void MQTTClient::Stop(bool permanently) {
  esp_err_t ret __attribute__((unused)) = ESP_OK;
  stop_permanently = permanently;
  if (permanently) {
    ESP_LOGI(TAG, "Stopping MQTT client permanently");
    stop_permanently = true;
  }

  if (!(m_started || m_client != nullptr)) {
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

  m_started = false;
  m_connected = false;

  // Note: esp_mqtt_client_stop() handles graceful disconnection internally,
  // including sending MQTT DISCONNECT packet and flushing pending messages.
  // Calling disconnect() first would interfere with this graceful shutdown.
  ESP_LOGD(TAG, "Stopping MQTT client");
  ESP_RETURN_VOID_ON_ERROR(esp_mqtt_client_stop(m_client), TAG,
                           "Failed to stop MQTT client");
  ESP_LOGI(TAG, "Stopped MQTT client");

  ESP_RETURN_VOID_ON_ERROR(
      esp_mqtt_client_unregister_event(
          m_client, static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID),
          mqtt_event_handler),
      TAG, "Failed to unregister MQTT event handler");
  ESP_LOGD(TAG, "Destroying MQTT client");
  ESP_RETURN_VOID_ON_ERROR(
      esp_mqtt_client_destroy(m_client), TAG,
      "Failed to destroy MQTT client, maybe wrong initialization");
  ESP_LOGI(TAG, "Destroyed MQTT client");
  m_client = nullptr;

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
}

void MQTTClient::SetFallbackConfig() {
  if (m_custom_broker) {
    return;
  }
  m_config.broker.address.uri = MQTT_APP_FALLBACK_URI;
  m_config.broker.verification.common_name = MQTT_APP_HOSTNAME;

  if (!std::string(MQTT_APP_FALLBACK_URI).starts_with("mqtts://")) {
    m_config.broker.verification.certificate = nullptr;
    m_config.credentials.authentication.certificate = nullptr;
    m_config.credentials.authentication.key = nullptr;
  }
}

esp_err_t MQTTClient::PublishBytes(const char *topic,
                                   const std::vector<uint8_t> &data, int qos) {
  if (data.empty()) {
    ESP_LOGW(TAG, "Attempted to publish empty data");
    return ESP_FAIL;
  }

  message_tx_count_++;

  if (qos == 0 && !m_connected) {
    // Drop the message if QoS is 0 and MQTT is not connected
    return ESP_OK;
  }

  ESP_LOG_BUFFER_HEX_LEVEL(TAG, data.data(), data.size(), ESP_LOG_DEBUG);

  int message_id = esp_mqtt_client_publish(
      m_client, topic, reinterpret_cast<const char *>(data.data()), data.size(),
      qos, 0);
  if (message_id < 0) {
    ESP_LOGE(TAG, "Failed to publish message");
    return ESP_FAIL;
  }

  ESP_LOGD(TAG, "Message published successfully, Topic: %s, Message ID: %d",
           topic, message_id);

  return ESP_OK;
}

esp_err_t MQTTClient::Publish(ServiceCommand cmd,
                              const pb_CommandResponse &resp, int qos) {
  std::string topic = Topic::GetEnduserRespTopic(cmd);
  std::vector<uint8_t> data = serialize_pb(pb_CommandResponse_fields, &resp);
  return Publish(topic, data, qos);
}

esp_err_t MQTTClient::Publish(TelemetryCommand cmd,
                              const pb_TelemetryData &data, int qos) {
  std::string topic = Topic::GetTelemetryTopic(cmd);
  std::vector<uint8_t> payload = serialize_pb(pb_TelemetryData_fields, &data);
  return Publish(topic, payload, qos);
}

esp_err_t MQTTClient::Publish(AdminCommand cmd,
                              const pb_AdminCommandResponse &data, int qos) {
  std::string topic = Topic::GetAdminRespTopic(cmd);
  std::vector<uint8_t> payload =
      serialize_pb(pb_AdminCommandResponse_fields, &data);
  return Publish(topic, payload, qos);
}

esp_err_t MQTTClient::Subscribe() {
  if (!Connected()) {
    ESP_LOGE(TAG, "Failed to subscribe, MQTT client is not connected");
    return ESP_ERR_INVALID_STATE;
  }

  std::string topic = Topic::GetSubscribeTopic();
  int ret = esp_mqtt_client_subscribe(m_client, topic.c_str(), 2);
  if (ret < 0) {
    ESP_LOGE(TAG, "Failed to subscribe to topic: %s, ret: %d", topic.c_str(),
             ret);
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "Subscribing topic: %s", topic.c_str());
  return ESP_OK;
}

void MQTTClient::on_mqtt_event_connected(esp_mqtt_event_handle_t event) {
  std::string topic = Topic::GetSubscribeTopic();
  esp_mqtt_client_subscribe(m_client, topic.c_str(), 2);
  ESP_LOGI(TAG, "MQTT connected and subscribing topic: %s", topic.c_str());

  m_connected = true;
  m_low_memory = false;
  m_connected_at = esp_timer_get_time();

  connection_count_++;
  connection_time_ += (esp_timer_get_time() / 1e6) - connection_start_time;
  connection_start_time = 0;

  // Let animation layer decide what to show based on device status
  DisplayManager::GetInstance().SetNextAnimation(AnimationType::IDLE, false);

  if (m_custom_broker) {
    ESP_LOGI(TAG, "MQTT connected to custom broker");
    // Web server is already started when custom broker is detected in
    // WiFiIdleState
  }

  // Check for pending config syncs
#ifndef CONFIG_ENABLE_FACTORY_MODE
  ConfigSyncManager &sync_manager = ConfigSyncManager::GetInstance();
  sync_manager.CheckPendingSync();
#else
  ESP_LOGD(TAG, "Factory mode: skipping config sync after MQTT connect");
#endif
}

void MQTTClient::on_mqtt_event_disconnected(esp_mqtt_event_handle_t event) {
  m_connected = false;
  m_connected_at = 0;

  if (m_custom_broker) {
    ESP_LOGI(
        TAG,
        "Custom broker disconnected, keeping client alive for reconfiguration");
    return;
  }

  if (!m_low_memory) {
    ESP_LOGI(TAG, "MQTT disconnected, destroying client");
    MarkForDestroy();
    return;
  }

  ESP_LOGE(TAG, "MQTT reconnect attempts exceeded and low memory, rebooting");
  DELAY_MS(200);  // ensure log is printed
  esp_restart();
  return;  // Should not reach here
}

void MQTTClient::on_mqtt_event_published(esp_mqtt_event_handle_t event) {
  ESP_LOGD(TAG, "MQTT message published, msg_id: %d, qos: %d", event->msg_id,
           event->qos);
}

void MQTTClient::on_mqtt_event_data(esp_mqtt_event_handle_t event) {
  message_rx_count_++;

  ESP_LOGD(TAG, "MQTT message received, topic: %s", event->topic);
  if (event->data_len == 0) {
    ESP_LOGW(TAG, "MQTT message data length is zero");
    return;
  }
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, event->data, event->data_len, ESP_LOG_DEBUG);
  if (m_task == nullptr) {
    ESP_LOGE(TAG, "MQTT task is not initialized");
    return;
  }

  auto cmd_opt =
      Command::ParseRequest(std::string(event->topic, event->topic_len));
  if (!cmd_opt) {
    ESP_LOGE(TAG, "Failed to parse MQTT topic: %s, len: %d", event->topic,
             event->topic_len);
    return;
  }

  MQTTMsg msg;
  msg.cmd = *cmd_opt;
  msg.data = std::vector<uint8_t>(event->data, event->data + event->data_len);
  kMQTTMsgQueue.push(msg);
  xTaskNotify(m_task, static_cast<uint32_t>(MQTTMsgTaskEvent::RECEIVED),
              eSetValueWithOverwrite);
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

void MQTTClient::on_mqtt_event_subscribed(esp_mqtt_event_handle_t event) {
  static bool at_first_subscribed = true;
  std::string topic = Topic::GetSubscribeTopic();
  if (event->error_handle->error_type == MQTT_ERROR_TYPE_SUBSCRIBE_FAILED) {
    ESP_LOGE(TAG, "Failed to subscribe to topic: %s", topic.c_str());
    esp_mqtt_client_subscribe(m_client, topic.c_str(), 2);
    return;
  }

  ESP_LOGI(TAG, "Subscribed to topic: %s", topic.c_str());
  TelemetryTask &task = TelemetryTask::GetInstance();
  if (at_first_subscribed) {
    at_first_subscribed = false;
    task.ReportMQTTConnectionTime();
  }

  // Skip BLE management for custom brokers
  if (!m_custom_broker) {
    ESP_LOGI(TAG, "Default broker - managing BLE lifecycle");
    ble_adv_delay_start();
    task.RequestShutdownBle();
  } else {
    ESP_LOGI(TAG, "Custom broker - skipping BLE management");
  }
}

void MQTTClient::TaskLoop() {
  ESP_LOGI(TAG, "MQTT message task started");
  MQTTMsgTaskEvent event;
  MQTTMsg msg;
  while (true) {
    if (xTaskNotifyWait(pdTRUE, pdTRUE, (uint32_t *)&event, portMAX_DELAY) !=
        pdTRUE) {  // Wait for event
      ESP_LOGW(TAG, "Failed to wait for MQTT message task event");
      continue;
    }
    if (event == MQTTMsgTaskEvent::EXIT) {
      ESP_LOGI(TAG, "MQTT message task exiting");
      break;
    }
    if (event != MQTTMsgTaskEvent::RECEIVED) {
      ESP_LOGW(TAG, "Invalid MQTT message task event: %d", (int)event);
      continue;
    }
    while (!kMQTTMsgQueue.empty()) {
      msg = kMQTTMsgQueue.front();
      kMQTTMsgQueue.pop();

      ESP_LOG_BUFFER_HEX_LEVEL(TAG, msg.data.data(), msg.data.size(),
                               ESP_LOG_DEBUG);
      ESP_LOGD(TAG, "MQTT Message received, command: %s(0x%02X)",
               msg.cmd.ToString().c_str(), msg.cmd.id);

      switch (msg.cmd.source) {
        case Command::CommandSource::ENDUSER: {
          this->ProcessEnduserMsg(msg);
        } break;
        case Command::CommandSource::ADMIN: {
          this->ProcessAdminMsg(msg);
        } break;
        default:
          ESP_LOGE(TAG, "Unknown source: %d", (int)msg.cmd.source);
          break;
      }
      DELAY_MS(1);
    }
  }

  while (!kMQTTMsgQueue.empty()) {
    kMQTTMsgQueue.pop();
    DELAY_MS(1);
  }
  ESP_LOGI(TAG, "MQTT message task exited");
  vTaskDelete(NULL);
}
