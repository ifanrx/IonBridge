#include "mqtt_app.h"

#include <sys/stat.h>

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <queue>
#include <string>
#include <vector>

#include "app.h"
#include "ble.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event_base.h"
#include "esp_log.h"
#include "esp_log_buffer.h"
#include "esp_log_level.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "ionbridge.h"
#include "machine_info.h"
#include "mqtt_client.h"
#include "mqtt_message.h"
#include "nvs_default.h"
#include "nvs_namespace.h"
#include "portmacro.h"
#include "sdkconfig.h"
#include "telemetry_task.h"
#include "utils.h"
#include "wifi.h"
#include "wifi_state.h"

static const char *TAG = "MQTTApp";

typedef std::queue<std::vector<uint8_t>> MQTTMsgQueue;
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

  // 3. If client doesn't need to be destroyed, everything is fine
  if (!client_need_destroy) {
    return ESP_OK;
  }

  // 4. Handle reconnection when client needs to be destroyed
  bool was_stable =
      mqtt->GetConnectionDuration() > MQTT_APP_STABLE_CONNECTION_DURATION;
  DestroyInstance();

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

  // 5. Create a new instance and start it
  MQTTClient *new_mqtt = GetInstance();
  if (!new_mqtt) {
    return ESP_ERR_INVALID_STATE;
  }
  if (!new_mqtt->Start()) {
    ESP_LOGE(TAG, "Failed to restart MQTT client");
    mqtt->MarkForDestroy();
    return ESP_ERR_NOT_FINISHED;
  }
  return ESP_OK;
}

void MQTTClient::MarkForDestroy() { client_need_destroy = true; }

MQTTClient::MQTTClient() : m_started(false), m_custom_broker(false) {
  m_config = {};
  esp_err_t ret;

  init_topics();

  size_t length = sizeof(m_uri);
  ret = UserMQTTNVSGet(m_uri, &length, NVSKey::MQTT_CUSTOM_BROKER);
  if (ret == ESP_OK && strlen(m_uri) > 0) {
    // use custom broker
    m_custom_broker = true;
  } else if (MQTTNVSGet(m_uri, &length, NVSKey::MQTT_BROKER) != ESP_OK) {
    // use default broker
    ESP_LOGW(TAG, "Failed to get MQTT broker address: %s (%d)",
             esp_err_to_name(ret), ret);
    // Safer copy with size check
    int written = snprintf(m_uri, sizeof(m_uri), "%s", MQTT_APP_URI);
    if (written >= sizeof(m_uri)) {
      ESP_LOGE(TAG, "MQTT URI truncated (needed %d bytes, have %d)",
               written + 1, (int)sizeof(m_uri));
    }
  }
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

bool MQTTClient::ForceStart() {
  stop_permanently = false;
  if (m_started) {
    return true;
  }

  ESP_LOGI(TAG, "Starting MQTT client, Broker: %s",
           m_config.broker.address.uri);

  // resolve address
  if (std::strcmp(m_config.broker.address.uri, MQTT_APP_FALLBACK_URI) != 0 &&
      !wifi_controller.ResolveAddress(MQTT_APP_HOSTNAME)) {
    ESP_LOGE(TAG, "Failed to resolve MQTT broker: %s",
             m_config.broker.address.uri);
    SetFallbackConfig();
  }

  // start MQTT message task
  if (xTaskCreate(RunMQTTMessageTask, "mqtt_msg", 1024 * 3, (void *)this, 5,
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
  WAIT_FOR_CONDITION(m_publishing == 0, 10);

  ESP_RETURN_VOID_ON_ERROR(
      esp_mqtt_client_disconnect(m_client), TAG,
      "Couldn't disconnect MQTT client, maybe wrong initialization");

  ESP_LOGI(TAG, "Stopping MQTT client");
  ESP_RETURN_VOID_ON_ERROR(esp_mqtt_client_stop(m_client), TAG,
                           "Failed to stop MQTT client");  // 5 secs for stop
  ESP_LOGI(TAG, "Stopped MQTT client");
  ESP_RETURN_VOID_ON_ERROR(
      esp_mqtt_client_unregister_event(
          m_client, static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID),
          mqtt_event_handler),
      TAG, "Failed to unregister MQTT event handler");
  ESP_LOGI(TAG, "Destroying MQTT client");
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

  if (qos == 0 && !m_connected) {
    // Drop the message if QoS is 0 and MQTT is not connected
    return ESP_OK;
  }

  m_publishing = m_publishing + 1;
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, data.data(), data.size(), ESP_LOG_DEBUG);

  int message_id = esp_mqtt_client_publish(
      m_client, topic, reinterpret_cast<const char *>(data.data()), data.size(),
      qos, 0);
  if (message_id < 0) {
    ESP_LOGE(TAG, "Failed to publish message");
    m_publishing = m_publishing - 1;
    return ESP_FAIL;
  }

  ESP_LOGD(TAG, "Message published successfully, Topic: %s, Message ID: %d",
           topic, message_id);

  if (qos == 0) {
    m_publishing = m_publishing - 1;
  }

  return ESP_OK;
}

esp_err_t MQTTClient::Publish(const std::vector<uint8_t> &data, int qos) {
  message_tx_count_++;
  return PublishBytes(m_telemetry_topic, data, qos);
}

esp_err_t MQTTClient::Subscribe() {
  if (!Connected()) {
    ESP_LOGE(TAG, "Failed to subscribe, MQTT client is not connected");
    return ESP_ERR_INVALID_STATE;
  }
  int ret = esp_mqtt_client_subscribe(m_client, m_command_topic, 2);
  if (ret < 0) {
    ESP_LOGE(TAG, "Failed to subscribe to topic: %s, ret: %d", m_command_topic,
             ret);
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "Subscribing topic: %s", m_command_topic);
  return ESP_OK;
}

void MQTTClient::on_mqtt_event_connected(esp_mqtt_event_handle_t event) {
  esp_mqtt_client_subscribe(m_client, m_command_topic, 2);
  ESP_LOGI(TAG, "MQTT connected and subscribing topic: %s", m_command_topic);

  m_connected = true;
  m_low_memory = false;
  m_connected_at = esp_timer_get_time();

  connection_count_++;
  connection_time_ += (esp_timer_get_time() / 1e6) - connection_start_time;
  connection_start_time = 0;

  if (m_custom_broker) {
    DELAY_MS(100);
    ESP_LOGI(TAG, "MQTT connected to custom broker, starting web server");
    wifi_controller.Notify(WiFiEventType::START_WEB_SERVER);
  }
}

void MQTTClient::on_mqtt_event_disconnected(esp_mqtt_event_handle_t event) {
  m_connected = false;
  m_connected_at = 0;

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
  if (m_publishing != 0) {
    m_publishing = m_publishing - 1;
  }
  ESP_LOGD(TAG, "MQTT message published, msg_id: %d, publishing: %d",
           event->msg_id, m_publishing);
}

void MQTTClient::on_mqtt_event_data(esp_mqtt_event_handle_t event) {
  message_rx_count_++;

  if (event->data_len == 0) {
    ESP_LOGW(TAG, "MQTT message data length is zero");
    return;
  }
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, event->data, event->data_len, ESP_LOG_DEBUG);
  if (m_task == nullptr) {
    ESP_LOGE(TAG, "MQTT task is not initialized");
    return;
  }

  kMQTTMsgQueue.push(
      std::vector<uint8_t>(event->data, event->data + event->data_len));
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
  if (event->error_handle->error_type == MQTT_ERROR_TYPE_SUBSCRIBE_FAILED) {
    ESP_LOGE(TAG, "Failed to subscribe to topic: %s", m_command_topic);
    esp_mqtt_client_subscribe(m_client, m_command_topic, 2);
    return;
  }
  ESP_LOGI(TAG, "Subscribed to topic: %s", m_command_topic);
  TelemetryTask &task = TelemetryTask::GetInstance();
  if (at_first_subscribed) {
    at_first_subscribed = false;
    task.ReportMqttConnectionTime();
  }
  ble_adv_delay_start();
  task.RequestShutdownBle();
}

void MQTTClient::init_topics() {
  const std::string &psn = MachineInfo::GetInstance().GetPSN();
  snprintf(m_telemetry_topic, sizeof(m_telemetry_topic), "device/%s/telemetry",
           psn.c_str());
  snprintf(m_command_topic, sizeof(m_command_topic), "device/%s/command",
           psn.c_str());
}

void MQTTClient::TaskLoop() {
  ESP_LOGI(TAG, "MQTT message task started");
  std::vector<uint8_t> request, data, response;
  App &app = App::GetInstance();
  uint8_t service;
  uint16_t message_id;
  MQTTMsgTaskEvent event;
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
      response.clear();
      request = kMQTTMsgQueue.front();
      kMQTTMsgQueue.pop();

      if (request.size() < sizeof(uint8_t) + sizeof(uint16_t)) {
        ESP_LOGW(TAG, "Invalid request data length: %d", (int)request.size());
        continue;
      }

      ESP_LOG_BUFFER_HEX_LEVEL(TAG, request.data(), request.size(),
                               ESP_LOG_DEBUG);
      service = request[0];
      message_id = (request[2] << 8) | request[1];
      data = std::vector<uint8_t>(request.begin() + 3, request.end());
      ESP_LOGD(TAG, "MQTT Message received, service: 0x%02x", service);
      app.ExecSrvFromMQTT(service, data, response);

      if (response.empty()) {
        ESP_LOGW(TAG, "MQTT Message response is empty");
        continue;
      }

      MQTTMessageHeader header(service, message_id);
      std::vector<uint8_t> header_data = header.Serialize();
      response.insert(response.begin(), header_data.begin(), header_data.end());
      Publish(response);
      DELAY_MS(1);
    }
  }

  while (!kMQTTMsgQueue.empty()) {
    kMQTTMsgQueue.pop();
  }
  ESP_LOGI(TAG, "MQTT message task exited");
  vTaskDelete(NULL);
}
