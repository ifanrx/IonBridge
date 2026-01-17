#include "telemetry_task.h"

#include <sys/param.h>

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "admin.pb.h"
#include "controller.h"
#include "data_types.pb.h"
#include "esp_check.h"
#include "esp_chip_info.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "ionbridge.h"
#include "machine_info.h"
#include "mqtt_app.h"
#include "multi_heap.h"
#include "nvs.h"
#include "nvs_key.h"
#include "nvs_namespace.h"
#include "pb.h"
#include "pb_encode.h"
#include "pb_utils.h"
#include "port.h"
#include "port_manager.h"
#include "portmacro.h"
#include "sdkconfig.h"
#include "service.h"
#include "telemetry.pb.h"
#include "telemetry_timer.h"
#include "topic_parser.h"
#include "utils.h"
#include "version.h"
#include "wifi.h"

#ifdef CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
#include "esp_core_dump.h"
#include "esp_log_buffer.h"
#include "esp_log_level.h"
#include "esp_partition.h"
#endif

#ifdef CONFIG_MCU_MODEL_SW3566
#include "rpc.h"
#include "sw3566_data_types.h"
#endif

#ifdef CONFIG_ENABLE_PROGRAMMER
#include "firmware_programmer.h"
#endif

#ifdef CONFIG_ACDC_MODULE
#include "acdc.h"
#endif

#define TELEMETRY_TASK_STACK_SIZE CONFIG_TELEMETRY_TASK_STACK_SIZE
#define TELEMETRY_TASK_PRIORITY CONFIG_TELEMETRY_TASK_PRIORITY

#define SET_DEFAULT_VERSION(version) \
  do {                               \
    version.major = 0;               \
    version.minor = 0;               \
    version.patch = 1;               \
  } while (0)

static const char *TAG = "TelemetryTask";

TelemetryTask::~TelemetryTask() {
  // Delete mutex
  if (buffer_mutex_ != nullptr) {
    vSemaphoreDelete(buffer_mutex_);
    buffer_mutex_ = nullptr;
  }

  if (mqtt_msg_queue_ != nullptr) {
    vQueueDelete(mqtt_msg_queue_);
  }
}

void TelemetryTask::Start() {
  if (started_) {
    return;
  }
  started_ = true;

#ifdef CONFIG_TELEMETRY_ENABLED
  StartTelemetryTimerTask();

  InitBufferPool();

  mqtt_msg_queue_ = xQueueCreate(BUFFER_COUNT, sizeof(QueueItem));
  if (mqtt_msg_queue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create MQTT message queue");
    return;
  }

  BaseType_t xStatus = xTaskCreate(&TelemetryTask::TaskLoopWrapper, "telemetry",
                                   TELEMETRY_TASK_STACK_SIZE, this,
                                   TELEMETRY_TASK_PRIORITY, &task_handle_);
  if (xStatus != pdPASS) {
    ESP_LOGE(TAG, "Failed to create TelemetryTask");
    return;
  }
#endif
}

void TelemetryTask::ReportMQTTMessage(const TelemetryCommand command,
                                      const pb_TelemetryData &data) {
  std::vector<uint8_t> payload = serialize_pb(pb_TelemetryData_fields, &data);
  if (payload.size() > BUFFER_SIZE) {
    ESP_LOGE(TAG, "Message too large");
    return;
  }

  // Acquire a buffer ID
  BufferId id = AcquireBuffer(payload.size());
  ESP_RETURN_VOID_ON_FALSE(id != INVALID_BUFFER_ID, ESP_ERR_NO_MEM, TAG,
                           "No available buffer");

  // Get buffer and serialize message
  uint8_t *buffer = GetBuffer(id);
  // Copy to buffer
  memcpy(buffer, payload.data(), payload.size());
  QueueItem item = {.id = id, .command = static_cast<uint8_t>(command)};

  BaseType_t xStatus = xQueueSend(mqtt_msg_queue_, &item, pdMS_TO_TICKS(10));
  if (xStatus != pdPASS) {
    ESP_LOGE(TAG, "Failed to queue message, command: 0x%04x",
             static_cast<uint8_t>(command));
    ReleaseBuffer(id);
  }
}

void TelemetryTask::ReportDeviceBootInfo() {
  CHECK_MQTT_CONNECTED();
#ifdef CONFIG_TELEMETRY_ENABLED
  esp_err_t __attribute__((unused)) err;
  int64_t uptime = esp_timer_get_time();
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  size_t totalBytes;
  size_t usedBytes;
  ESP_RETURN_VOID_ON_ERROR(
      esp_littlefs_info("littlefs", &totalBytes, &usedBytes), TAG,
      "esp_littlefs_info");
  int8_t max_tw_power;
  ESP_RETURN_VOID_ON_ERROR(esp_wifi_get_max_tx_power(&max_tw_power), TAG,
                           "esp_wifi_get_max_tx_power");

  MachineInfo &info = MachineInfo::GetInstance();
  pb_DeviceBootInfo data = pb_DeviceBootInfo_init_zero;

  memcpy(&data.esp32_version, info.GetESP32Version().data(),
         sizeof(data.esp32_version));
#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
  memcpy(&data.sw3566_version, info.GetMCUVersion().data(),
         sizeof(data.sw3566_version));
  memcpy(&data.fpga_version, info.GetFPGAVersion().data(),
         sizeof(data.fpga_version));
  memcpy(&data.zrlib_version, info.GetZRLIBVersion().data(),
         sizeof(data.zrlib_version));
#else
  SET_DEFAULT_VERSION(data.sw3566_version);
  SET_DEFAULT_VERSION(data.fpga_version);
  SET_DEFAULT_VERSION(data.zrlib_version);
#endif

  data.uptime = static_cast<uint64_t>(uptime);
  data.chip_model = chip_info.model;
  data.chip_features = chip_info.features;
  data.chip_revision = chip_info.revision;
  data.chip_cores = chip_info.cores;
  data.esp_idf_version.major = ESP_IDF_VERSION_MAJOR;
  data.esp_idf_version.minor = ESP_IDF_VERSION_MINOR;
  data.esp_idf_version.patch = ESP_IDF_VERSION_PATCH;
  data.fs_total_size = static_cast<uint32_t>(totalBytes);
  data.fs_used_size = static_cast<uint32_t>(usedBytes);
  data.wifi_max_tx_power = max_tw_power;
  data.reset_reason = static_cast<uint32_t>(esp_reset_reason());
  data.active_mcu_count =
      static_cast<uint32_t>(PortManager::GetInstance().GetActivePortCount());
  const std::string &hwrev = info.GetHwRev();
  strlcpy(data.hardware_revision, hwrev.c_str(),
          sizeof(data.hardware_revision));
  const std::string &model = info.GetDeviceModel();
  strlcpy(data.device_model, model.c_str(), sizeof(data.device_model));
  const std::string &family = info.GetProductFamily();
  strlcpy(data.product_family, family.c_str(), sizeof(data.product_family));

  get_ipv4_addr(data.ipv4);
#ifdef CONFIG_LWIP_IPV6
  get_ipv6_addr(data.ipv6);
#else
  memset(data.ipv6, 0, sizeof(data.ipv6));
#endif
  memcpy(data.ble_mac, info.GetBleMac().data(), 6);

  pb_TelemetryData payload = pb_TelemetryData_init_zero;
  payload.which_data = pb_TelemetryData_device_boot_info_tag;
  payload.data.device_boot_info = data;
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(
                               TelemetryCommand::DEVICE_BOOT_INFO, payload),
                           TAG, "PublishDeviceBootInfo");

#endif
}

static void RequestSystemTime() {
  CHECK_MQTT_CONNECTED();
#ifdef CONFIG_TELEMETRY_ENABLED
  pb_AdminCommandResponse payload = pb_AdminCommandResponse_init_zero;
  pb_RequestSystemTime data = pb_RequestSystemTime_init_zero;
  payload.which_payload = pb_AdminCommandResponse_request_system_time_tag;
  payload.payload.request_system_time = data;
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(
                               AdminCommand::REQUEST_SYSTEM_TIME, payload),
                           TAG, "Publish RequestTimeSyncInfo");
#endif
}

#ifdef CONFIG_MCU_MODEL_SW3566
static void ReportSW3566BootloaderInfo() {
  CHECK_MQTT_CONNECTED();
#ifdef CONFIG_TELEMETRY_ENABLED
  pb_SW3566BootloaderInfo data = pb_SW3566BootloaderInfo_init_zero;
  PortManager &pm = PortManager::GetInstance();
  int port_count = 0;
  for (const Port &port : pm) {
    port.GetBootloaderInfo(&data.infos[port_count].version,
                           &data.infos[port_count].flash_timestamp);
    port_count++;
  }
  data.infos_count = port_count;

  pb_TelemetryData payload = pb_TelemetryData_init_zero;
  payload.which_data = pb_TelemetryData_sw3566_bootloader_info_tag;
  payload.data.sw3566_bootloader_info = data;
  ESP_RETURN_VOID_ON_ERROR(
      MQTTClient::GetInstance()->Publish(
          TelemetryCommand::SW3566_BOOTLOADER_INFO, payload),
      TAG, "ReportBootloaderInfo");
#endif
}
#endif

void TelemetryTask::ReportMQTTConnectionTime() {
  RequestLicense();
  // 上报设备信息
  ReportDeviceBootInfo();
  RequestSystemTime();
#ifdef CONFIG_MCU_MODEL_SW3566
  ReportSW3566BootloaderInfo();
#endif
  ReportUpgradeResult();
#ifdef CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
  ReportCoredumpInfo();
#endif
#ifdef CONFIG_ACDC_MODULE
  ReportACDCSelfTestInfo();
#endif
}

void TelemetryTask::ReportUpgradeResult() {
  using nvs_namespace::user_data::OTA;

  ConfirmResult confirm_result;
  size_t size = sizeof(confirm_result);
  esp_err_t err = OTA.Get(nvs_keys::OTA_CONFIRM_RESULT,
                          reinterpret_cast<uint8_t *>(&confirm_result), size);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    return;
  }
  ESP_RETURN_VOID_ON_ERROR(err, TAG, "Failed to get OTA confirm result: 0x%04x",
                           err);
  OTA.EraseKey(nvs_keys::OTA_CONFIRM_RESULT);
  pb_UpgradeStatus status = pb_UpgradeStatus_UPGRADE_SUCCESS;
  pb_UpgradeFailureReason reason =
      confirm_result.success ? pb_UpgradeFailureReason_REASON_NONE
                             : pb_UpgradeFailureReason_REASON_CONFIRM_FAILED;
  ESP_LOGI(TAG, "Reporting upgrade info: status: %d, reason: %d", status,
           reason);
  ReportUpgradeData(Version(confirm_result.esp32_version),
#ifdef CONFIG_MCU_MODEL_SW3566
                    Version(confirm_result.sw3566_version),
                    Version(confirm_result.fpga_version),
#else
                    Version(0, 0, 0), Version(0, 0, 0),
#endif
                    status, reason, ESP_OK);
}

void TelemetryTask::ReportUpgradeData(const Version &esp32_version,
                                      const Version &mcu_version,
                                      const Version &fpga_version,
                                      pb_UpgradeStatus status,
                                      pb_UpgradeFailureReason reason,
                                      esp_err_t esp_err) {
  CHECK_MQTT_CONNECTED();
  ESP_LOGI(TAG, "Publishing upgrade info: status=%d, reason=%d, esp_err=%d",
           status, reason, esp_err);

  pb_UpgradeData data = pb_UpgradeData_init_zero;
  memcpy(&data.mcu_version, mcu_version.ToArray().data(),
         sizeof(data.mcu_version));
  memcpy(&data.fpga_version, fpga_version.ToArray().data(),
         sizeof(data.fpga_version));
  memcpy(&data.esp32_version, esp32_version.ToArray().data(),
         sizeof(data.esp32_version));
  data.status = pb_UpgradeStatus_UPGRADE_START;
  data.reason = pb_UpgradeFailureReason_REASON_NONE;
  data.esp_err = ESP_OK;
  data.active_port_count =
      static_cast<uint32_t>(PortManager::GetInstance().GetActivePortCount());

  pb_TelemetryData payload = pb_TelemetryData_init_zero;
  payload.which_data = pb_TelemetryData_upgrade_data_tag;
  payload.data.upgrade_data = data;
  // Use QoS 1 for upgrade messages to ensure delivery with broker
  // acknowledgment
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(
                               TelemetryCommand::UPGRADE_DATA, payload, 1),
                           TAG, "ReportUpgradeData");
}

esp_err_t TelemetryTask::ReportPingInfo() {
  MQTTClient *mqtt_client = MQTTClient::GetInstance();
  if (!(mqtt_client && mqtt_client->Connected())) {
    return ESP_ERR_INVALID_STATE;
  }
  pb_PingInfo data = pb_PingInfo_init_zero;
  data.dummy = 1;
  pb_TelemetryData payload = pb_TelemetryData_init_zero;
  payload.which_data = pb_TelemetryData_ping_info_tag;
  payload.data.ping_info = data;
  return mqtt_client->Publish(TelemetryCommand::PING, payload);
}

void TelemetryTask::RequestLicense() {
  CHECK_MQTT_CONNECTED();
  GenerateNonce();
  pb_RequestLicenseInfo data = pb_RequestLicenseInfo_init_zero;
  memcpy(data.nonce, nonce_, sizeof(nonce_));

  pb_AdminCommandResponse payload = pb_AdminCommandResponse_init_zero;
  payload.which_payload = pb_AdminCommandResponse_request_license_info_tag;
  payload.payload.request_license_info = data;
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(
                               AdminCommand::REQUEST_LICENSE, payload, 1),
                           TAG, "Publish RequestLicenseInfo");
}

void TelemetryTask::RequestShutdownBle() {
  CHECK_MQTT_CONNECTED();
  ESP_LOGI(TAG, "Requesting shutdown ble");
  pb_RequestShutdownBleInfo data = pb_RequestShutdownBleInfo_init_zero;
  pb_AdminCommandResponse payload = pb_AdminCommandResponse_init_zero;
  payload.which_payload = pb_AdminCommandResponse_request_shutdown_ble_info_tag;
  payload.payload.request_shutdown_ble_info = data;
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(
                               AdminCommand::REQUEST_SHUTDOWN_BLE, payload),
                           TAG, "RequestShutdownBle");
}

void TelemetryTask::ReportUpgradeStart() {
  MQTTClient *client = MQTTClient::GetInstance();
  if (client == nullptr) {
    ESP_LOGE(TAG, "MQTT client is null");
    return;
  }

  const Version esp32_version =
      Version(MachineInfo::GetInstance().GetESP32Version());
#ifdef CONFIG_MCU_MODEL_SW3566
  const Version mcu_version =
      Version(MachineInfo::GetInstance().GetMCUVersion());
  const Version fpga_version =
      Version(MachineInfo::GetInstance().GetFPGAVersion());
#else
  const Version mcu_version = Version(0, 0, 0);
  const Version fpga_version = Version(0, 0, 0);
#endif
  ReportUpgradeData(esp32_version, mcu_version, fpga_version,
                    pb_UpgradeStatus_UPGRADE_START,
                    pb_UpgradeFailureReason_REASON_NONE, ESP_OK);

  // Brief delay to allow message to queue and begin transmission before
  // stopping MQTT client. esp_mqtt_client_stop() will wait for graceful
  // shutdown and flush pending messages, but this gives the message a head
  // start.
  vTaskDelay(pdMS_TO_TICKS(100));

  client->Stop(true);
}

void TelemetryTask::ReportUpgradeError(esp_err_t err) {
  const Version esp32_version =
      Version(MachineInfo::GetInstance().GetESP32Version());
#ifdef CONFIG_MCU_MODEL_SW3566
  const Version mcu_version =
      Version(MachineInfo::GetInstance().GetMCUVersion());
  const Version fpga_version =
      Version(MachineInfo::GetInstance().GetFPGAVersion());
#else
  const Version mcu_version = Version(0, 0, 0);
  const Version fpga_version = Version(0, 0, 0);
#endif
  ReportUpgradeData(esp32_version, mcu_version, fpga_version,
                    pb_UpgradeStatus_UPGRADE_FAIL,
                    pb_UpgradeFailureReason_REASON_ESP_ERROR, err);
}

void TelemetryTask::SubscribeTelemetryStream() {
  subscribed_at_ = esp_timer_get_time();
  ESP_LOGI(TAG, "Telemetry stream subscription started.");
}

void TelemetryTask::UnsubscribeTelemetryStream() { subscribed_at_ = -1; }

bool TelemetryTask::IsTelemetryStreamSubscribed() {
  if (subscribed_at_ <= 0) {
    return false;
  }
  bool expired = esp_timer_get_time() - subscribed_at_ > kSubscribeTimeoutUs;
  if (expired) {
    ESP_LOGW(TAG, "Telemetry stream subscription has expired.");
    UnsubscribeTelemetryStream();
  }
  return !expired;
}

bool TelemetryTask::IsPaused() { return paused; }

void TelemetryTask::Pause() {
  paused = true;
  ESP_LOGI(TAG, "TelemetryTask has been paused.");
}

void TelemetryTask::Resume() {
  paused = false;
  ESP_LOGI(TAG, "TelemetryTask has been resumed.");
}

void TelemetryTask::GenerateNonce() {
  for (int i = 0; i < sizeof(nonce_); i++) {
    nonce_[i] = '0' + (esp_random() % 10);
  }
}

bool TelemetryTask::ValidateNonce(const char *input_nonce) {
  return strcmp(reinterpret_cast<const char *>(nonce_), input_nonce) == 0;
}

void TelemetryTask::ReportPowerAllocationData(uint16_t power_budget,
                                              uint16_t remaining_power) {
#if CONFIG_MCU_MODEL_FAKE_SW3566
  return;
#endif
  CHECK_MQTT_CONNECTED();
  pb_PowerAllocationData data = pb_PowerAllocationData_init_zero;
  data.power_budget = power_budget;
  data.remaining_power = remaining_power;
  data.temperature = 0.0f;
  uint8_t adc_value = 0;
#if defined(CONFIG_MCU_MODEL_SW3566) && \
    !defined(CONFIG_FPGA_DUAL_FIRMWARE_WORKAROUND)
  ESP_RETURN_VOID_ON_ERROR(rpc::fpga::read_adc_value(&adc_value), TAG,
                           "Failed to read ADC value");
#endif
  data.adc_value = adc_value;

  PortManager &pm = PortManager::GetInstance();
  int port_count = 0;
  for (const Port &port : pm) {
    data.port_allocations[port_count] = {
        .source_cap = static_cast<uint8_t>(port.MaxPowerBudget()),
        .usage = static_cast<uint8_t>(port.GetPowerUsage()),
    };
    port_count++;
  }
  data.port_allocations_count = port_count;

  pb_TelemetryData payload = pb_TelemetryData_init_zero;
  payload.which_data = pb_TelemetryData_power_allocation_data_tag;
  payload.data.power_allocation_data = data;
  this->ReportMQTTMessage(TelemetryCommand::POWER_ALLOCATION_DATA, payload);
}

#if defined(CONFIG_MCU_MODEL_SW3566) && defined(CONFIG_ENABLE_PROGRAMMER)
void TelemetryTask::ReportProgrammerStart() {
  CHECK_MQTT_CONNECTED();
  pb_ProgrammerStart data = pb_ProgrammerStart_init_zero;
  FirmwareProgrammer &pm = FirmwareProgrammer::GetInstance();
  memcpy(&data.programmer_version, pm.GetProgrammerVersion().ToArray().data(),
         sizeof(data.programmer_version));

  pb_TelemetryData payload = pb_TelemetryData_init_zero;
  payload.which_data = pb_TelemetryData_programmer_start_tag;
  payload.data.programmer_start = data;
  this->ReportMQTTMessage(TelemetryCommand::PROGRAMMER_START, payload);
}

void TelemetryTask::ReportProgrammerFinish() {
  CHECK_MQTT_CONNECTED();
  pb_ProgrammerFinish data = pb_ProgrammerFinish_init_zero;
  FirmwareProgrammer &pm = FirmwareProgrammer::GetInstance();
  data.success = pm.IsStateDone();
  data.error_reason = static_cast<pb_ProgrammerState>(pm.GetErrorReason());
  pb_TelemetryData payload = pb_TelemetryData_init_zero;
  payload.which_data = pb_TelemetryData_programmer_finish_tag;
  payload.data.programmer_finish = data;
  this->ReportMQTTMessage(TelemetryCommand::PROGRAMMER_FINISH, payload);
}
#endif

void TelemetryTask::TaskLoopWrapper(void *param) {
  TelemetryTask *task = static_cast<TelemetryTask *>(param);
  task->TaskLoop();
}

void TelemetryTask::TaskLoop() {
  QueueItem item;
  MQTTClient *mqtt_client;
  MqttTopicParser::Command::CommandInfo command;
  command.source = MqttTopicParser::Command::CommandSource::TELEMETRY;
  while (mqtt_msg_queue_ != nullptr) {
    item.id = INVALID_BUFFER_ID;
    BaseType_t xStatus = xQueueReceive(mqtt_msg_queue_, &item, portMAX_DELAY);
    if (xStatus != pdPASS) {
      continue;
    }
    if (item.id == INVALID_BUFFER_ID) {
      continue;
    }

    mqtt_client = MQTTClient::GetInstance();
    if (!(mqtt_client && mqtt_client->Connected())) {
      continue;
    }

    command.id = item.command;
    if (!command.IsValid()) {
      continue;
    }

    const BufferType buffer = GetBuffer(item.id);
    size_t buffer_size = GetBufferSize(item.id);
    std::string topic =
        MqttTopicParser::Topic::GetTelemetryTopic(TelemetryCommand(command.id));
    ESP_ERROR_COMPLAIN(
        mqtt_client->Publish(
            topic, std::vector<uint8_t>(buffer, buffer + buffer_size)),
        "Publish MQTT message");
    ReleaseBuffer(item.id);
    item.id = INVALID_BUFFER_ID;
  }
}

void TelemetryTask::ReportOutOfMemory(uint32_t caps) const {
  CHECK_MQTT_CONNECTED();
  static bool memory_exceeded = false;
  multi_heap_info_t info;
  heap_caps_get_info(&info, caps);
  uint32_t freeHeapSize = info.total_free_bytes;
  if (freeHeapSize <= CONFIG_MEMORY_ALERT_THRESHOLD_BYTES && !memory_exceeded) {
    memory_exceeded = true;

    pb_OutOfMemoryAlertInfo data = pb_OutOfMemoryAlertInfo_init_zero;
    data.free_heap_size = freeHeapSize;
    data.free_internal_heap_size = esp_get_free_internal_heap_size();
    data.minimum_free_heap_size = info.minimum_free_bytes;
    data.free_heap_size_threshold = CONFIG_MEMORY_ALERT_THRESHOLD_BYTES;
    ESP_LOGW(TAG,
             "OOM: threshold: %d, free heap size: %" PRIu32
             ", free internal heap size: %" PRIu32
             ", minimum free heap size: %" PRIu32,
             CONFIG_MEMORY_ALERT_THRESHOLD_BYTES, freeHeapSize,
             data.free_internal_heap_size, data.minimum_free_heap_size);

    MQTTClient *mqtt = MQTTClient::GetInstance();
    if (mqtt->Connected()) {
      pb_TelemetryData payload = pb_TelemetryData_init_zero;
      payload.which_data = pb_TelemetryData_out_of_memory_alert_info_tag;
      payload.data.out_of_memory_alert_info = data;
      ESP_ERROR_COMPLAIN(
          mqtt->Publish(TelemetryCommand::OUT_OF_MEMORY_ALERT, payload, 1),
          "Publish OutOfMemoryAlertInfo");
    }
  }
}

void TelemetryTask::InitBufferPool() {
  // Initialize all buffers as free
  for (int i = 0; i < BUFFER_COUNT; i++) {
    msg_buffer_sizes_[i] = 0;
  }
  buffer_mutex_ = xSemaphoreCreateMutex();
}

TelemetryTask::BufferId TelemetryTask::AcquireBuffer(size_t buffer_size) {
  // Validate input
  if (buffer_size == 0) {
    ESP_LOGE(TAG, "Requested zero-sized buffer");
    return INVALID_BUFFER_ID;
  }

  if (xSemaphoreTake(buffer_mutex_, pdMS_TO_TICKS(kMutexTimeoutMs)) != pdTRUE) {
    ESP_LOGW(TAG, "Failed to acquire buffer mutex");
    return INVALID_BUFFER_ID;
  }

  BufferId result = INVALID_BUFFER_ID;
  bool all_buffers_in_use = true;

  for (int i = 0; i < BUFFER_COUNT; i++) {
    if (msg_buffers_[i] == nullptr) {
      all_buffers_in_use = false;
      // allocate buffer
      size_t alloc_size = std::min(buffer_size, BUFFER_SIZE);

      msg_buffers_[i] = new uint8_t[alloc_size]();  // zero-initialize
      if (!msg_buffers_[i]) {
        ESP_LOGE(TAG, "Failed to allocate buffer of size %d", (int)alloc_size);
        break;
      }

      // Zero-initialize buffer
      memset(msg_buffers_[i], 0, alloc_size);
      msg_buffer_sizes_[i] = alloc_size;
      result = i;
      ESP_LOGD(TAG, "Buffer ID %d acquired (%d bytes)", i, (int)alloc_size);
      break;
    }
  }

  if (result == INVALID_BUFFER_ID && all_buffers_in_use) {
    ESP_LOGW(TAG, "All buffers in use");
  }

  xSemaphoreGive(buffer_mutex_);
  return result;
}

void TelemetryTask::ReleaseBuffer(BufferId id) {
  if (id >= BUFFER_COUNT) {
    ESP_LOGW(TAG, "Attempted to release invalid buffer ID: %d", id);
    return;
  }

  // Try multiple times to acquire the mutex
  constexpr int MAX_RETRIES = 3;
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    if (xSemaphoreTake(buffer_mutex_, pdMS_TO_TICKS(kMutexTimeoutMs)) ==
        pdTRUE) {
      if (msg_buffers_[id]) {
        // Secure buffer clearing before release
        if (msg_buffer_sizes_[id] > 0) {
          memset(msg_buffers_[id], 0, msg_buffer_sizes_[id]);
        }

        delete[] msg_buffers_[id];
        msg_buffers_[id] = nullptr;
        msg_buffer_sizes_[id] = 0;
        ESP_LOGD(TAG, "Buffer ID %d released", id);
      } else {
        ESP_LOGW(TAG, "Buffer ID %d already released", id);
      }

      xSemaphoreGive(buffer_mutex_);
      return;
    }

    // Backoff before retry
    vTaskDelay(pdMS_TO_TICKS(10 * (retry + 1)));
    ESP_LOGW(TAG, "Retry %d to acquire mutex for buffer release", retry + 1);
  }

  ESP_LOGE(TAG, "Failed to release buffer ID %d after %d attempts", id,
           MAX_RETRIES);

  // Another option: Add this buffer ID to a pending release queue
  // QueuePendingBufferRelease(id);

  return;
}

TelemetryTask::BufferType TelemetryTask::GetBuffer(BufferId id) {
  return (id < BUFFER_COUNT) ? msg_buffers_[id] : nullptr;
}

size_t TelemetryTask::GetBufferSize(BufferId id) {
  return (id < BUFFER_COUNT) ? msg_buffer_sizes_[id] : 0;
}

void TelemetryTask::ReportTaskExecutionResult(
    uint32_t id, pb_TaskExecutionStatus status) const {
  CHECK_MQTT_CONNECTED();
  if (id == 0) {
    ESP_LOGE(TAG, "Invalid task ID");
    return;
  }

  pb_TelemetryData data = pb_TelemetryData_init_zero;
  data.which_data = pb_TelemetryData_task_execution_result_tag;
  data.data.task_execution_result.task_id = id;
  data.data.task_execution_result.status = status;
  ESP_ERROR_COMPLAIN(MQTTClient::GetInstance()->Publish(
                         TelemetryCommand::TASK_EXECUTION_RESULT, data),
                     "Publish TaskExecutionResult");
}

void TelemetryTask::ReportACDCSelfTestInfo() {
#ifdef CONFIG_ACDC_MODULE
  CHECK_MQTT_CONNECTED();

  ACDCSelfTestStatus status = acdc_get_self_test_status();
  ESP_LOGI(TAG, "Reporting ACDC self-test status: 0x%02x",
           static_cast<uint8_t>(status));

  pb_TelemetryData data = pb_TelemetryData_init_zero;
  data.which_data = pb_TelemetryData_acdc_self_test_info_tag;
  data.data.acdc_self_test_info.status =
      static_cast<pb_ACDCSelfTestStatus>(status);

  ESP_ERROR_COMPLAIN(MQTTClient::GetInstance()->Publish(
                         TelemetryCommand::ACDC_SELF_TEST_INFO, data),
                     "Publish ACDCSelfTestInfo");
#endif
}

#ifdef CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
namespace {
constexpr size_t kCoredumpChunkSize =
    512;  // Reduced to minimize memory pressure (BLE startup needs buffer)
// FreeRTOS stack sizes are expressed in words, not bytes.
constexpr configSTACK_DEPTH_TYPE kCoredumpTaskStackDepth =
    4 * 1024 / sizeof(StackType_t);
}  // namespace

static void StreamCoredumpInfo(void *arg) {
  pb_TelemetryData data = pb_TelemetryData_init_zero;
  data.which_data = pb_TelemetryData_coredump_slice_tag;
  pb_CoredumpPartitionSlice *coredump_slice = &data.data.coredump_slice;

  uint32_t reported_at = get_epoch_timestamp();
  ESP_LOGI(TAG, "Streaming coredump info, reported_at: %" PRIu32, reported_at);
  size_t remaining = 0, offset = 0;
  MQTTClient *mqtt_client = MQTTClient::GetInstance();

  const esp_partition_t *coredump_part = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, NULL);
  if (coredump_part == NULL) {
    ESP_LOGE(TAG, "No coredump partition found");
    goto EXIT;
  }
  remaining = coredump_part->size;

  coredump_slice->data = new uint8_t[kCoredumpChunkSize];
  if (coredump_slice->data == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate coredump slice");
    goto EXIT;
  }

  // Stream in slices
  while (remaining > 0) {
    size_t chunk_size = std::min(remaining, kCoredumpChunkSize);
    remaining -= chunk_size;

    coredump_slice->timestamp = reported_at;
    coredump_slice->offset = offset;
    coredump_slice->size = chunk_size;
    coredump_slice->data_count = chunk_size / sizeof(uint8_t);
    coredump_slice->last = remaining == 0;

    esp_err_t err = esp_partition_read(coredump_part, offset,
                                       coredump_slice->data, chunk_size);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read coredump partition: %s",
               esp_err_to_name(err));
      break;
    }
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, coredump_slice->data, 4, ESP_LOG_DEBUG);

    // Send chunk
    err = mqtt_client->Publish(TelemetryCommand::COREDUMP_INFO, data, 1);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to publish coredump slice: %s",
               esp_err_to_name(err));
      break;
    }
    if (coredump_slice->last) {
      ESP_LOGI(TAG, "Coredump info sent successfully");
      ESP_ERROR_COMPLAIN(esp_core_dump_image_erase(),
                         "esp_core_dump_image_erase failed");
      break;
    }

    offset += chunk_size;
    DELAY_MS(50);  // Increased delay to give other tasks (especially BLE)
                   // breathing room
  }
  delete[] coredump_slice->data;

EXIT:
  vTaskDelete(NULL);
}

void TelemetryTask::ReportCoredumpInfo() {
  esp_err_t err = esp_core_dump_image_check();
  if (err == ESP_ERR_NOT_FOUND || err == ESP_ERR_INVALID_SIZE) {
    ESP_LOGI(TAG, "No coredump image found");
    return;
  }
  if (err == ESP_ERR_INVALID_CRC) {
    ESP_LOGW(TAG, "Coredump image is corrupted");
    ESP_RETURN_VOID_ON_ERROR(esp_core_dump_image_erase(), TAG,
                             "Failed to erase coredump image");
    return;
  }
  ESP_RETURN_VOID_ON_ERROR(err, TAG, "Failed to check coredump image: 0x%04X",
                           err);
  // Stack size: 4KB. Could potentially be reduced to 3KB after testing with
  // the smaller chunk size (512 bytes), but 4KB is safe for now.
  xTaskCreate(StreamCoredumpInfo, "codedump_info", kCoredumpTaskStackDepth,
              NULL, tskIDLE_PRIORITY + 1, NULL);
}
#endif
