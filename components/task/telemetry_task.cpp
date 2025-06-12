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

#include "controller.h"
#include "esp_check.h"
#include "esp_chip_info.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_log_buffer.h"
#include "esp_log_level.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "ionbridge.h"
#include "machine_info.h"
#include "mqtt_app.h"
#include "mqtt_message.h"
#include "multi_heap.h"
#include "nvs.h"
#include "nvs_default.h"
#include "nvs_namespace.h"
#include "port.h"
#include "port_manager.h"
#include "portmacro.h"
#include "sdkconfig.h"
#include "telemetry_timer.h"
#include "version.h"
#include "wifi.h"

#ifdef CONFIG_MCU_MODEL_SW3566
#include "rpc.h"
#include "sw3566_data_types.h"
#endif

#define TELEMETRY_TASK_STACK_SIZE CONFIG_TELEMETRY_TASK_STACK_SIZE
#define TELEMETRY_TASK_PRIORITY CONFIG_TELEMETRY_TASK_PRIORITY

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

  mqtt_msg_queue_ = xQueueCreate(BUFFER_COUNT, sizeof(uint8_t));
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

void TelemetryTask::ReportMQTTMessage(const MQTTMessage &msg) {
  std::vector<uint8_t> temp = msg.Serialize();
  if (temp.size() > BUFFER_SIZE) {
    ESP_LOGE(TAG, "Message too large");
    return;
  }

  // Acquire a buffer ID
  BufferId id = AcquireBuffer(temp.size());
  ESP_RETURN_VOID_ON_FALSE(id != INVALID_BUFFER_ID, ESP_ERR_NO_MEM, TAG,
                           "No available buffer");

  // Get buffer and serialize message
  uint8_t *buffer = GetBuffer(id);
  // Copy to buffer
  memcpy(buffer, temp.data(), temp.size());

  BaseType_t xStatus = xQueueSend(mqtt_msg_queue_, &id, pdMS_TO_TICKS(10));
  if (xStatus != pdPASS) {
    ESP_LOGE(TAG, "Failed to queue message, service: 0x%04x",
             msg.header.GetService());
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

  DeviceBootInfo data;
  MachineInfo &info = MachineInfo::GetInstance();
  std::array<uint8_t, 3> esp32_version = info.GetESP32Version(),
#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
                         sw3566_version = info.GetMCUVersion(),
                         fpga_version = info.GetFPGAVersion(),
#endif
                         zrlib_version = info.GetZRLIBVersion();

  std::copy(esp32_version.begin(), esp32_version.end(), data.esp32);
#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
  std::copy(sw3566_version.begin(), sw3566_version.end(), data.mcu);
  std::copy(fpga_version.begin(), fpga_version.end(), data.fpga);
#endif
  std::copy(zrlib_version.begin(), zrlib_version.end(), data.zrlib);
  data.uptime = static_cast<uint64_t>(uptime);
  data.chipModel = static_cast<uint16_t>(chip_info.model);
  data.chipFeatures = chip_info.features;
  data.chipRevision = chip_info.revision;
  data.chipCores = chip_info.cores;
  data.ESPIdfVersion[0] = ESP_IDF_VERSION_MAJOR;
  data.ESPIdfVersion[1] = ESP_IDF_VERSION_MINOR;
  data.ESPIdfVersion[2] = ESP_IDF_VERSION_PATCH;
  data.fsTotalSize = static_cast<uint32_t>(totalBytes);
  data.fsUsedSize = static_cast<uint32_t>(usedBytes);
  data.wifiMaxTxPower = max_tw_power;
  data.resetReason = static_cast<int32_t>(esp_reset_reason());
  ESP_LOGD(TAG, "Publishing reset reason: %d", (int)data.resetReason);
  data.activeMCUCount = PortManager::GetInstance().GetActivePortCount();
  memset(data.hardwareRev, 0, sizeof(data.hardwareRev));
  const std::string &hwrev = info.GetHwRev();
  strncpy(data.hardwareRev, hwrev.c_str(), sizeof(data.hardwareRev) - 1);
  data.hardwareRev[sizeof(data.hardwareRev) - 1] = '\0';
  memset(data.deviceModel, 0, sizeof(data.deviceModel));
  const std::string &model = info.GetDeviceModel();
  strncpy(data.deviceModel, model.c_str(), sizeof(data.deviceModel) - 1);
  data.deviceModel[sizeof(data.deviceModel) - 1] = '\0';
  memset(data.productFamily, 0, sizeof(data.productFamily));
  const std::string &family = info.GetProductFamily();
  strncpy(data.productFamily, family.c_str(), sizeof(data.productFamily) - 1);
  data.productFamily[sizeof(data.productFamily) - 1] = '\0';

  get_ipv4_addr(data.ipv4);
#ifdef CONFIG_LWIP_IPV6
  get_ipv6_addr(data.ipv6);
#else
  memset(data.ipv6, 0, 16);
#endif
  memcpy(data.bleAddress, info.GetBleMac().data(), 6);
  std::vector<uint8_t> payload = data.Serialize();
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(payload), TAG,
                           "PublishDeviceBootInfo");

#endif
}

static void RequestSystemTime() {
  CHECK_MQTT_CONNECTED();
#ifdef CONFIG_TELEMETRY_ENABLED
  RequestSystemTimeInfo data;
  std::vector<uint8_t> payload = data.Serialize();
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(payload), TAG,
                           "Publish RequestTimeSyncInfo");
#endif
}

void TelemetryTask::ReportMqttConnectionTime() {
  RequestLicense();
  ReportDeviceBootInfo();
  RequestSystemTime();
  ReportUpgradeResult();
}

void TelemetryTask::ReportESP32UpgradeError(const Version &version,
                                            esp_err_t err) {
  ReportESP32UpgradeInfo(version, UpgradeStatus::UPGRADE_FAIL,
                         UpgradeFailureReason::REASON_ESP_ERROR, err);
}

void TelemetryTask::ReportESP32UpgradeInfo(const Version &version,
                                           UpgradeStatus status,
                                           UpgradeFailureReason reason,
                                           esp_err_t esp_err) {
  CHECK_MQTT_CONNECTED();
  ESP_LOGI(TAG,
           "Publishing ESP32 upgrade info: status=%d, reason=%d, esp_err=%d",
           status, reason, esp_err);

  ESP32UpgradeInfo data;
  data.version[0] = version.major;
  data.version[1] = version.minor;
  data.version[2] = version.revision;
  data.status = status;
  data.reason = reason;
  data.esp_err = esp_err;

  ESP_RETURN_VOID_ON_ERROR(
      MQTTClient::GetInstance()->Publish(data.Serialize(), 1), TAG,
      "ReportESP32UpgradeInfo");
}

void TelemetryTask::ReportUpgradeResult() {
  ConfirmResult confirm_result;
  size_t size = sizeof(confirm_result);
  esp_err_t err = OTANVSGet(reinterpret_cast<uint8_t *>(&confirm_result), &size,
                            NVSKey::OTA_CONFIRM_RESULT);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    return;
  }
  ESP_RETURN_VOID_ON_ERROR(err, TAG, "Failed to get OTA confirm result: 0x%04x",
                           err);
  OTANVSEraseKey(NVSKey::OTA_CONFIRM_RESULT);
  UpgradeStatus status = UpgradeStatus::UPGRADE_SUCCESS;
  UpgradeFailureReason reason =
      confirm_result.success ? UpgradeFailureReason::REASON_NONE
                             : UpgradeFailureReason::REASON_CONFIRM_FAILED;
  ESP_LOGI(TAG, "Reporting upgrade info: status: %d, reason: %d", status,
           reason);
  ReportESP32UpgradeInfo(Version(confirm_result.esp32_version), status, reason,
                         ESP_OK);
}

void TelemetryTask::ReportAllUpgradeInfo(bool confirmed,
                                         bool confirmed_timeout) {
  static bool reported = false;
  CHECK_MQTT_CONNECTED();
  if (reported) {
    return;
  }
  UpgradeStatus status = UpgradeStatus::UPGRADE_SUCCESS;
  UpgradeFailureReason reason = UpgradeFailureReason::REASON_NONE;
  if (!confirmed) {
    status = UpgradeStatus::UPGRADE_FAIL;
    reason = confirmed_timeout ? UpgradeFailureReason::REASON_CONFIRM_TIMEOUT
                               : UpgradeFailureReason::REASON_CONFIRM_FAILED;
  }
  ESP_LOGI(TAG, "Reporting upgrade info: status: %d, reason: %d", status,
           reason);
  ReportESP32UpgradeInfo(Version(MachineInfo::GetInstance().GetESP32Version()),
                         status, reason, ESP_OK);
  reported = true;
}

esp_err_t TelemetryTask::ReportPingInfo() {
  PingInfo data;
  MQTTClient *mqtt_client = MQTTClient::GetInstance();
  if (!(mqtt_client && mqtt_client->Connected())) {
    return ESP_ERR_INVALID_STATE;
  }
  return mqtt_client->Publish(data.Serialize());
}

void TelemetryTask::ReportOTAConfirmInfo() {}

void TelemetryTask::RequestLicense() {
  CHECK_MQTT_CONNECTED();
  RequestLicenseInfo data;
  std::vector<uint8_t> payload = data.Serialize();
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(payload, 1), TAG,
                           "Publish RequestLicenseInfo");
}

void TelemetryTask::RequestShutdownBle() {
  CHECK_MQTT_CONNECTED();
  ESP_LOGI(TAG, "Requesting shutdown ble");
  RequestShutdownBleInfo data;
  std::vector<uint8_t> payload = data.Serialize();
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(payload), TAG,
                           "RequestShutdownBle");
}

void TelemetryTask::ReportUpgradeStart() {
  MQTTClient *client = MQTTClient::GetInstance();
  if (client == nullptr) {
    ESP_LOGE(TAG, "MQTT client is null");
    return;
  }

  ReportESP32UpgradeInfo(Version(MachineInfo::GetInstance().GetESP32Version()),
                         UpgradeStatus::UPGRADE_START,
                         UpgradeFailureReason::REASON_NONE, ESP_OK);

  client->Stop(true);
}

void TelemetryTask::ReportUpgradeError(esp_err_t err) {
  MQTTClient *client = MQTTClient::GetInstance();
  esp_err_t ret __attribute__((unused)) = ESP_OK;
  if (client == nullptr) {
    ESP_LOGE(TAG, "MQTT client is null");
    return;
  }
  if (!client->Connected() && !client->ForceStart()) {
    return;
  }
  WAIT_FOR_CONDITION_OR_GOTO(
      10, client->Connected(), 100, REPORT_FAILED,
      "Connection timeout while reporting upgrade error");

  ReportESP32UpgradeError(Version(MachineInfo::GetInstance().GetESP32Version()),
                          err);
REPORT_FAILED:
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

void TelemetryTask::ReportPowerAllocationData(uint16_t power_budget,
                                              uint16_t remaining_power) {
#if CONFIG_MCU_MODEL_FAKE_SW3566
  return;
#endif
  CHECK_MQTT_CONNECTED();
  PowerAllocationData data;
  data.power_budget = power_budget;
  data.remaining_power = remaining_power;
  data.temperature = 0;
  data.unused = 0;

  uint8_t adc_value = 0;
#if defined(CONFIG_MCU_MODEL_SW3566)
  ESP_RETURN_VOID_ON_ERROR(rpc::fpga::read_adc_value(&adc_value), TAG,
                           "Failed to read ADC value");
#else
  // FIXME: Implement ADC reading for other models.
#endif
  data.adc_value = adc_value;

  PortManager &pm = PortManager::GetInstance();
  for (int i = 0; i < 8; i++) {
    Port *port = pm.GetPort(i);
    data.port_power_allocations[i] = {
        .source_cap = static_cast<uint8_t>(port ? port->MaxPowerBudget() : 0),
        .usage = static_cast<uint8_t>(port ? port->GetPowerUsage() : 0),
    };
  }
  this->ReportMQTTMessage(data);
}

#ifdef CONFIG_MCU_MODEL_SW3566
void TelemetryTask::ReportPDPCapData(uint8_t port, PcapEntry &entry) {
#if CONFIG_MCU_MODEL_FAKE_SW3566
  return;
#endif
  CHECK_MQTT_CONNECTED();
  PDPCapData data;
  data.port = port;

  constexpr size_t fixed_size = sizeof(PcapEntry);
  size_t total_size = fixed_size + entry.length;
  data.pcap_entry.resize(total_size);
  memcpy(data.pcap_entry.data(), &entry, fixed_size);
  if (entry.length > 0) {
    memcpy(data.pcap_entry.data() + fixed_size, entry.payload, entry.length);
  }

  this->ReportMQTTMessage(data);
}
#endif

void TelemetryTask::TaskLoopWrapper(void *param) {
  TelemetryTask *task = static_cast<TelemetryTask *>(param);
  task->TaskLoop();
}

void TelemetryTask::TaskLoop() {
  BufferId id = INVALID_BUFFER_ID;
  MQTTClient *mqtt_client;
  while (mqtt_msg_queue_ != nullptr) {
    BaseType_t xStatus = xQueueReceive(mqtt_msg_queue_, &id, portMAX_DELAY);
    if (xStatus != pdPASS) {
      continue;
    }
    if (id == INVALID_BUFFER_ID) {
      continue;
    }

    mqtt_client = MQTTClient::GetInstance();
    if (!(mqtt_client && mqtt_client->Connected())) {
      continue;
    }

    const BufferType buffer = GetBuffer(id);
    size_t buffer_size = GetBufferSize(id);
    ESP_ERROR_COMPLAIN(mqtt_client->Publish(
                           std::vector<uint8_t>(buffer, buffer + buffer_size)),
                       "Publish MQTT message");
    ReleaseBuffer(id);
    id = INVALID_BUFFER_ID;
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

    OutOfMemoryAlertInfo data;
    data.freeHeapSize = freeHeapSize;
    data.freeInternalHeapSize = esp_get_free_internal_heap_size();
    data.minimumFreeHeapSize = info.minimum_free_bytes;
    data.thresholdFreeHeapSize = CONFIG_MEMORY_ALERT_THRESHOLD_BYTES;
    ESP_LOGW(TAG,
             "OOM: threshold: %d, free heap size: %" PRIu32
             ", free internal heap size: %" PRIu32
             ", minimum free heap size: %" PRIu32,
             CONFIG_MEMORY_ALERT_THRESHOLD_BYTES, freeHeapSize,
             data.freeInternalHeapSize, data.minimumFreeHeapSize);

    MQTTClient *mqtt = MQTTClient::GetInstance();
    if (mqtt->Connected()) {
      ESP_ERROR_COMPLAIN(mqtt->Publish(data.Serialize(), 1),
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
