#include "telemetry_task.h"

#include <sys/param.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <utility>
#include <vector>

#include "ble.h"
#include "controller.h"
#include "esp_check.h"
#include "esp_chip_info.h"
#include "esp_err.h"
#include "esp_idf_version.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "ionbridge.h"
#include "machine_info.h"
#include "mqtt_app.h"
#include "mqtt_message.h"
#include "nvs_default.h"
#include "nvs_namespace.h"
#include "power_allocator.h"
#include "sdkconfig.h"
#include "version.h"
#include "wifi.h"
#include "wifi_manager.h"

#ifdef CONFIG_INTERFACE_TYPE_UART
#include "uart.h"
#endif

#define SW3566_COUNT CONFIG_SW3566_COUNT
#define CONFIG_MEMORY_ALERT_THRESHOLD_BYTES 10240
#define TASK_INTERVAL_MS 100
#define MAX_TASK_COUNT 10

static const char *TAG = "TelemetryTask";
static uint16_t all_in_one_task_interval_counter = 0;
static void AllInOneTimerTask(void *);

// Initialize static members
std::unique_ptr<TelemetryTask> TelemetryTask::instance_ = nullptr;
std::mutex TelemetryTask::mutex_;

uint16_t TelemetryTask::GetMessageId() {
  static uint16_t message_id = static_cast<uint16_t>(esp_random());
  message_id++;
  return message_id;
}

TelemetryTask::TelemetryTask(PowerAllocator &powerAllocator,
                             DeviceController &controller)
    : allocator_(powerAllocator), controller_(controller) {}

PowerAllocator &TelemetryTask::GetPowerAllocator() { return allocator_; }

void TelemetryTask::Start() {
#ifdef CONFIG_TELEMETRY_ENABLED
  esp_timer_handle_t timer_handle;
  esp_timer_create_args_t timer_args = {
      .callback = &AllInOneTimerTask,
      .arg = this,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "AllInOneTimerTask",
      .skip_unhandled_events = true,
  };
  const uint64_t timer_interval_us = ((uint64_t)TASK_INTERVAL_MS) * 1000;
  esp_timer_create(&timer_args, &timer_handle);
  esp_timer_start_periodic(timer_handle, timer_interval_us);
#endif
}

#ifdef CONFIG_TELEMETRY_ENABLED
static void ReportPowerStatsTask(void *parameters) {
  CHECK_MQTT_CONNECTED();

  TelemetryTask *task = (TelemetryTask *)parameters;
  PowerAllocator *powerAllocator = &task->GetPowerAllocator();
  PortManager &pm = powerAllocator->GetPortManager();
  for (const Port &port : pm) {
    size_t start = 0;
    std::vector<PortStatsData> stats_data =
        port.GetHistoricalStats().GetPointsInRange(start, 360);

    PowerHistoricalData data;
    data.header = {
        .service = TelemetryServiceCommand::POWER_HISTORICAL_DATA,
        .message_id = TelemetryTask::GetInstance()->GetMessageId(),
    };
    data.port = port.Id();
    data.length = static_cast<uint16_t>(stats_data.size());
    data.data = std::move(stats_data);
    ESP_ERROR_COMPLAIN(MQTTClient::GetInstance()->Publish(data.serialize()),
                       "publish power stats");

    // TODO get actual data
    PowerConsumptionData consumptionData;
    consumptionData.header = {
        .service = TelemetryServiceCommand::POWER_CONSUMPTION_DATA,
        .message_id = TelemetryTask::GetInstance()->GetMessageId(),
    };
    consumptionData.port = port.Id();
    consumptionData.length = 1;
    consumptionData.power_consumption = 12;
    ESP_ERROR_COMPLAIN(
        MQTTClient::GetInstance()->Publish(consumptionData.serialize()),
        "publish power consumption");
  }
}
#endif

void TelemetryTask::ReportDeviceBootInfo() {
#ifdef CONFIG_TELEMETRY_ENABLED
  esp_err_t __attribute__((unused)) err;
  int64_t uptime = controller_.get_uptime();
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
  data.header = {
      .service = TelemetryServiceCommand::DEVICE_BOOT_INFO,
      .message_id = GetMessageId(),
  };

  std::array<uint8_t, 3> esp32_version =
                             MachineInfo::GetInstance().GetESP32Version(),
                         sw3566_version =
                             MachineInfo::GetInstance().GetMCUVersion(),
                         fpga_version =
                             MachineInfo::GetInstance().GetFPGAVersion(),
                         zrlib_version =
                             MachineInfo::GetInstance().GetZRLIBVersion();

  std::copy(esp32_version.begin(), esp32_version.end(), data.esp32);
  std::copy(sw3566_version.begin(), sw3566_version.end(), data.mcu);
  std::copy(fpga_version.begin(), fpga_version.end(), data.fpga);
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
  data.activeMCUCount = allocator_.GetPortManager().GetActivePortCount();
  memset(data.hardwareRev, 0, sizeof(data.hardwareRev));
  const std::string &hwrev = MachineInfo::GetInstance().GetHwRev();
  strncpy(data.hardwareRev, hwrev.c_str(), sizeof(data.hardwareRev) - 1);
  data.hardwareRev[sizeof(data.hardwareRev) - 1] = '\0';
  memset(data.deviceModel, 0, sizeof(data.deviceModel));
  const std::string &model = MachineInfo::GetInstance().GetDeviceModel();
  strncpy(data.deviceModel, model.c_str(), sizeof(data.deviceModel) - 1);
  data.deviceModel[sizeof(data.deviceModel) - 1] = '\0';
  memset(data.productFamily, 0, sizeof(data.productFamily));
  const std::string &family = MachineInfo::GetInstance().GetProductFamily();
  strncpy(data.productFamily, family.c_str(), sizeof(data.productFamily) - 1);
  data.productFamily[sizeof(data.productFamily) - 1] = '\0';

  get_ipv4_addr(data.ipv4);
#ifdef CONFIG_LWIP_IPV6
  get_ipv6_addr(data.ipv6);
#else
  memset(data.ipv6, 0, 16);
#endif
  memcpy(data.bleAddress, MachineInfo::GetInstance().GetBleMac().data(), 6);
  std::vector<uint8_t> payload = data.serialize();
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(payload), TAG,
                           "PublishDeviceBootInfo");

#endif
}

static void RequestSystemTime() {
#ifdef CONFIG_TELEMETRY_ENABLED
  RequestSystemTimeInfo data;
  data.header = {
      .service = TelemetryServiceCommand::REQUEST_SYSTEM_TIME,
      .message_id = TelemetryTask::GetInstance()->GetMessageId(),
  };
  std::vector<uint8_t> payload = data.serialize();
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(payload), TAG,
                           "Publish RequestTimeSyncInfo");
#endif
}

void TelemetryTask::ReportMqttConnectionTime() {
  RequestLicense();
  ReportDeviceBootInfo();
  RequestSystemTime();
}

#ifdef CONFIG_TELEMETRY_SYSTEM_STATE_ENABLED
static void ReportDeviceMemoryTask(void *parameters) {
  static uint8_t initial_boot = 1;
  CHECK_MQTT_CONNECTED();
  uint32_t freeHeapSize = esp_get_free_heap_size();
  uint32_t freeInternalHeapSize = esp_get_free_internal_heap_size();
  uint32_t minimumFreeHeapSize = esp_get_minimum_free_heap_size();

  DeviceMemoryInfo data;
  data.header = {
      .service = TelemetryServiceCommand::DEVICE_MEMORY_INFO,
      .message_id = TelemetryTask::GetInstance()->GetMessageId(),
  };
  data.initialBoot = initial_boot;
  data.freeHeapSize = freeHeapSize;
  data.freeInternalHeapSize = freeInternalHeapSize;
  data.minimumFreeHeapSize = minimumFreeHeapSize;
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(data.serialize()),
                           TAG, "ReportDeviceMemoryTask");
  initial_boot = 0;
}

static void ReportWIFIStatsDataTask(void *parameters) {
  CHECK_MQTT_CONNECTED();
  int rssi;
  ESP_RETURN_VOID_ON_ERROR(esp_wifi_sta_get_rssi(&rssi), TAG,
                           "esp_wifi_sta_get_rssi");
  ssid_detail_t ssid_detail;
  ESP_ERROR_COMPLAIN(get_connected_ssid_detail(&ssid_detail),
                     "get_connected_ssid_detail");

  WIFIStatsData data;
  WiFiManager &wm = WiFiManager::GetInstance();
  data.header = {
      .service = TelemetryServiceCommand::WIFI_STATS_DATA,
      .message_id = TelemetryTask::GetInstance()->GetMessageId(),
  };
  data.associationCount = wm.GetAssociationCount();
  data.disassociationCount = wm.GetDisassociationCount();
  data.rssi = rssi;
  data.channel = ssid_detail.channel;
  data.wifiConnectionTime = wm.GetWifiConnectionTime();
  data.mqttConnectionTime = MQTTClient::GetInstance()->GetConnectionTime();
  data.connectivityFailureCount = wm.GetConnectivityFailureCount();
  data.dnsResolutionFailureCount = wm.GetDNSResolutionFailureCount();
  data.mqttConnectionCount = MQTTClient::GetInstance()->GetConnectionCount();
  data.mqttMessageTxCount = MQTTClient::GetInstance()->GetMessageTxCount();
  data.mqttMessageRxCount = MQTTClient::GetInstance()->GetMessageRxCount();

  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(data.serialize()),
                           TAG, "ReportWIFIStatsDataTask");
}

void ReportDeviceTelemetryTask(void *parameters) {
  CHECK_MQTT_CONNECTED();
  ReportDeviceMemoryTask(parameters);
  ReportWIFIStatsDataTask(parameters);
}
#endif

void TelemetryTask::ReportESP32UpgradeError(const Version &version,
                                            esp_err_t err) {
  ReportESP32UpgradeInfo(version, UpgradeStatus::UPGRADE_FAIL,
                         UpgradeFailureReason::REASON_ESP_ERROR, err);
}

void TelemetryTask::ReportESP32UpgradeInfo(const Version &version,
                                           UpgradeStatus status,
                                           UpgradeFailureReason reason,
                                           esp_err_t esp_err) {
#ifdef CONFIG_ENABLE_MQTT
  ESP_LOGI(TAG,
           "Publishing ESP32 upgrade info: status=%d, reason=%d, esp_err=%d",
           status, reason, esp_err);

  ESP32UpgradeInfo data;
  data.header = {
      .service = TelemetryServiceCommand::ESP32_UPGRADE_DATA,
      .message_id = GetMessageId(),
  };
  data.version[0] = version.major;
  data.version[1] = version.minor;
  data.version[2] = version.revision;
  data.status = status;
  data.reason = reason;
  data.esp_err = esp_err;
  ESP_RETURN_VOID_ON_ERROR(
      MQTTClient::GetInstance()->Publish(data.serialize(), 1), TAG,
      "MQTT Publication");
#endif
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

#if defined(CONFIG_TELEMETRY_SYSTEM_STATE_ENABLED) && \
    defined(CONFIG_INTERFACE_TYPE_UART)
static void ReportUartMetricsTask(void *parameters) {
  CHECK_MQTT_CONNECTED();
  UartMetricsData data;
  data.header = {
      .service = TelemetryServiceCommand::UART_METRICS_DATA,
      .message_id = TelemetryTask::GetInstance()->GetMessageId(),
  };
  uart_metrics_t *metrics = uart_get_metrics();
  data.reset_state_count = metrics->reset_state_count;
  data.resend_count = metrics->resend_count;
  data.sent_failed_count = metrics->sent_failed_count;
  data.sent_count = metrics->sent_count;
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(data.serialize()),
                           TAG, "PublishUartMetrics");
}
#endif

esp_err_t TelemetryTask::ReportPingInfo() {
#ifdef CONFIG_ENABLE_MQTT
  PingInfo data;
  data.header = {
      .service = TelemetryServiceCommand::PING,
      .message_id = GetMessageId(),
  };
  return MQTTClient::GetInstance()->Publish(data.serialize());
#else
  return ESP_OK;
#endif
}

void TelemetryTask::ReportOTAConfirmInfo() {}

void TelemetryTask::RequestLicense() {
#ifdef CONFIG_ENABLE_MQTT
  RequestLicenseInfo data;
  data.header = {
      .service = TelemetryServiceCommand::REQUEST_LICENSE,
      .message_id = GetMessageId(),
  };
  std::vector<uint8_t> payload = data.serialize();
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(payload, 1), TAG,
                           "Publish RequestLicenseInfo");
#endif
}

void TelemetryTask::ReportStreamPortsStatus() {
#ifdef CONFIG_ENABLE_MQTT
  StreamPortStatus data = {};
  for (int i = 0; i < 8; i++) {
    memset(&data.ports[i], 0, sizeof(PortStatusData));
  }

  PortManager &pm = allocator_.GetPortManager();
  data.header = {
      .service = TelemetryServiceCommand::STREAM_PORTS_STATUS,
      .message_id = GetMessageId(),
  };
  data.status = 0;
  data.port_status_map = pm.GetPortsOpenStatus();
  size_t i = 0;
  for (const Port &port : pm) {
    PowerFeatures features;
    PortDetails details;

    port.GetPowerFeatures(&features);
    port.GetDetails(&details);

    data.ports[i].features = features;
    data.ports[i].details = details;
    data.ports[i].charging_minutes = port.GetChargingDurationSeconds() / 60;
    i++;
  }
  std::vector<uint8_t> payload = data.serialize();
  ESP_LOG_BUFFER_HEXDUMP(TAG, payload.data(), payload.size(), ESP_LOG_DEBUG);
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(payload), TAG,
                           "ReportStreamPortsStatus");
#endif
}

void TelemetryTask::ReportStreamDeviceStatus() {
#ifdef CONFIG_ENABLE_MQTT
  StreamDeviceStatus data = {};
  data.header = {
      .service = TelemetryServiceCommand::STREAM_DEVICE_STATUS,
      .message_id = GetMessageId(),
  };
  data.status = 0;
  data.allocator = static_cast<uint8_t>(allocator_.Type());
  if (allocator_.Type() == PowerAllocatorType::STATIC_ALLOCATOR) {
    auto strategy = allocator_.GetStrategyAs<PowerStaticChargingStrategy>();
    if (strategy) {
      strategy->GetIdentifier(&data.power_allocator_param);
    }
  }
  data.display_config.mode = controller_.get_display_mode();
  DisplayNVSGetOrDefault(&data.display_config.flip, NVSKey::DISPLAY_FLIP);
  data.display_config.intensity = controller_.get_display_intensity();
  data.device_uptime = controller_.get_uptime() / 1e6;

  bool standby_state = controller_.is_power_on();
  data.feature_flag.flags.device_switch = standby_state;

  data.feature_flag.flags.ble_state = ble_is_advertising();

  bool enable_report = true;
  SyslogNVSGetOrDefault(&enable_report, NVSKey::SYSLOG_REPORT_STATE);
  data.feature_flag.flags.syslog_state = enable_report;

  data.charging_minutes =
      allocator_.GetPortManager().GetChargingDurationSeconds() / 60;
  data.temperature_mode = allocator_.GetTemperatureMode();

  std::vector<uint8_t> payload = data.serialize();
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(payload), TAG,
                           "ReportStreamDeviceStatus");
#endif
}

void TelemetryTask::ReportStreamPortPDStatus() {
#ifdef CONFIG_ENABLE_MQTT
  PortManager &pm = allocator_.GetPortManager();
  for (const Port &port : pm) {
    if (!port.Attached()) {
      continue;
    }

    StreamPortPDStatus data = {};

    data.header = {
        .service = TelemetryServiceCommand::STREAM_PORT_PD_STATUS,
        .message_id = GetMessageId(),
    };
    data.port_index = port.Id();
    port.GetPDStatus(&data.pd_status);
    data.status = 0;

    std::vector<uint8_t> payload = data.serialize();
    ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(payload), TAG,
                             "ReportStreamPortPDStatus");
  }
#endif
}

void TelemetryTask::RequestShutdownBle() {
#ifdef CONFIG_ENABLE_MQTT
  ESP_LOGI(TAG, "Requesting shutdown ble");
  RequestShutdownBleInfo data;
  data.header = {
      .service = TelemetryServiceCommand::REQUEST_SHUTDOWN_BLE,
      .message_id = GetMessageId(),
  };
  std::vector<uint8_t> payload = data.serialize();
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(payload), TAG,
                           "RequestShutdownBle");
#endif
}

void TelemetryTask::ReportUpgradeStart() {
#ifdef CONFIG_MCU_MODEL_SW3566
  ReportSW3566UpgradeInfo(Version(MachineInfo::GetInstance().GetMCUVersion()),
                          UpgradeStatus::UPGRADE_START,
                          UpgradeFailureReason::REASON_NONE, ESP_OK);
  ReportFPGAUpgradeInfo(Version(MachineInfo::GetInstance().GetFPGAVersion()),
                        UpgradeStatus::UPGRADE_START,
                        UpgradeFailureReason::REASON_NONE, ESP_OK);
#endif
  ReportESP32UpgradeInfo(Version(MachineInfo::GetInstance().GetESP32Version()),
                         UpgradeStatus::UPGRADE_START,
                         UpgradeFailureReason::REASON_NONE, ESP_OK);
}

void TelemetryTask::ReportUpgradeError(esp_err_t err) {
#ifdef CONFIG_MCU_MODEL_SW3566
  ReportSW3566UpgradeError(Version(MachineInfo::GetInstance().GetMCUVersion()),
                           err);
  ReportFPGAUpgradeError(Version(MachineInfo::GetInstance().GetFPGAVersion()),
                         err);
#endif
  ReportESP32UpgradeError(Version(MachineInfo::GetInstance().GetESP32Version()),
                          err);
}

static void ReportStreamPortsStatusTimer(void *arg) {
  CHECK_MQTT_CONNECTED();
  TelemetryTask *task = TelemetryTask::GetInstance();
  if (task == nullptr || !task->IsTelemetryStreamSubscribed()) {
    return;
  }
  task->ReportStreamPortsStatus();
}

static void ReportStreamDeviceStatusTimer(void *arg) {
  if (!MQTTClient::GetInstance()->Connected()) {
    return;
  }
  TelemetryTask *task = TelemetryTask::GetInstance();
  if (task == nullptr || !task->IsTelemetryStreamSubscribed()) {
    return;
  }
  task->ReportStreamDeviceStatus();
}

static void ReportStreamPortPDStatusTimer(void *arg) {
  CHECK_MQTT_CONNECTED();

  TelemetryTask *task = TelemetryTask::GetInstance();
  if (task == nullptr || !task->IsTelemetryStreamSubscribed()) {
    return;
  }
  task->ReportStreamPortPDStatus();
}

void TelemetryTask::SubscribeTelemetryStream() {
  telemetry_stream_subscribed_at_ = esp_timer_get_time();
}

void TelemetryTask::UnsubscribeTelemetryStream() {
  telemetry_stream_subscribed_at_ = -1;
}

bool TelemetryTask::IsTelemetryStreamSubscribed() {
  if (telemetry_stream_subscribed_at_ <= 0) {
    return false;
  }
  bool expired = esp_timer_get_time() - telemetry_stream_subscribed_at_ >
                 telemetry_stream_subscribe_timeout_us_;
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

void TelemetryTask::ReportPowerAllocationData(
    uint16_t power_budget, uint8_t temperature, uint16_t remaining_power,
    uint16_t adc_value, const PortPowerAllocation port_power_allocations[8]) {
#if CONFIG_MCU_MODEL_FAKE_SW3566
  return;
#endif
  CHECK_MQTT_CONNECTED();
  PowerAllocationData data;
  data.header = {
      .service = TelemetryServiceCommand::POWER_ALLOCATION_DATA,
      .message_id = GetMessageId(),
  };
  data.power_budget = power_budget;
  data.remaining_power = remaining_power;
  data.adc_value = adc_value;
  data.temperature = temperature;
  data.unused = 0;
  for (int i = 0; i < 8; i++) {
    data.port_power_allocations[i] = port_power_allocations[i];
  }
  std::vector<uint8_t> payload = data.serialize();
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(payload), TAG,
                           "ReportPowerAllocationData");
}

typedef void (*TimerFunc)(void *);

typedef struct {
  TimerFunc func;
  uint16_t interval;
} Timer;

static Timer timers[MAX_TASK_COUNT] = {
#if !CONFIG_MCU_MODEL_FAKE_SW3566
    {ReportPowerStatsTask, CONFIG_TELEMETRY_INTERVAL},
#if defined(CONFIG_TELEMETRY_SYSTEM_STATE_ENABLED) && \
    defined(CONFIG_INTERFACE_TYPE_UART)
    {ReportUartMetricsTask, CONFIG_UART_METRICS_INTERVAL},
#endif
#endif
#ifdef CONFIG_TELEMETRY_SYSTEM_STATE_ENABLED
    {ReportDeviceTelemetryTask, CONFIG_TELEMETRY_INTERVAL},
#endif
    {ReportStreamPortsStatusTimer, CONFIG_STREAM_PORTS_STATUS_INTERVAL_MS},
    {ReportStreamDeviceStatusTimer, CONFIG_STREAM_DEVICE_STATUS_INTERVAL_MS},
    {ReportStreamPortPDStatusTimer, CONFIG_STREAM_PD_STATUS_INTERVAL_MS},
};

static void AllInOneTimerTask(void *telemtry_task) {
  if (telemtry_task == nullptr) {
    return;
  }
  TelemetryTask &task = *(TelemetryTask *)telemtry_task;
  if (task.IsPaused()) {
    return;
  }

  all_in_one_task_interval_counter++;
  uint16_t max_interval = 0;
  for (int i = 0; i < MAX_TASK_COUNT; i++) {
    if (timers[i].func == nullptr) {
      continue;
    }
    if (timers[i].interval > max_interval) {
      max_interval = timers[i].interval;
    }
    if (all_in_one_task_interval_counter %
            (timers[i].interval / TASK_INTERVAL_MS) ==
        0) {
      timers[i].func(telemtry_task);
    }
  }

  if (all_in_one_task_interval_counter >= max_interval / TASK_INTERVAL_MS) {
    all_in_one_task_interval_counter = 0;
  }
}
