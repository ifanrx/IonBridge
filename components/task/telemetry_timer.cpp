#include "telemetry_timer.h"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "ble.h"
#include "controller.h"
#include "display_manager.h"
#include "esp_check.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "ionbridge.h"
#include "mqtt_app.h"
#include "mqtt_message.h"
#include "nvs_default.h"
#include "nvs_namespace.h"
#include "port.h"
#include "port_data.h"
#include "port_manager.h"
#include "power_allocator.h"
#include "sdkconfig.h"
#include "telemetry_task.h"
#include "wifi.h"
#include "wifi_manager.h"

#ifdef CONFIG_INTERFACE_TYPE_UART
#include "uart.h"
#endif

static const char *TAG = "TelemetryTimer";

#define MAX_TASK_COUNT 10
#define TASK_INTERVAL_MS 100

#define POWER_MQTT_REPORT_SECONDS 60
#define POWER_MQTT_REPORT_SECONDS_DEBUG 10

#ifdef CONFIG_TELEMETRY_SYSTEM_STATE_ENABLED
static void ReportDeviceMemory();
static void ReportWIFIStatsData();
#endif

#ifndef CONFIG_MCU_MODEL_FAKE_SW3566
#ifdef CONFIG_TELEMETRY_ENABLED
static void ReportPowerStats();
#endif

#if defined(CONFIG_TELEMETRY_SYSTEM_STATE_ENABLED) && \
    defined(CONFIG_INTERFACE_TYPE_UART)
static void ReportUartMetrics();
#endif
#endif

#ifdef CONFIG_ENABLE_TEMP_MONITOR
static void ReportOverTemperatureAlert();
#endif

static void ReportStreamPortsStatus();
static void ReportStreamDeviceStatus();
static void ReportStreamPortPDStatus();
static void ReportPowerAllocationData();

// Helper function that returns a lambda with common logic
auto MakeTelemetryReporter = [](auto reportMethod) {
  return [reportMethod]() {
    CHECK_MQTT_CONNECTED();
    TelemetryTask &task = TelemetryTask::GetInstance();
    if (!task.IsTelemetryStreamSubscribed()) {
      return;
    }
    (reportMethod)();
  };
};

static std::array<TelemetryTimer, MAX_TASK_COUNT> s_telemetryTimers = {
    TelemetryTimer(MakeTelemetryReporter(ReportStreamPortsStatus),
                   CONFIG_STREAM_PORTS_STATUS_INTERVAL_MS),
    TelemetryTimer(MakeTelemetryReporter(ReportStreamDeviceStatus),
                   CONFIG_STREAM_DEVICE_STATUS_INTERVAL_MS),
    TelemetryTimer(MakeTelemetryReporter(ReportStreamPortPDStatus),
                   CONFIG_STREAM_PD_STATUS_INTERVAL_MS),

#ifdef CONFIG_TELEMETRY_SYSTEM_STATE_ENABLED
    TelemetryTimer(ReportDeviceMemory, CONFIG_TELEMETRY_INTERVAL),
    TelemetryTimer(ReportWIFIStatsData, CONFIG_TELEMETRY_INTERVAL),
#endif

#ifndef CONFIG_MCU_MODEL_FAKE_SW3566
#ifdef CONFIG_TELEMETRY_ENABLED
    TelemetryTimer(ReportPowerStats, CONFIG_TELEMETRY_INTERVAL),
#endif

#if defined(CONFIG_TELEMETRY_SYSTEM_STATE_ENABLED) && \
    defined(CONFIG_INTERFACE_TYPE_UART)
    TelemetryTimer(ReportUartMetrics, CONFIG_UART_METRICS_INTERVAL),
#endif
#endif

#ifdef CONFIG_ENABLE_TEMP_MONITOR
    TelemetryTimer(ReportOverTemperatureAlert,
                   CONFIG_OVER_TEMP_ALERT_INTERVAL_MS),
#endif

    TelemetryTimer(ReportPowerAllocationData,
                   POWER_MQTT_REPORT_SECONDS_DEBUG * 1e3),
};

#ifdef CONFIG_TELEMETRY_SYSTEM_STATE_ENABLED
void ReportDeviceMemory() {
  static uint8_t initial_boot = 1;
  CHECK_MQTT_CONNECTED();
  uint32_t freeHeapSize = esp_get_free_heap_size();
  uint32_t freeInternalHeapSize = esp_get_free_internal_heap_size();
  uint32_t minimumFreeHeapSize = esp_get_minimum_free_heap_size();

  DeviceMemoryInfo data;
  data.initialBoot = initial_boot;
  data.freeHeapSize = freeHeapSize;
  data.freeInternalHeapSize = freeInternalHeapSize;
  data.minimumFreeHeapSize = minimumFreeHeapSize;
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(data.Serialize()),
                           TAG, "ReportDeviceMemoryTask");
  initial_boot = 0;
}

void ReportWIFIStatsData() {
  CHECK_MQTT_CONNECTED();
  int rssi;
  ESP_RETURN_VOID_ON_ERROR(esp_wifi_sta_get_rssi(&rssi), TAG,
                           "esp_wifi_sta_get_rssi");
  ssid_detail_t ssid_detail;
  ESP_ERROR_COMPLAIN(get_connected_ssid_detail(&ssid_detail),
                     "get_connected_ssid_detail");

  WIFIStatsData data;
  WiFiManager &wm = WiFiManager::GetInstance();
  MQTTClient *client = MQTTClient::GetInstance();
  data.associationCount = wm.GetAssociationCount();
  data.disassociationCount = wm.GetDisassociationCount();
  data.rssi = rssi;
  data.channel = ssid_detail.channel;
  data.wifiConnectionTime = wm.GetWifiConnectionTime();
  data.mqttConnectionTime = client->GetConnectionTime();
  data.connectivityFailureCount = wm.GetConnectivityFailureCount();
  data.dnsResolutionFailureCount = wm.GetDNSResolutionFailureCount();
  data.mqttConnectionCount = client->GetConnectionCount();
  data.mqttMessageTxCount = client->GetMessageTxCount();
  data.mqttMessageRxCount = client->GetMessageRxCount();

  ESP_RETURN_VOID_ON_ERROR(client->Publish(data.Serialize()), TAG,
                           "ReportWIFIStatsDataTask");
}
#endif

#ifndef CONFIG_MCU_MODEL_FAKE_SW3566
#ifdef CONFIG_TELEMETRY_ENABLED
void ReportPowerStats() {
  CHECK_MQTT_CONNECTED();
  PortManager &pm = PortManager::GetInstance();
  MQTTClient *client = MQTTClient::GetInstance();
  // 10s sample interval
  size_t n = CONFIG_TELEMETRY_INTERVAL / 10000;

  AggregatedPowerHistoricalData aggregatedData;

  for (const Port &port : pm) {
    std::vector<PortStatsData> stats_data =
        port.GetHistoricalStats().GetLastNPoints(n);

    if (!stats_data.empty()) {
      aggregatedData.data.push_back(
          {.port = port.Id(),
           .length = static_cast<uint16_t>(stats_data.size()),
           .data = std::move(stats_data)});
    }

    PowerConsumptionData consumptionData;
    consumptionData.port = port.Id();
    consumptionData.length = 1;
    consumptionData.power_consumption = 12;
    ESP_ERROR_COMPLAIN(client->Publish(consumptionData.Serialize()),
                       "publish power consumption");
  }

  if (!aggregatedData.data.empty()) {
    ESP_ERROR_COMPLAIN(client->Publish(aggregatedData.Serialize()),
                       "publish aggregated power stats");
  }
}
#endif

#if defined(CONFIG_TELEMETRY_SYSTEM_STATE_ENABLED) && \
    defined(CONFIG_INTERFACE_TYPE_UART)
void ReportUartMetrics() {
  CHECK_MQTT_CONNECTED();
  UartMetricsData data;
  uart_metrics_t *metrics = uart_get_metrics();
  data.reset_state_count = metrics->reset_state_count;
  data.resend_count = metrics->resend_count;
  data.sent_failed_count = metrics->sent_failed_count;
  data.sent_count = metrics->sent_count;
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(data.Serialize()),
                           TAG, "PublishUartMetrics");
}
#endif
#endif

#ifdef CONFIG_ENABLE_TEMP_MONITOR
void ReportOverTemperatureAlert() {
  static bool temperature_exceeded = false;
  float temperature = 0;

  if (temperature >= CONFIG_TEMPERATURE_ALERT_THRESHOLD &&
      !temperature_exceeded) {
    CHECK_MQTT_CONNECTED();
    OverTemperatureAlertInfo data;
    data.temperature = temperature;
    data.thresholdTemperature = CONFIG_TEMPERATURE_ALERT_THRESHOLD;
    ESP_LOGW(TAG, "Temperature exceeds threshold: %d, current temperature: %f",
             CONFIG_TEMPERATURE_ALERT_THRESHOLD, temperature);
    ESP_ERROR_COMPLAIN(MQTTClient::GetInstance()->Publish(data.Serialize(), 1),
                       "Failed to publish OverTemperatureAlertInfo");
  }
  temperature_exceeded = temperature >= CONFIG_TEMPERATURE_ALERT_THRESHOLD;
}
#endif

void ReportStreamPortsStatus() {
  CHECK_MQTT_CONNECTED();
  StreamPortStatus data = {};
  for (int i = 0; i < 8; i++) {
    memset(&data.ports[i], 0, sizeof(PortStatusData));
  }

  PortManager &pm = PortManager::GetInstance();
  data.status = 0;
  data.port_status_map = pm.GetOpenPortsBitMask();
  for (const Port &port : pm) {
    PowerFeatures features;
    PortDetails details;
    size_t i = port.Id();

    port.GetPowerFeatures(&features);
    port.GetDetails(&details);

    data.ports[i].features = features;
    data.ports[i].details = details;
    data.ports[i].charging_minutes = port.GetChargingDurationSeconds() / 60;
    data.ports[i].port_type = port.GetPortType();
  }
  std::vector<uint8_t> payload = data.Serialize();
  ESP_LOG_BUFFER_HEXDUMP(TAG, payload.data(), payload.size(), ESP_LOG_DEBUG);
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(payload), TAG,
                           "ReportStreamPortsStatus");
}

void ReportStreamDeviceStatus() {
  CHECK_MQTT_CONNECTED();
  PowerAllocator &allocator = PowerAllocator::GetInstance();
  PortManager &pm = PortManager::GetInstance();
  DeviceController &controller = DeviceController::GetInstance();
  DisplayManager &dm = DisplayManager::GetInstance();

  StreamDeviceStatus data = {};
  data.status = 0;
  data.allocator = static_cast<uint8_t>(allocator.Type());
  if (allocator.Type() == PowerAllocatorType::STATIC_ALLOCATOR) {
    auto strategy = allocator.GetStrategyAs<PowerStaticChargingStrategy>();
    if (strategy) {
      strategy->GetIdentifier(&data.power_allocator_param);
    }
  }
  data.display_config.mode = dm.GetDisplayMode();
  DisplayNVSGetOrDefault(&data.display_config.flip, NVSKey::DISPLAY_FLIP);
  data.display_config.intensity = dm.GetDisplayIntensity();
  data.device_uptime = esp_timer_get_time() / 1e6;

  bool standby_state = controller.is_power_on();
  data.feature_flag.flags.device_switch = standby_state;

  data.feature_flag.flags.ble_state = ble_is_advertising();

  bool enable_report = true;
  SyslogNVSGetOrDefault(&enable_report, NVSKey::SYSLOG_REPORT_STATE);
  data.feature_flag.flags.syslog_state = enable_report;

  data.charging_minutes = pm.GetChargingDurationSeconds() / 60;
  data.temperature_mode = allocator.GetTemperatureMode();

  std::vector<uint8_t> payload = data.Serialize();
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(payload), TAG,
                           "ReportStreamDeviceStatus");
}

void ReportStreamPortPDStatus() {
  CHECK_MQTT_CONNECTED();
  PortManager &pm = PortManager::GetInstance();
  for (const Port &port : pm) {
    if (!port.Attached()) {
      continue;
    }

    StreamPortPDStatus data = {};

    data.port_index = port.Id();
    port.GetPDStatus(&data.pd_status);
    data.status = 0;

    std::vector<uint8_t> payload = data.Serialize();
    ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(payload), TAG,
                             "ReportStreamPortPDStatus");
  }
}

#if CONFIG_MCU_MODEL_FAKE_SW3566
void ReportPowerAllocationData() {}
#else
void ReportPowerAllocationData() {
  static int64_t last_reported_at = 0;
  int64_t expected_report_us;
  int64_t now = esp_timer_get_time();
  TelemetryTask &task = TelemetryTask::GetInstance();
  PowerAllocator &allocator = PowerAllocator::GetInstance();

  if (task.IsDebug()) {
    expected_report_us = POWER_MQTT_REPORT_SECONDS_DEBUG * 1e6;
  } else {
    expected_report_us = POWER_MQTT_REPORT_SECONDS * 1e6;
  }

  if (now - last_reported_at < expected_report_us) {
    return;
  }
  last_reported_at = now;

  uint16_t power_budget, remaining_power;
  allocator.GetAllocationData(&power_budget, &remaining_power);
  task.ReportPowerAllocationData(power_budget, remaining_power);
}
#endif

void TelemetryTimerTask(void *) {
  if (TelemetryTask::GetInstance().IsPaused()) {
    return;
  }
  auto now = std::chrono::steady_clock::now();
  for (auto &timer : s_telemetryTimers) {
    timer.Run(now);
  }
}

void StartTelemetryTimerTask() {
  esp_timer_handle_t timer_handle;
  esp_timer_create_args_t timer_args = {
      .callback = TelemetryTimerTask,
      .arg = nullptr,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "TelemetryTimerTask",
      .skip_unhandled_events = true,
  };
  const uint64_t timer_interval_us = ((uint64_t)TASK_INTERVAL_MS) * 1000;
  ESP_RETURN_VOID_ON_ERROR(esp_timer_create(&timer_args, &timer_handle), TAG,
                           "Failed to create timer for TelemetryTimerTask");
  ESP_RETURN_VOID_ON_ERROR(
      esp_timer_start_periodic(timer_handle, timer_interval_us), TAG,
      "Failed to start timer for TelemetryTimerTask");
}
