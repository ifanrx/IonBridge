#include "telemetry_timer.h"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "ble.h"
#include "command.pb.h"
#include "controller.h"
#include "data_types.h"
#include "data_types.pb.h"
#include "display_manager.h"
#include "esp_check.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "ionbridge.h"
#include "mqtt_app.h"
#include "nvs_key.h"
#include "nvs_namespace.h"
#include "pb_encode.h"
#include "pb_utils.h"
#include "port.h"
#include "port_manager.h"
#include "power_allocator.h"
#include "sdkconfig.h"
#include "service.h"

#ifdef CONFIG_ENABLE_PROGRAMMER
#include "firmware_programmer.h"
#endif
#include "telemetry.pb.h"
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
static void ReportDeviceStatsData();
#endif

#ifdef CONFIG_TELEMETRY_ENABLED
static void ReportPowerStats();
#endif

#ifndef CONFIG_MCU_MODEL_FAKE_SW3566
#if defined(CONFIG_TELEMETRY_SYSTEM_STATE_ENABLED) && \
    defined(CONFIG_INTERFACE_TYPE_UART)
static void ReportUartMetrics();
#endif
#endif


static void ReportStreamPortsStatus();
static void ReportStreamDeviceStatus();
static void ReportStreamPortPDStatus();
static void ReportPowerAllocationData();

static uint32_t kMessageID = 0;

uint32_t GetMessageID() { return kMessageID++; }

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
    TelemetryTimer(ReportDeviceStatsData, CONFIG_TELEMETRY_INTERVAL),
#endif

#ifdef CONFIG_TELEMETRY_ENABLED
    TelemetryTimer(ReportPowerStats, CONFIG_TELEMETRY_INTERVAL, true),
#endif

#ifndef CONFIG_MCU_MODEL_FAKE_SW3566
#if defined(CONFIG_TELEMETRY_SYSTEM_STATE_ENABLED) && \
    defined(CONFIG_INTERFACE_TYPE_UART)
    TelemetryTimer(ReportUartMetrics, CONFIG_UART_METRICS_INTERVAL),
#endif
#endif


    TelemetryTimer(ReportPowerAllocationData,
                   POWER_MQTT_REPORT_SECONDS_DEBUG * 1e3),
};

bool TelemetryTimer::ShouldRun(steady_clock::time_point now) const {
  if (!runnable) {
    return false;
  }
  if (now < next_trigger) {
    return false;
  }
  if (power_on_only && !DeviceController::GetInstance().is_power_on()) {
    return false;
  }
  return true;
}

#ifdef CONFIG_TELEMETRY_SYSTEM_STATE_ENABLED
void ReportDeviceStatsData() {
  static bool initial_boot = true;
  CHECK_MQTT_CONNECTED();
  pb_DeviceStatsData data = pb_DeviceStatsData_init_zero;
  data.initial_boot = initial_boot;
  data.free_heap_size = esp_get_free_heap_size();
  data.free_internal_heap_size = esp_get_free_internal_heap_size();
  data.minimum_free_heap_size = esp_get_minimum_free_heap_size();

  int rssi = 0;
  ESP_RETURN_VOID_ON_ERROR(esp_wifi_sta_get_rssi(&rssi), TAG,
                           "esp_wifi_sta_get_rssi");
  ssid_detail_t ssid_detail = {};
  ESP_ERROR_COMPLAIN(get_connected_ssid_detail(&ssid_detail),
                     "get_connected_ssid_detail");
  WiFiManager &wm = WiFiManager::GetInstance();
  data.wifi_association_count = wm.GetAssociationCount();
  data.wifi_disassociation_count = wm.GetDisassociationCount();
  data.wifi_rssi = rssi;
  data.wifi_channel = ssid_detail.channel;
  data.wifi_connection_time = wm.GetWifiConnectionTime();
  data.mqtt_connection_time = MQTTClient::GetInstance()->GetConnectionTime();
  data.mqtt_connection_count = MQTTClient::GetInstance()->GetConnectionCount();
  data.mqtt_message_tx_count = MQTTClient::GetInstance()->GetMessageTxCount();
  data.mqtt_message_rx_count = MQTTClient::GetInstance()->GetMessageRxCount();
  data.connectivity_failure_count = wm.GetConnectivityFailureCount();
  data.dns_resolution_failure_count = wm.GetDNSResolutionFailureCount();

  data.temperature = 0.0f;

  pb_TelemetryData payload = pb_TelemetryData_init_zero;
  payload.which_data = pb_TelemetryData_device_stats_data_tag;
  payload.data.device_stats_data = data;
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(
                               TelemetryCommand::DEVICE_STATS_DATA, payload),
                           TAG, "ReportDeviceStatsData");
  initial_boot = false;
}
#endif

#ifdef CONFIG_TELEMETRY_ENABLED

#ifdef CONFIG_MCU_MODEL_SW3566
static uint64_t ApplyChargeConversionFactor(uint64_t session_charge) {
  uint64_t max_safe_value =
      UINT64_MAX / (uint64_t)SW3566_CHARGE_CONVERSION_FACTOR;
  if (session_charge > max_safe_value) {
    return UINT64_MAX;
  }
  return session_charge * SW3566_CHARGE_CONVERSION_FACTOR;
}
#endif

bool port_stats_data_pb_encode_callback(pb_ostream_t *stream,
                                        const pb_field_t *field,
                                        void *const *arg) {
  std::vector<PortDetails> *stats_data =
      static_cast<std::vector<PortDetails> *>(*arg);
  for (const PortDetails &data : *stats_data) {
    if (!pb_encode_tag_for_field(stream, field)) {
      return false;
    }
    pb_PortStatsData pb_data = pb_PortStatsData_init_zero;
    pb_data.voltage = data.vout_value;
    pb_data.amperage = data.iout_value;
    pb_data.temperature = data.die_temperature;
    pb_data.vin_value = data.vin_value;
    pb_data.session_id = data.session_id;
#ifdef CONFIG_MCU_MODEL_SW3566
    pb_data.session_charge = ApplyChargeConversionFactor(data.session_charge);
#else
    pb_data.session_charge = data.session_charge;
#endif
    pb_data.manufacturer_vid = data.manufacturer_vid;
    pb_data.manufacturer_pid = data.manufacturer_pid;
    pb_data.battery_vid = data.battery_vid;
    pb_data.battery_pid = data.battery_pid;
    if (!pb_encode_submessage(stream, pb_PortStatsData_fields, &pb_data)) {
      return false;
    }
  }
  return true;
}

void ReportPowerStats() {
  CHECK_MQTT_CONNECTED();
  PortManager &pm = PortManager::GetInstance();
  MQTTClient *client = MQTTClient::GetInstance();
  // 10s sample interval
  size_t n = CONFIG_TELEMETRY_INTERVAL / 10000;

  pb_PowerHistoricalData data = pb_PowerHistoricalData_init_zero;
  int data_count = 0;
  static std::vector<std::vector<PortDetails>> stats_data_list;
  stats_data_list.reserve(pm.PortCount());
  for (const Port &port : pm) {
    std::vector<PortDetails> stats_data =
        port.GetHistoricalDetails().GetLastNPoints(n);
    if (stats_data.empty()) {
      continue;
    }
    stats_data_list.emplace_back(std::move(stats_data));
    std::vector<PortDetails> &stats_data_ref = stats_data_list.back();

    data.data[data_count].port = port.Id();
    data.data[data_count].stats.arg = &stats_data_ref;
    data.data[data_count].stats.funcs.encode =
        port_stats_data_pb_encode_callback;
    data_count++;
  }
  data.data_count = data_count;
  if (data.data_count > 0) {
    pb_TelemetryData payload = pb_TelemetryData_init_zero;
    payload.which_data = pb_TelemetryData_power_historical_data_tag;
    payload.data.power_historical_data = data;
    ESP_ERROR_COMPLAIN(
        client->Publish(TelemetryCommand::POWER_HISTORICAL_DATA, payload),
        "publish power stats");
  }
  stats_data_list.clear();
}
#endif

#ifndef CONFIG_MCU_MODEL_FAKE_SW3566
#if defined(CONFIG_TELEMETRY_SYSTEM_STATE_ENABLED) && \
    defined(CONFIG_INTERFACE_TYPE_UART)
void ReportUartMetrics() {
  CHECK_MQTT_CONNECTED();
  uart_metrics_t *metrics = uart_get_metrics();
  pb_UartMetricsData data = pb_UartMetricsData_init_zero;
  data.reset_state_count = metrics->reset_state_count;
  data.resend_count = metrics->resend_count;
  data.sent_failed_count = metrics->sent_failed_count;
  data.sent_count = metrics->sent_count;

  pb_TelemetryData payload = pb_TelemetryData_init_zero;
  payload.which_data = pb_TelemetryData_uart_metrics_data_tag;
  payload.data.uart_metrics_data = data;
  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(
                               TelemetryCommand::UART_METRICS_DATA, payload),
                           TAG, "ReportUartMetrics");
}
#endif
#endif


void ReportStreamPortsStatus() {
  CHECK_MQTT_CONNECTED();
  PortManager &pm = PortManager::GetInstance();

  pb_CommandResponse payload = pb_CommandResponse_init_zero;
  payload.id = GetMessageID();
  payload.status = pb_CommandStatus_SUCCESS;
  payload.which_payload = pb_CommandResponse_stream_port_status_tag;

  pb_StreamPortStatus &data = payload.payload.stream_port_status;
  data.port_status_map = pm.GetOpenPortsBitMask();
  data.ports_count = 0;

  for (const Port &port : pm) {
    PowerFeatures features;
    PortDetails details;

    port.GetPowerFeatures(&features);
    port.GetDetails(&details);

    pb_PortStatusData &port_data = data.ports[data.ports_count++];
    port_data.charging_minutes = port.GetChargingDurationSeconds() / 60;
    PowerFeaturesToPB(port_data.features, features);

#ifdef CONFIG_MCU_MODEL_SW3566
    details.session_charge =
        ApplyChargeConversionFactor(details.session_charge);
#endif
    std::memcpy(&port_data.details, &details, sizeof(pb_PortDetails));
    port_data.port_type = static_cast<pb_PortType>(port.GetPortType());
    port_data.custom_pdo_voltage_mv = port.GetCustomPdoVoltage();

    // Get cable compensation parameters
    bool cable_compensation_enable;
    uint8_t resistance_mohm, voltage_offset_level;
    port.GetCableCompensation(cable_compensation_enable, resistance_mohm,
                              voltage_offset_level);

    port_data.cable_compensation_enable = cable_compensation_enable;
    // Convert hardware resistance to proto resistance
    // Hardware: 0=100mΩ, 1=65mΩ -> Proto: 0=65mΩ, 1=100mΩ
    port_data.cable_resistance =
        static_cast<pb_CableCompensation>(resistance_mohm ? 0 : 1);
    port_data.cable_voltage_offset =
        static_cast<pb_FixedVoltageOffset>(voltage_offset_level);
  }

  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(
                               ServiceCommand::STREAM_PORT_STATUS, payload),
                           TAG, "ReportStreamPortsStatus");
}

void ReportStreamDeviceStatus() {
  using nvs_namespace::user_data::DISPLAY_DATA;
  using nvs_namespace::user_data::SYSLOG;

  CHECK_MQTT_CONNECTED();
  PowerAllocator &allocator = PowerAllocator::GetInstance();
  PortManager &pm = PortManager::GetInstance();
  DeviceController &controller = DeviceController::GetInstance();
  DisplayManager &dm = DisplayManager::GetInstance();
#ifdef CONFIG_ENABLE_PROGRAMMER
  FirmwareProgrammer &prog_mgr = FirmwareProgrammer::GetInstance();
#endif

  pb_CommandResponse payload = pb_CommandResponse_init_zero;
  payload.id = GetMessageID();
  payload.status = pb_CommandStatus_SUCCESS;
  payload.which_payload = pb_CommandResponse_stream_device_status_tag;

  pb_StreamDeviceStatus &data = payload.payload.stream_device_status;
  data.allocator = static_cast<uint8_t>(allocator.Type());
  if (allocator.Type() == PowerAllocatorType::STATIC_ALLOCATOR) {
    auto strategy = allocator.GetStrategyAs<PowerStaticChargingStrategy>();
    if (strategy) {
      data.power_allocator_param = strategy->GetIdentifier();
    }
  }

  data.display_config.mode = static_cast<pb_DisplayMode>(dm.GetMode());

  // Convert rotation to flip for legacy protobuf compatibility
  uint8_t rotation = dm.GetRotation();
  uint8_t flip_mode = (rotation == 180) ? 1 : 0;
  data.display_config.flip = static_cast<pb_DisplayFlip>(flip_mode);

  data.display_config.intensity = dm.GetIntensity();
  data.display_config.rotation = static_cast<pb_DisplayRotation>(rotation);
  data.device_uptime = esp_timer_get_time() / 1e6;

  data.feature_flag.device_power_on = controller.is_power_on();
  data.feature_flag.ble_on = ble_is_advertising();

  SYSLOG.GetOrDefault(nvs_keys::SYSLOG_REPORT_STATE,
                      data.feature_flag.syslog_on);

  data.charging_minutes = pm.GetChargingDurationSeconds() / 60;
  data.temperature_mode =
      static_cast<pb_TemperatureMode>(allocator.GetTemperatureMode());
#ifdef CONFIG_ENABLE_PROGRAMMER
  data.programmer_mode = prog_mgr.IsActivated();
#else
  data.programmer_mode = false;
#endif

  ESP_RETURN_VOID_ON_ERROR(MQTTClient::GetInstance()->Publish(
                               ServiceCommand::STREAM_DEVICE_STATUS, payload),
                           TAG, "ReportStreamDeviceStatus");
}

void ReportStreamPortPDStatus() {
  CHECK_MQTT_CONNECTED();

  PortManager &pm = PortManager::GetInstance();
  for (const Port &port : pm) {
    if (!port.Attached()) {
      continue;
    }
    pb_CommandResponse payload = pb_CommandResponse_init_zero;
    payload.id = GetMessageID();
    payload.status = pb_CommandStatus_SUCCESS;
    payload.which_payload = pb_CommandResponse_stream_port_pd_status_tag;

    pb_StreamPortPDStatus &data = payload.payload.stream_port_pd_status;
    data.port = port.Id();

    ClientPDStatus pd_status = {};
    port.GetPDStatus(&pd_status);
    pb_ClientPDStatus &pb_pd_status = data.pd_status;
    ClientPDStatusToPB(pb_pd_status, pd_status);
    ESP_RETURN_VOID_ON_ERROR(
        MQTTClient::GetInstance()->Publish(
            ServiceCommand::STREAM_PORT_PD_STATUS, payload),
        TAG, "ReportStreamPortPDStatus");
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
