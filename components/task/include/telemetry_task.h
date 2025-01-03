#ifndef TELEMETRYTASK_H_
#define TELEMETRYTASK_H_

#include <cstdint>
#include <memory>
#include <mutex>

#include "controller.h"
#include "esp_err.h"
#include "mqtt_message.h"
#include "power_allocator.h"
#include "sdkconfig.h"
#include "version.h"

#define CHECK_MQTT_CONNECTED()                                     \
  ({                                                               \
    if (!MQTTClient::GetInstance()->Connected()) {                 \
      ESP_LOGD(TAG, "MQTT client is not started yet, skipping %s", \
               __FUNCTION__);                                      \
      return;                                                      \
    }                                                              \
  })

class TelemetryTask {
  // Singleton
  static std::unique_ptr<TelemetryTask> instance_;
  static std::mutex mutex_;

  // Member variables
  PowerAllocator &allocator_;
  DeviceController &controller_;
  volatile int64_t telemetry_stream_subscribed_at_ = -1;
  volatile bool paused = false;
  int64_t telemetry_stream_subscribe_timeout_us_ = 2 * 60 * 1e6;

  // Private constructor
  TelemetryTask(PowerAllocator &powerAllocator,
                DeviceController &espController);

 public:
  // Delete copy constructor and assignment operator
  TelemetryTask(TelemetryTask const &) = delete;
  TelemetryTask &operator=(TelemetryTask const &) = delete;

  // Static method to initialize the singleton instance
  static bool Initialize(PowerAllocator &powerAllocator,
                         DeviceController &controller) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!instance_) {
      instance_.reset(new TelemetryTask(powerAllocator, controller));
      return true;
    }
    return false;
  }

  // Static method to get the singleton instance
  static TelemetryTask *GetInstance() {
    std::lock_guard<std::mutex> lock(mutex_);
    return instance_.get();
  }

  PowerAllocator &GetPowerAllocator();

  void Start();

  void ReportDeviceBootInfo();
  void ReportMqttConnectionTime();

  void ReportAllUpgradeInfo(bool confirmed, bool confirmed_timeout = false);
  void ReportUpgradeStart();
  void ReportUpgradeError(esp_err_t err);

  void ReportESP32UpgradeError(const Version &version, esp_err_t err);
  void ReportESP32UpgradeInfo(const Version &version, UpgradeStatus status,
                              UpgradeFailureReason reason, esp_err_t esp_err);

#ifdef CONFIG_MCU_MODEL_SW3566
  void ReportSW3566UpgradeError(const Version &version, esp_err_t err);
  void ReportSW3566UpgradeInfo(const Version &version, UpgradeStatus status,
                               UpgradeFailureReason reason, esp_err_t esp_err);

  void ReportFPGAUpgradeError(const Version &version, esp_err_t err);
  void ReportFPGAUpgradeInfo(const Version &version, UpgradeStatus status,
                             UpgradeFailureReason reason, esp_err_t esp_err);
#endif

  esp_err_t ReportPingInfo();
  void ReportOTAConfirmInfo();

  void RequestLicense();

  void ReportStreamPortsStatus();
  void ReportStreamDeviceStatus();
  void ReportStreamPortPDStatus();

  void RequestShutdownBle();

  void SubscribeTelemetryStream();
  void UnsubscribeTelemetryStream();
  bool IsTelemetryStreamSubscribed();

  bool IsPaused();
  void Pause();
  void Resume();

  uint16_t GetMessageId();

  void ReportPowerAllocationData(
      uint16_t power_budget, uint8_t temperature, uint16_t remaining_power,
      uint16_t adc_value, const PortPowerAllocation port_power_allocations[8]);
};

#endif
