#ifndef TELEMETRYTASK_H_
#define TELEMETRYTASK_H_

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "freertos/idf_additions.h"
#include "mqtt_message.h"
#include "sdkconfig.h"
#include "version.h"

#ifdef CONFIG_MCU_MODEL_SW3566
#include "sw3566_data_types.h"
#endif

class TelemetryTask {
  // Member variables
  static constexpr int64_t kSubscribeTimeoutUs = 2 * 60 * 1e6;
  volatile int64_t subscribed_at_ = -1;
  volatile bool paused = false;
  bool debug_ = false, started_ = false;
  TaskHandle_t task_handle_;
  QueueHandle_t mqtt_msg_queue_;

  // Buffer pool configuration
  using BufferId = uint8_t;
  using BufferType = uint8_t *;

  static constexpr uint32_t kMutexTimeoutMs = 50;
  static constexpr BufferId INVALID_BUFFER_ID = 0xFF;
  static constexpr size_t BUFFER_SIZE = 272;
  static constexpr size_t BUFFER_COUNT = 32;

  BufferType msg_buffers_[BUFFER_COUNT];
  size_t msg_buffer_sizes_[BUFFER_COUNT] = {0};
  SemaphoreHandle_t buffer_mutex_;

  // Private constructor
  TelemetryTask() = default;
  ~TelemetryTask();

  static void TaskLoopWrapper(void *pvParameters);
  void TaskLoop();
  void ReportMQTTMessage(const MQTTMessage &msg);

  void InitBufferPool();
  BufferId AcquireBuffer(size_t buffer_size);
  void ReleaseBuffer(BufferId buffer_id);
  BufferType GetBuffer(BufferId buffer_id);
  size_t GetBufferSize(BufferId buffer_id);

 public:
  // Delete copy constructor and assignment operator
  TelemetryTask(TelemetryTask const &) = delete;
  TelemetryTask &operator=(TelemetryTask const &) = delete;

  // Static method to get the singleton instance
  static TelemetryTask &GetInstance() {
    static TelemetryTask instance;
    return instance;
  }

  void Start();
  bool IsDebug() { return debug_; }
  void SetDebug(bool debug) { debug_ = debug; }

  void ReportDeviceBootInfo();
  void ReportMqttConnectionTime();

  void ReportUpgradeResult();
  void ReportAllUpgradeInfo(bool confirmed, bool confirmed_timeout = false);
  void ReportUpgradeStart();
  void ReportUpgradeError(esp_err_t err);

  void ReportESP32UpgradeError(const Version &version, esp_err_t err);
  void ReportESP32UpgradeInfo(const Version &version, UpgradeStatus status,
                              UpgradeFailureReason reason, esp_err_t esp_err);

#ifdef CONFIG_MCU_MODEL_SW3566
  void ReportPDPCapData(uint8_t port, PcapEntry &pcap_entry);
#endif

  esp_err_t ReportPingInfo();
  void ReportOTAConfirmInfo();

  void RequestLicense();

  void RequestShutdownBle();

  void SubscribeTelemetryStream();
  void UnsubscribeTelemetryStream();
  bool IsTelemetryStreamSubscribed();

  bool IsPaused();
  void Pause();
  void Resume();

  void ReportPowerAllocationData(uint16_t power_budget,
                                 uint16_t remaining_power);

  void ReportOutOfMemory(uint32_t caps) const;
};

#endif
