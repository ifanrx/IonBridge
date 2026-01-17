#include "firmware_programmer.h"

#include <cstring>

#include "command.pb.h"
#include "controller.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "internal/sw3566_command_types.h"
#include "mqtt_app.h"
#include "rpc.h"
#include "storage.h"
#include "telemetry_task.h"
#include "uart.h"
#include "upgrade.h"

static const char* TAG = "FirmwareProgrammer";

#define PROGRAMMER_UPDATE_DELAY_US \
  (3 * 1000 * 1000)  // 3 seconds in microseconds

FirmwareProgrammer::FirmwareProgrammer() {
  mutex_ = xSemaphoreCreateMutex();
  if (mutex_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create mutex");
  }

  esp_timer_create_args_t timer_args = {.callback = UpdateFirmwareTimerCallback,
                                        .arg = nullptr,
                                        .dispatch_method = ESP_TIMER_TASK,
                                        .name = "prog_update",
                                        .skip_unhandled_events = false};
  esp_err_t err = esp_timer_create(&timer_args, &update_timer_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create update timer: 0x%04X", err);
    update_timer_ = nullptr;
  }
}

FirmwareProgrammer::~FirmwareProgrammer() {
  if (update_timer_ != nullptr) {
    esp_timer_delete(update_timer_);
  }
  if (mutex_ != nullptr) {
    vSemaphoreDelete(mutex_);
  }
}

void FirmwareProgrammer::Start(uint8_t port_id) {
  if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to take mutex");
    return;
  }

  port_id_ = port_id;
  programming_ = false;
  state_done_ = false;
  state_error_ = false;
  error_reason_ = 0;

  xSemaphoreGive(mutex_);
  ESP_LOGI(TAG, "Started programmer on port %d", port_id);

  // set 5 min timeout
  DeviceController& controller = DeviceController::GetInstance();
  controller.reboot_after(5 * 60 * 1000);

  // Start firmware update process
  UpdateFirmware(version_, product_family_);
}

void FirmwareProgrammer::Stop() {
  if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to take mutex");
    return;
  }

  port_id_ = -1;  // Set to inactive
  programming_ = false;
  state_done_ = false;
  state_error_ = false;
  error_reason_ = 0;

  xSemaphoreGive(mutex_);
  ESP_LOGI(TAG, "Stopped programmer");
}

void FirmwareProgrammer::UpdateFirmware(const Version& version,
                                        const char* product_family) {
  ESP_LOGI(
      TAG,
      "Starting programmer firmware update for version: %s, product_family: %s",
      version.ToString().c_str(), product_family);

  if (update_timer_ == nullptr) {
    ESP_LOGE(TAG, "Update timer not initialized");
    return;
  }

  // Start timer for 3 seconds
  esp_err_t err =
      esp_timer_start_once(update_timer_, PROGRAMMER_UPDATE_DELAY_US);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start update timer: 0x%04X", err);
  }
}

void FirmwareProgrammer::HandleStartMessage(uint8_t port_id) {
  ESP_LOGI(TAG, "Received PROGRAMMER_START_CMD from port %d", port_id);

  if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to take mutex");
    return;
  }

  port_id_ = port_id;
  programming_ = true;
  state_done_ = false;
  state_error_ = false;
  error_reason_ = 0;

#ifdef CONFIG_MCU_MODEL_SW3566
  TelemetryTask& task = TelemetryTask::GetInstance();
  task.ReportProgrammerStart();
  StreamProgrammerStatus(true);
#endif

  xSemaphoreGive(mutex_);
}

void FirmwareProgrammer::HandleDoneMessage(uint8_t port_id) {
  ESP_LOGI(TAG, "Received PROGRAMMER_DONE_CMD from port %d", port_id);

  if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to take mutex");
    return;
  }

  port_id_ = port_id;
  state_done_ = true;
  state_error_ = false;
  programming_ = false;

#ifdef CONFIG_MCU_MODEL_SW3566
  TelemetryTask& task = TelemetryTask::GetInstance();
  task.ReportProgrammerFinish();
  StreamProgrammerStatus(false);
#endif

  xSemaphoreGive(mutex_);
  ESP_LOGI(TAG, "Programming completed successfully, rebooting shortly");

  // Cleanup programmer firmware after successful programming (one-time use)
  Storage::RemoveOldVersionFile(version_.ToString().c_str(), "PROG");

  DeviceController& controller = DeviceController::GetInstance();
  controller.reboot_after(5000);
}

void FirmwareProgrammer::HandleErrorMessage(uint8_t port_id, uint8_t reason) {
  ESP_LOGE(TAG, "Received PROGRAMMER_ERROR_CMD from port %d, reason: %d",
           port_id, reason);

  if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to take mutex");
    return;
  }

  port_id_ = port_id;
  state_error_ = true;
  error_reason_ = reason;
  state_done_ = false;
  programming_ = false;

#ifdef CONFIG_MCU_MODEL_SW3566
  TelemetryTask& task = TelemetryTask::GetInstance();
  task.ReportProgrammerFinish();
  StreamProgrammerStatus(false);
#endif

  xSemaphoreGive(mutex_);
  ESP_LOGE(TAG, "Programming failed with reason: %d", reason);

  DeviceController& controller = DeviceController::GetInstance();
  controller.reboot_after(5000);
}

#ifdef CONFIG_INTERFACE_TYPE_UART
void FirmwareProgrammer::HandleUARTMessage(const uart_message_t& uart_msg,
                                           uint8_t port_id) {
  ESP_LOGD(TAG, "Handling UART message: 0x%04x from port %d", uart_msg.command,
           port_id);

  switch (uart_msg.command) {
    case SW3566Command::PROGRAMMER_START_CMD: {
      HandleStartMessage(port_id);
    } break;
    case SW3566Command::PROGRAMMER_DONE_CMD: {
      HandleDoneMessage(port_id);
    } break;
    case SW3566Command::PROGRAMMER_ERROR_CMD: {
      ProgrammerErrorResponse resp;
      std::memcpy(&resp, uart_msg.payload, sizeof(ProgrammerErrorResponse));
      HandleErrorMessage(port_id, resp.reason);
    } break;
    default: {
      ESP_LOGD(TAG, "Ignoring command 0x%04x in programmer mode",
               uart_msg.command);
    } break;
  }
}
#endif

void FirmwareProgrammer::UpdateFirmwareTimerCallback(void* param) {
  ESP_LOGI(TAG, "Programmer update timer callback started");

  UpgradeManager& upgrade_mgr = UpgradeManager::GetInstance();
  FirmwareProgrammer& prog_mgr = FirmwareProgrammer::GetInstance();

  MQTTClient* mqtt_client = MQTTClient::GetInstance();
  if (mqtt_client != nullptr) {
    mqtt_client->Stop(true);
  }

  esp_err_t err = upgrade_mgr.ExecuteProgrammerDownload(
      prog_mgr.version_, prog_mgr.product_family_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Programmer download failed: 0x%04X", err);
    prog_mgr.Stop();
    return;
  }

  ESP_LOGI(TAG, "Programmer download completed");
  mqtt_client->ForceStart();

  // Port shutdown already handles GPIO toggle, so bring up the port directly
  int8_t port_id = prog_mgr.port_id_;
  err = rpc::mcu::bringup(port_id, false, &prog_mgr.version_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to bring up port %d: 0x%04X", port_id, err);
    return;
  }

  Storage::RemoveOldVersionFile(prog_mgr.version_.ToString().c_str(),
                                "Programmer");
  ESP_LOGI(TAG, "Port %d programmer update completed", port_id);
}

void FirmwareProgrammer::StreamProgrammerStatus(bool start) {
  ESP_LOGI(TAG, "Streaming programmer status: %s", start ? "START" : "STOP");

  pb_CommandResponse payload = pb_CommandResponse_init_zero;
  payload.status = pb_CommandStatus_SUCCESS;
  payload.which_payload = pb_CommandResponse_stream_programmer_status_tag;
  payload.payload.stream_programmer_status.start = start;
  payload.payload.stream_programmer_status.success = IsStateDone();
  payload.payload.stream_programmer_status.error_reason =
      static_cast<pb_ProgrammerState>(GetErrorReason());
  MQTTClient::GetInstance()->Publish(ServiceCommand::STREAM_PROGRAMMER_STATUS,
                                     payload);
}