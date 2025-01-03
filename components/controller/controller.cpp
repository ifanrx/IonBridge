#include "controller.h"

#include <sys/_intsup.h>
#include <sys/types.h>

#include "data_types.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_flash_partitions.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "ionbridge.h"
#include "mqtt_app.h"
#include "power_allocator.h"
#include "rpc.h"
#include "sdkconfig.h"
#include "telemetry_task.h"
#include "utils.h"

#define OTA_CONFIRM_REQUEST_TIMEOUT CONFIG_OTA_CONFIRM_REQUEST_TIMEOUT
#define OTA_CONFIRM_REQUEST_INTERVAL CONFIG_OTA_CONFIRM_REQUEST_INTERVAL
#define OTA_CONFIRM_TIMEOUT CONFIG_OTA_CONFIRM_TIMEOUT

static const char *TAG = "Controller";
static uint32_t elapsed_time_ms = 0;

void rollback() {
  ESP_LOGI(TAG, "Rolling back to previous app");
  if (esp_ota_mark_app_invalid_rollback_and_reboot() != ESP_OK) {
    ESP_LOGE(TAG,
             "esp_ota_mark_app_invalid_rollback_and_reboot failed, restarting");
    esp_restart();
  }
}

bool is_ota_pending_verify() {
  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t ota_state;
  esp_err_t ret = esp_ota_get_state_partition(running, &ota_state);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_get_state_partition: %d, rolling back", ret);
    rollback();
    return false;
  }
  return ota_state == ESP_OTA_IMG_PENDING_VERIFY;
}

void confirmation_failed_cb(void *arg) {
  ESP_LOGE(TAG, "Confirmation timed out, rolling back");
  esp_err_t ret = ESP_ERR_TIMEOUT;
  MQTTClient *client = MQTTClient::GetInstance();
  TelemetryTask *task = TelemetryTask::GetInstance();
  WAIT_FOR_CONDITION_OR_GOTO(10, client->Connected(), 100, ROLLBACK,
                             "MQTT connection timeout, rolling back anyway");
  if (task) {
    task->ReportUpgradeError(ret);
  }
  client->Stop();  // Calling Stop to ensure that the MQTT messages are sent

ROLLBACK:
  rollback();
}

void request_timer_cb(void *arg) {
  elapsed_time_ms += OTA_CONFIRM_REQUEST_INTERVAL;

  DeviceController *controller = static_cast<DeviceController *>(arg);
  // Check MQTT connection
  MQTTClient *client = MQTTClient::GetInstance();
  TelemetryTask *task = TelemetryTask::GetInstance();
  if (client->Connected() && controller->is_normally_booted()) {
    ESP_LOGI(TAG,
             "MQTT connected and normally booted, requesting OTA confirmation");
    // Send OTA confirmation request
    task->ReportOTAConfirmInfo();
    controller->waiting_for_confirmation();
    return;
  }

  if (elapsed_time_ms >= OTA_CONFIRM_REQUEST_TIMEOUT) {
    ESP_LOGE(TAG, "Confirmation request timed out, rollback to previous app");
    esp_ota_mark_app_invalid_rollback_and_reboot();
    return;
  }

  ESP_LOGW(TAG, "MQTT not connected, check again in %dms",
           OTA_CONFIRM_REQUEST_INTERVAL);
}

DeviceController::DeviceController() {
  if (is_ota_pending_verify()) {
    status_ = DeviceControllerStatus::REQUESTING_CONFIRMATION;
    start_request_timer(OTA_CONFIRM_REQUEST_INTERVAL);
    return;
  }
  ESP_LOGD(TAG, "No OTA flag found");
  status_ = DeviceControllerStatus::POWER_UP;
}

void DeviceController::start_request_timer(int timeout_ms) {
  ESP_LOGI(TAG, "Requesting OTA confirmation, timeout: %dms", timeout_ms);
  esp_timer_create_args_t timer_config = {.callback = &request_timer_cb,
                                          .arg = (void *)this,
                                          .dispatch_method = ESP_TIMER_TASK,
                                          .name = "REQUEST_CONFIRMATION_TIMER",
                                          .skip_unhandled_events = true};
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      esp_timer_create(&timer_config, &request_timer_));
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      esp_timer_start_periodic(request_timer_, timeout_ms * 1000));
}

void DeviceController::start_confirmation_timer(int timeout_ms) {
  ESP_LOGI(TAG, "Waiting for OTA confirmation, timeout: %dms", timeout_ms);
  esp_timer_create_args_t timer_config = {.callback = &confirmation_failed_cb,
                                          .arg = nullptr,
                                          .dispatch_method = ESP_TIMER_TASK,
                                          .name = "CONFIRM_TIMER",
                                          .skip_unhandled_events = true};
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      esp_timer_create(&timer_config, &confirmation_timer_));
  ESP_ERROR_CHECK_WITHOUT_ABORT(
      esp_timer_start_once(confirmation_timer_, timeout_ms * 1000));
}

void DeviceController::waiting_for_confirmation() {
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_stop(request_timer_));
  status_ = DeviceControllerStatus::WAITING_FOR_CONFIRMATION;
  start_confirmation_timer(OTA_CONFIRM_TIMEOUT);
}

esp_err_t DeviceController::confirm(const uint8_t *hash, size_t hash_len) {
  esp_err_t ret = ESP_OK;
  TelemetryTask *task = TelemetryTask::GetInstance();
  ESP_RETURN_ON_FALSE(
      status_ == DeviceControllerStatus::WAITING_FOR_CONFIRMATION,
      ESP_ERR_INVALID_STATE, TAG,
      "Attempted to confirm OTA while not waiting for it");
  ESP_GOTO_ON_FALSE(validate_partition_hash(hash, hash_len),
                    ESP_ERR_INVALID_ARG, ROLLBACK, TAG, "Invalid hash");
  ESP_GOTO_ON_ERROR(esp_ota_mark_app_valid_cancel_rollback(), ROLLBACK, TAG,
                    "esp_ota_mark_app_valid_cancel_rollback");
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_stop(confirmation_timer_));
  ESP_LOGI(TAG, "OTA confirmed, powering up");
  status_ = DeviceControllerStatus::POWER_UP;
  if (task) {
    task->ReportAllUpgradeInfo(true, false);
  }
  return ret;

ROLLBACK:
  ESP_LOGE(TAG, "Failed to confirm OTA, rolling back");
  if (task) {
    task->ReportAllUpgradeInfo(false, false);
  }
  rollback();
  return ret;
}

esp_err_t DeviceController::power_off() {
  if (status_ == DeviceControllerStatus::POWER_DOWN) {
    return ESP_OK;
  }
  ESP_RETURN_ON_FALSE(status_ == DeviceControllerStatus::POWER_UP,
                      ESP_ERR_INVALID_STATE, TAG,
                      "Attempted to power off while not in POWER_UP state");
  status_ = DeviceControllerStatus::POWER_DOWN;
  ESP_ERROR_COMPLAIN(rpc::display::set_display_mode(DisplayMode::OFF),
                     "rpc::display::set_display_mode: OFF");
  PowerAllocator::GetInstance().GetPortManager().TurnOffAllPorts();
  return ESP_OK;
}

esp_err_t DeviceController::power_on() {
  if (status_ == DeviceControllerStatus::POWER_UP) {
    return ESP_OK;
  }
  status_ = DeviceControllerStatus::POWER_UP;
  ESP_ERROR_COMPLAIN(rpc::display::set_display_mode(display_mode_),
                     "rpc::display::set_display_mode: 0x%x", display_mode_);
  PowerAllocator::GetInstance().GetPortManager().TurnOnAllPorts();
  return ESP_OK;
}

bool DeviceController::should_reboot() {
  return reboot_at_ > 0 && esp_timer_get_time() > reboot_at_;
}

void DeviceController::reboot_after(int delay_ms) {
  reboot_at_ = esp_timer_get_time() + delay_ms * 1e3;
}

void DeviceController::reboot() {
  ESP_LOGI(TAG, "Rebooting");
  esp_restart();
}

esp_err_t DeviceController::set_display_intensity(uint8_t intensity) {
  ESP_RETURN_ON_ERROR(rpc::display::set_display_intensity(intensity), TAG,
                      "rpc::display::set_display_intensity");
  display_intensity_ = intensity;
  return ESP_OK;
}

esp_err_t DeviceController::set_display_mode(uint8_t mode) {
  ESP_RETURN_ON_ERROR(rpc::display::set_display_mode(mode), TAG,
                      "rpc::display::set_display_mode");
  display_mode_ = mode;
  return ESP_OK;
}
