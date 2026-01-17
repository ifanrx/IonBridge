#include "user_task_manager.h"

#include <cinttypes>
#include <cstdint>

#include "admin.pb.h"
#include "app.h"
#include "command.pb.h"
#include "data_types.pb.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "pb_decode.h"
#include "sdkconfig.h"
#include "service.h"
#include "telemetry_task.h"
#include "time.h"

#define MAX_USER_TIMER_TASK CONFIG_MAX_USER_TIMER_TASK

static const char* TAG = "UserTaskManager";

UserTaskManager& UserTaskManager::GetInstance() {
  static UserTaskManager instance;
  return instance;
}

esp_err_t UserTaskManager::ScheduleTask(const pb_Task& task) {
  // Check if task already exists
  if (tasks_.find(task.id) != tasks_.end()) {
    ESP_LOGW(TAG, "Task ID %" PRIu32 " already exists", task.id);
    return ESP_ERR_INVALID_STATE;
  }

  if (tasks_.size() >= MAX_USER_TIMER_TASK) {
    ESP_LOGE(TAG, "Max user timer tasks reached: %d", MAX_USER_TIMER_TASK);
    return ESP_ERR_NO_MEM;
  }

  // Get current time
  time_t now;
  time(&now);

  // Calculate delay in seconds
  int64_t delay_sec = (int64_t)task.run_at - now;
  if (delay_sec < 0) {
    ESP_LOGW(TAG,
             "Task %" PRIu32 " scheduled in the past, executing immediately",
             task.id);
    delay_sec = 0;
  }

  // Convert to microseconds
  int64_t delay_us = delay_sec * 1000000;

  // Create task info
  TaskInfo task_info;
  task_info.id = task.id;
  task_info.run_at = task.run_at;
  task_info.commands.reserve(task.commands_count);
  for (size_t i = 0; i < task.commands_count; ++i) {
    Command command;
    command.service = static_cast<ServiceCommand>(task.commands[i].service);
    command.data.assign(
        task.commands[i].data.bytes,
        task.commands[i].data.bytes + task.commands[i].data.size);
    task_info.commands.emplace_back(command);
  }

  // Create timer
  esp_timer_create_args_t timer_args = {
      .callback = UserTaskManager::TimerCallback,
      .arg = (void*)(uintptr_t)task.id,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "task_timer",
      .skip_unhandled_events = true,
  };

  esp_err_t err = esp_timer_create(&timer_args, &task_info.timer_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(err));
    return err;
  }

  // Store task
  tasks_[task.id] = task_info;

  // Start timer
  err = esp_timer_start_once(task_info.timer_handle, delay_us);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start timer: %s", esp_err_to_name(err));
    esp_timer_delete(task_info.timer_handle);
    tasks_.erase(task.id);
    return err;
  }

  return ESP_OK;
}

void UserTaskManager::TimerCallback(void* arg) {
  uint32_t task_id = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(arg));
  UserTaskManager& manager = UserTaskManager::GetInstance();
  auto it = manager.tasks_.find(task_id);
  if (it == manager.tasks_.end()) {
    ESP_LOGW(TAG, "Timer triggered for unknown task %" PRIu32, task_id);
    return;
  }

  const TaskInfo& task = it->second;

  // Execute the task
  esp_err_t err = ESP_OK;
  for (const auto& command : task.commands) {
    ESP_LOGI(TAG, "Executing task %" PRIu32 ", service 0x%02X", task_id,
             static_cast<uint8_t>(command.service));
    err = manager.ExecuteTask(task_id, command.service, command.data.data(),
                              command.data.size());
    if (err != ESP_OK) {
      break;
    }
  }

  // Update and report status
  pb_TaskExecutionStatus status = err == ESP_OK
                                      ? pb_TaskExecutionStatus_SUCCEEDED
                                      : pb_TaskExecutionStatus_FAILED;
  manager.SaveTaskStatus(task_id, status);
  manager.ReportTaskExecution(task_id, status);

  // Clean up timer
  esp_timer_delete(task.timer_handle);
  manager.tasks_.erase(task_id);
}

esp_err_t UserTaskManager::ExecuteTask(uint32_t id, ServiceCommand service,
                                       const uint8_t* data, size_t len) {
  pb_CommandRequest request = {};  // Default initialize (usually zeros)
  pb_CommandResponse response = {};
  pb_istream_t stream = pb_istream_from_buffer(data, len);

  if (pb_decode(&stream, pb_CommandRequest_fields, &request)) {
    ESP_LOGD(TAG, "MQTT Message decoded, cmd: 0x%02x",
             static_cast<int>(service));
    // Execute the command (assuming App method is overloaded/templated)
    App::GetInstance().ExecMQTTSrv(service, request, response);
  } else {
    ESP_LOGE(TAG,
             "Failed to decode request: %s, task_id %" PRIu32
             ", service 0x%02X",
             PB_GET_ERROR(&stream), id, static_cast<uint8_t>(service));
    return ESP_ERR_INVALID_ARG;
  }

  return response.status == pb_CommandStatus_SUCCESS ? ESP_OK : ESP_FAIL;
}

esp_err_t UserTaskManager::GetTaskStatus(uint32_t id,
                                         pb_TaskExecutionStatus* status) {
  // First check in-memory cache
  auto it = tasks_.find(id);
  if (it != tasks_.end()) {
    *status = pb_TaskExecutionStatus_PENDING;
    return ESP_OK;
  }

  for (const auto& result : task_results_) {
    if (result.id == id) {
      *status = result.status;
      return ESP_OK;
    }
  }

  return ESP_ERR_NOT_FOUND;
}

esp_err_t UserTaskManager::SaveTaskStatus(uint32_t id,
                                          pb_TaskExecutionStatus status) {
  task_results_[result_index_].id = id;
  task_results_[result_index_].status = status;
  result_index_ = (result_index_ + 1) % task_results_.size();
  return ESP_OK;
}

esp_err_t UserTaskManager::ReportTaskExecution(uint32_t id,
                                               pb_TaskExecutionStatus status) {
  TelemetryTask::GetInstance().ReportTaskExecutionResult(id, status);
  return ESP_OK;
}
