#include "task_handler.h"

#include "admin.pb.h"
#include "esp_check.h"
#include "esp_err.h"
#include "user_task_manager.h"

static const char *TAG = "TaskHandler";

esp_err_t TaskHandler::FromMQTT::ScheduleTask(
    const pb_AdminCommandRequest &request, pb_AdminCommandResponse &response) {
  response.which_payload = pb_AdminCommandResponse_schedule_task_tag;
  const pb_Task &task = request.payload.schedule_task.task;
  ESP_RETURN_ON_ERROR(UserTaskManager::GetInstance().ScheduleTask(task), TAG,
                      "schedule_task");
  return ESP_OK;
}

esp_err_t TaskHandler::FromMQTT::GetTaskStatus(
    const pb_AdminCommandRequest &request, pb_AdminCommandResponse &response) {
  uint32_t id = request.payload.get_task_status.task_id;

  pb_TaskExecutionStatus status;
  ESP_RETURN_ON_ERROR(UserTaskManager::GetInstance().GetTaskStatus(id, &status),
                      TAG, "Failed to get task status");
  response.which_payload = pb_AdminCommandResponse_get_task_status_tag;
  response.payload.get_task_status.status = status;
  return ESP_OK;
}