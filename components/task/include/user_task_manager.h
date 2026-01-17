#ifndef TASK_MANAGER_H_
#define TASK_MANAGER_H_

#include <array>
#include <map>
#include <vector>

#include "admin.pb.h"
#include "data_types.pb.h"
#include "esp_timer.h"
#include "service.h"

class UserTaskManager {
 public:
  static UserTaskManager& GetInstance();

  esp_err_t ScheduleTask(const pb_Task& request);
  esp_err_t GetTaskStatus(uint32_t id, pb_TaskExecutionStatus* status);
  esp_err_t ReportTaskExecution(uint32_t id, pb_TaskExecutionStatus status);

 private:
  static void TimerCallback(void* arg);
  esp_err_t ExecuteTask(uint32_t id, ServiceCommand service,
                        const uint8_t* data, size_t len);
  esp_err_t SaveTaskStatus(uint32_t id, pb_TaskExecutionStatus status);

  struct Command {
    ServiceCommand service;
    std::vector<uint8_t> data;
  };

  struct TaskInfo {
    uint32_t id;
    uint32_t run_at;
    std::vector<Command> commands;
    esp_timer_handle_t timer_handle;
  };

  std::map<uint32_t, TaskInfo> tasks_;

  struct TaskResult {
    uint32_t id;
    pb_TaskExecutionStatus status;
  };

  int result_index_ = 0;
  std::array<TaskResult, 2> task_results_ = {};
};

#endif /* TASK_MANAGER_H_ */
