#ifndef TASK_HANDLER_H_
#define TASK_HANDLER_H_

#include "admin.pb.h"
#include "esp_err.h"

namespace TaskHandler {
namespace FromMQTT {
esp_err_t ScheduleTask(const pb_AdminCommandRequest &request,
                       pb_AdminCommandResponse &response);
esp_err_t GetTaskStatus(const pb_AdminCommandRequest &request,
                        pb_AdminCommandResponse &response);
}  // namespace FromMQTT

}  // namespace TaskHandler

#endif /* TASK_HANDLER_H_ */