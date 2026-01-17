#ifndef CONFIG_SYNC_HANDLER_H_
#define CONFIG_SYNC_HANDLER_H_

#include "admin.pb.h"
#include "esp_err.h"

namespace ConfigSyncHandler {
namespace FromMQTT {

esp_err_t SetMachineConfig(const pb_AdminCommandRequest& request,
                           pb_AdminCommandResponse& response);

esp_err_t CleanupNVSConfigs(const pb_AdminCommandRequest& request,
                            pb_AdminCommandResponse& response);

}  // namespace FromMQTT
}  // namespace ConfigSyncHandler

#endif  // CONFIG_SYNC_HANDLER_H_
