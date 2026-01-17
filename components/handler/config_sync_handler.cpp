#include "config_sync_handler.h"

#include "admin.pb.h"
#include "config_sync_manager.h"
#include "esp_err.h"

namespace ConfigSyncHandler {
namespace FromMQTT {

esp_err_t SetMachineConfig(const pb_AdminCommandRequest& request,
                           pb_AdminCommandResponse& response) {
  response.which_payload = pb_AdminCommandResponse_set_machine_config_tag;
  response.payload.set_machine_config.success = false;

  const pb_SetMachineConfigReq& req = request.payload.set_machine_config;
  ConfigSyncManager& sync_manager = ConfigSyncManager::GetInstance();

  // Delegate all handling to ConfigSyncManager
  esp_err_t ret = sync_manager.HandleSetMachineConfig(req);

  if (ret == ESP_OK) {
    response.payload.set_machine_config.success = true;
  }

  return ret;
}

esp_err_t CleanupNVSConfigs(const pb_AdminCommandRequest& request,
                            pb_AdminCommandResponse& response) {
  response.which_payload = pb_AdminCommandResponse_cleanup_nvs_configs_tag;
  auto& resp = response.payload.cleanup_nvs_configs;

  const pb_CleanupNVSConfigsReq& req = request.payload.cleanup_nvs_configs;
  ConfigSyncManager& sync_manager = ConfigSyncManager::GetInstance();

  // Simple cleanup call
  esp_err_t ret = sync_manager.CleanupNVS(req.force);

  // Simple response
  resp.success = (ret == ESP_OK);
  snprintf(resp.message, sizeof(resp.message),
           resp.success ? "NVS cleanup complete"
                        : "NVS cleanup failed - check logs");

  return ret;
}

}  // namespace FromMQTT
}  // namespace ConfigSyncHandler
