#ifndef APP_T
#define APP_T

#include "esp_log.h"
#include "ionbridge.h"

template <typename CmdEnumT, typename PbRequestT, typename PbResponseT,
          typename HandlerMapT>
void App::ExecMQTTSrvGeneric(CmdEnumT command, const PbRequestT& request,
                             PbResponseT& response,
                             const HandlerMapT& handler_map) {
  const char* TAG = "App";
  const char* cmdTypeName = "Unknown";
  // Determine command type name for logging using if constexpr
  if constexpr (std::is_same_v<CmdEnumT, ServiceCommand>) {
    cmdTypeName = "Service";
  } else if constexpr (std::is_same_v<CmdEnumT, AdminCommand>) {
    cmdTypeName = "Admin service";
  } else {
    ESP_LOGE(TAG, "Unknown command type");
    response.status = pb_CommandStatus_FAILURE;
    return;
  }

  // 1. Find Handler
  auto it = handler_map.find(command);
  if (it == handler_map.end()) {
    ESP_LOGW(TAG, "%s not found: 0x%02X", cmdTypeName,
             static_cast<int>(command));
    response.status = pb_CommandStatus_FAILURE;
    return;
  }

  // 2. Pre-checks (Conditional based on command type)
  if constexpr (std::is_same_v<CmdEnumT, ServiceCommand>) {
    // ServiceCommand specific pre-check
    pb_CommandStatus preCheckRes =
        PreCheckSrv(command);  // Assumes takes ServiceCommand
    if (preCheckRes != pb_CommandStatus_SUCCESS) {
      ESP_LOGW(TAG, "Service 0x%02x pre-check failed: %d",
               static_cast<int>(command), (int)preCheckRes);
      response.status = preCheckRes;
      return;
    }
  } else {
    // AdminCommand specific pre-check (high temp check before execution)
    if (!is_service_available_at_high_temp(
            command)) {  // Assumes takes AdminCommand
      ESP_LOGW(TAG, "Admin service 0x%02x unavailable at high temp (pre-check)",
               static_cast<int>(command));
      response.status = pb_CommandStatus_FLASH_NOT_WRITABLE_AT_HIGH_TEMPERATURE;
      return;
    }
  }

  // 3. Execute Handler
  auto handler = it->second;  // Get handler function from iterator
  esp_err_t err = handler(request, response);

  // 4. Handle Result
  if (err != ESP_OK) {
    ESP_ERROR_COMPLAIN(err, "Failed to execute MQTT App %s 0x%02x", cmdTypeName,
                       static_cast<int>(command));

    // Post-failure check (common high-temp check)
    // Assuming is_service_available_at_high_temp works for both command types
    if (!is_service_available_at_high_temp(command)) {
      ESP_LOGW(TAG,
               "%s 0x%02x failed and unavailable at high temp (post-check)",
               cmdTypeName, static_cast<int>(command));
      response.status = pb_CommandStatus_FLASH_NOT_WRITABLE_AT_HIGH_TEMPERATURE;
    } else {
      response.status = pb_CommandStatus_FAILURE;
    }
    return;  // Return after setting failure status
  }

  // 5. Success
  response.status = pb_CommandStatus_SUCCESS;
}

#endif
