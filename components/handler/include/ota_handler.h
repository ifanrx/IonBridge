#pragma once

#include "admin.pb.h"
#include "command.pb.h"
#include "esp_err.h"
#include "sdkconfig.h"

namespace OTAHandler {
namespace FromMQTT {
esp_err_t StartOTA(const pb_CommandRequest &request,
                   pb_CommandResponse &response);
};  // namespace FromMQTT
};  // namespace OTAHandler
