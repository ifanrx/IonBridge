#pragma once

#include <vector>

#include "app.h"
#include "esp_err.h"

namespace OTAHandler {
esp_err_t StartOTA(AppContext &ctx, const std::vector<uint8_t> &request,
                   std::vector<uint8_t> &response);
esp_err_t ConfirmOTA(AppContext &ctx, const std::vector<uint8_t> &request,
                     std::vector<uint8_t> &response);
};  // namespace OTAHandler
