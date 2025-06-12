#pragma once

#include <vector>

#include "esp_err.h"

namespace OTAHandler {
esp_err_t StartOTA(const std::vector<uint8_t> &request,
                   std::vector<uint8_t> &response);
esp_err_t DownloadFirmware(const std::vector<uint8_t> &request,
                           std::vector<uint8_t> &response);
};  // namespace OTAHandler
