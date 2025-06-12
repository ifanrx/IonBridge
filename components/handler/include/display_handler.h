#pragma once

#include <vector>

#include "esp_err.h"

namespace DisplayHandler {
esp_err_t SetDisplayIntensity(const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
esp_err_t SetDisplayMode(const std::vector<uint8_t> &request,
                         std::vector<uint8_t> &response);
esp_err_t GetDisplayIntensity(const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
esp_err_t GetDisplayMode(const std::vector<uint8_t> &request,
                         std::vector<uint8_t> &response);
esp_err_t SetDisplayFlip(const std::vector<uint8_t> &request,
                         std::vector<uint8_t> &response);
esp_err_t GetDisplayFlip(const std::vector<uint8_t> &request,
                         std::vector<uint8_t> &response);
esp_err_t SetDisplayConfig(const std::vector<uint8_t> &request,
                           std::vector<uint8_t> &response);
esp_err_t SetDisplayState(const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
esp_err_t GetDisplayState(const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
};  // namespace DisplayHandler
