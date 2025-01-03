#pragma once

#include <vector>

#include "app.h"
#include "esp_err.h"

namespace DisplayHandler {
esp_err_t SetDisplayIntensity(AppContext &ctx,
                              const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
esp_err_t SetDisplayMode(AppContext &ctx, const std::vector<uint8_t> &request,
                         std::vector<uint8_t> &response);
esp_err_t GetDisplayIntensity(AppContext &ctx,
                              const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
esp_err_t GetDisplayMode(AppContext &ctx, const std::vector<uint8_t> &request,
                         std::vector<uint8_t> &response);
esp_err_t SetDisplayFlip(AppContext &ctx, const std::vector<uint8_t> &request,
                         std::vector<uint8_t> &response);
esp_err_t GetDisplayFlip(AppContext &ctx, const std::vector<uint8_t> &request,
                         std::vector<uint8_t> &response);
esp_err_t SetDisplayConfig(AppContext &ctx, const std::vector<uint8_t> &request,
                           std::vector<uint8_t> &response);
esp_err_t SetDisplayState(AppContext &ctx, const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
esp_err_t GetDisplayState(AppContext &ctx, const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
};  // namespace DisplayHandler
