#pragma once

#include <vector>

#include "command.pb.h"
#include "esp_err.h"

namespace DisplayHandler {
namespace FromBLE {
esp_err_t SetDisplayState(const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
esp_err_t GetDisplayState(const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
esp_err_t GetDisplayRotation(const std::vector<uint8_t> &request,
                             std::vector<uint8_t> &response);
esp_err_t SetDisplayRotation(const std::vector<uint8_t> &request,
                             std::vector<uint8_t> &response);
esp_err_t GetDisplayIntensity(const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
esp_err_t SetDisplayIntensity(const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
esp_err_t ClearFactoryDMFlag(const std::vector<uint8_t> &request,
                             std::vector<uint8_t> &response);
};  // namespace FromBLE

namespace FromMQTT {
esp_err_t SetDisplayIntensity(const pb_CommandRequest &request,
                              pb_CommandResponse &response);
esp_err_t SetDisplayMode(const pb_CommandRequest &request,
                         pb_CommandResponse &response);
esp_err_t GetDisplayIntensity(const pb_CommandRequest &request,
                              pb_CommandResponse &response);
esp_err_t GetDisplayMode(const pb_CommandRequest &request,
                         pb_CommandResponse &response);
esp_err_t SetDisplayFlip(const pb_CommandRequest &request,
                         pb_CommandResponse &response);
esp_err_t GetDisplayFlip(const pb_CommandRequest &request,
                         pb_CommandResponse &response);
esp_err_t SetDisplayConfig(const pb_CommandRequest &request,
                           pb_CommandResponse &response);

esp_err_t GetDisplayRotation(const pb_CommandRequest &request,
                             pb_CommandResponse &response);
esp_err_t SetDisplayRotation(const pb_CommandRequest &request,
                             pb_CommandResponse &response);
esp_err_t ClearFactoryDMFlag(const pb_CommandRequest &request,
                             pb_CommandResponse &response);
esp_err_t SetDisplaySetup(const pb_CommandRequest &request,
                          pb_CommandResponse &response);
esp_err_t GetDisplaySetup(const pb_CommandRequest &request,
                          pb_CommandResponse &response);
};  // namespace FromMQTT
};  // namespace DisplayHandler
