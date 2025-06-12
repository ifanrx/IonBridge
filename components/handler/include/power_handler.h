#ifndef POWER_HANDLER_H_
#define POWER_HANDLER_H_

#include <stdint.h>

#include <vector>

#include "esp_err.h"

namespace PowerHandler {
esp_err_t TogglePortPower(const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
esp_err_t GetPowerStats(const std::vector<uint8_t> &request,
                        std::vector<uint8_t> &response);
esp_err_t GetPowerHistoricalStats(const std::vector<uint8_t> &request,
                                  std::vector<uint8_t> &response);
esp_err_t GetPowerSupplyStatus(const std::vector<uint8_t> &request,
                               std::vector<uint8_t> &response);
esp_err_t GetChargingStatus(const std::vector<uint8_t> &request,
                            std::vector<uint8_t> &response);
esp_err_t SetChargingStrategy(const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
esp_err_t SetPortPriority(const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
esp_err_t GetPortPriority(const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
esp_err_t GetSlowChargingModeStatus(const std::vector<uint8_t> &request,
                                    std::vector<uint8_t> &response);
esp_err_t GetChargingStrategy(const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
esp_err_t GetPortPDStatus(const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
esp_err_t GetAllPowerStats(const std::vector<uint8_t> &request,
                           std::vector<uint8_t> &response);
esp_err_t GetStartChargeTimestamp(const std::vector<uint8_t> &request,
                                  std::vector<uint8_t> &response);
esp_err_t TurnOnPort(const std::vector<uint8_t> &request,
                     std::vector<uint8_t> &response);
esp_err_t TurnOffPort(const std::vector<uint8_t> &request,
                      std::vector<uint8_t> &response);
esp_err_t ForwardPort(const std::vector<uint8_t> &request,
                      std::vector<uint8_t> &response);
esp_err_t SetStaticAllocator(const std::vector<uint8_t> &request,
                             std::vector<uint8_t> &response);
esp_err_t GetStaticAllocator(const std::vector<uint8_t> &request,
                             std::vector<uint8_t> &response);
esp_err_t SetPortConfig(const std::vector<uint8_t> &request,
                        std::vector<uint8_t> &response);
esp_err_t GetPortConfig(const std::vector<uint8_t> &request,
                        std::vector<uint8_t> &response);
esp_err_t SetPortCompatibilitySettings(const std::vector<uint8_t> &request,
                                       std::vector<uint8_t> &response);
esp_err_t GetPortCompatibilitySettings(const std::vector<uint8_t> &request,
                                       std::vector<uint8_t> &response);
esp_err_t SetTemperatureMode(const std::vector<uint8_t> &request,
                             std::vector<uint8_t> &response);
esp_err_t SetTemporaryAllocator(const std::vector<uint8_t> &request,
                                std::vector<uint8_t> &response);

esp_err_t SubscribePDPcapStreaming(const std::vector<uint8_t> &request,
                                   std::vector<uint8_t> &response);
esp_err_t SetPortType(const std::vector<uint8_t> &request,
                      std::vector<uint8_t> &response);
};  // namespace PowerHandler

#endif
