#ifndef POWER_HANDLER_H_
#define POWER_HANDLER_H_

#include <stdint.h>

#include <vector>

#include "app.h"
#include "esp_err.h"

namespace PowerHandler {
esp_err_t TogglePortPower(AppContext &ctx, const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
esp_err_t GetPowerStats(AppContext &ctx, const std::vector<uint8_t> &request,
                        std::vector<uint8_t> &response);
esp_err_t GetPowerHistoricalStats(AppContext &ctx,
                                  const std::vector<uint8_t> &request,
                                  std::vector<uint8_t> &response);
esp_err_t GetPowerSupplyStatus(AppContext &ctx,
                               const std::vector<uint8_t> &request,
                               std::vector<uint8_t> &response);
esp_err_t GetChargingStatus(AppContext &ctx,
                            const std::vector<uint8_t> &request,
                            std::vector<uint8_t> &response);
esp_err_t SetChargingStrategy(AppContext &ctx,
                              const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
esp_err_t SetPortPriority(AppContext &ctx, const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
esp_err_t GetPortPriority(AppContext &ctx, const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
esp_err_t GetSlowChargingModeStatus(AppContext &ctx,
                                    const std::vector<uint8_t> &request,
                                    std::vector<uint8_t> &response);
esp_err_t GetChargingStrategy(AppContext &ctx,
                              const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
esp_err_t GetPortPDStatus(AppContext &ctx, const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
esp_err_t GetAllPowerStats(AppContext &ctx, const std::vector<uint8_t> &request,
                           std::vector<uint8_t> &response);
esp_err_t GetStartChargeTimestamp(AppContext &ctx,
                                  const std::vector<uint8_t> &request,
                                  std::vector<uint8_t> &response);
esp_err_t TurnOnPort(AppContext &ctx, const std::vector<uint8_t> &request,
                     std::vector<uint8_t> &response);
esp_err_t TurnOffPort(AppContext &ctx, const std::vector<uint8_t> &request,
                      std::vector<uint8_t> &response);
esp_err_t SetPortOverclockFeature(AppContext &ctx,
                                  const std::vector<uint8_t> &request,
                                  std::vector<uint8_t> &response);
esp_err_t GetPortOverclockFeature(AppContext &ctx,
                                  const std::vector<uint8_t> &request,
                                  std::vector<uint8_t> &response);
esp_err_t SetPortTFCPFeature(AppContext &ctx,
                             const std::vector<uint8_t> &request,
                             std::vector<uint8_t> &response);
esp_err_t GetPortTFCPFeature(AppContext &ctx,
                             const std::vector<uint8_t> &request,
                             std::vector<uint8_t> &response);
esp_err_t SetPortUFCSFeature(AppContext &ctx,
                             const std::vector<uint8_t> &request,
                             std::vector<uint8_t> &response);
esp_err_t GetPortUFCSFeature(AppContext &ctx,
                             const std::vector<uint8_t> &request,
                             std::vector<uint8_t> &response);
esp_err_t SetStaticAllocator(AppContext &ctx,
                             const std::vector<uint8_t> &request,
                             std::vector<uint8_t> &response);
esp_err_t GetStaticAllocator(AppContext &ctx,
                             const std::vector<uint8_t> &request,
                             std::vector<uint8_t> &response);
esp_err_t SetPortConfig(AppContext &ctx, const std::vector<uint8_t> &request,
                        std::vector<uint8_t> &response);
esp_err_t GetPortConfig(AppContext &ctx, const std::vector<uint8_t> &request,
                        std::vector<uint8_t> &response);
esp_err_t SetPortCompatibilitySettings(AppContext &ctx,
                                       const std::vector<uint8_t> &request,
                                       std::vector<uint8_t> &response);
esp_err_t GetPortCompatibilitySettings(AppContext &ctx,
                                       const std::vector<uint8_t> &request,
                                       std::vector<uint8_t> &response);
esp_err_t SetTemperatureMode(AppContext &ctx,
                             const std::vector<uint8_t> &request,
                             std::vector<uint8_t> &response);
esp_err_t SetTemporaryAllocator(AppContext &ctx,
                                const std::vector<uint8_t> &request,
                                std::vector<uint8_t> &response);
};  // namespace PowerHandler

#endif
