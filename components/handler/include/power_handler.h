#ifndef POWER_HANDLER_H_
#define POWER_HANDLER_H_

#include <stdint.h>

#include <vector>

#include "admin.pb.h"
#include "command.pb.h"
#include "esp_err.h"

namespace PowerHandler {
namespace FromBLE {
esp_err_t GetChargeDuration(const std::vector<uint8_t> &request,
                            std::vector<uint8_t> &response);
esp_err_t GetChargingStatus(const std::vector<uint8_t> &request,
                            std::vector<uint8_t> &response);
esp_err_t GetAllPowerStats(const std::vector<uint8_t> &request,
                           std::vector<uint8_t> &response);
esp_err_t ForwardPort(const std::vector<uint8_t> &request,
                      std::vector<uint8_t> &response);
esp_err_t GetChargingStrategy(const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
esp_err_t GetStaticAllocator(const std::vector<uint8_t> &request,
                             std::vector<uint8_t> &response);
esp_err_t SetPortConfig(const std::vector<uint8_t> &request,
                        std::vector<uint8_t> &response);
esp_err_t GetPortConfig(const std::vector<uint8_t> &request,
                        std::vector<uint8_t> &response);
esp_err_t GetPortsVin(const std::vector<uint8_t> &request,
                      std::vector<uint8_t> &response);
}  // namespace FromBLE

namespace FromMQTT {
esp_err_t GetPowerHistoricalStats(const pb_CommandRequest &request,
                                  pb_CommandResponse &response);
esp_err_t GetChargingStatus(const pb_CommandRequest &request,
                            pb_CommandResponse &response);
esp_err_t SetChargingStrategy(const pb_CommandRequest &request,
                              pb_CommandResponse &response);
esp_err_t SetPortPriority(const pb_CommandRequest &request,
                          pb_CommandResponse &response);
esp_err_t GetPortPriority(const pb_CommandRequest &request,
                          pb_CommandResponse &response);
esp_err_t GetChargingStrategy(const pb_CommandRequest &request,
                              pb_CommandResponse &response);
esp_err_t GetPortPDStatus(const pb_CommandRequest &request,
                          pb_CommandResponse &response);
esp_err_t GetAllPowerStats(const pb_CommandRequest &request,
                           pb_CommandResponse &response);
esp_err_t TurnOnPort(const pb_CommandRequest &request,
                     pb_CommandResponse &response);
esp_err_t TurnOffPort(const pb_CommandRequest &request,
                      pb_CommandResponse &response);
esp_err_t ForwardPort(const pb_AdminCommandRequest &request,
                      pb_AdminCommandResponse &response);
esp_err_t SetStaticAllocator(const pb_CommandRequest &request,
                             pb_CommandResponse &response);
esp_err_t GetStaticAllocator(const pb_CommandRequest &request,
                             pb_CommandResponse &response);
esp_err_t SetPortConfig(const pb_CommandRequest &request,
                        pb_CommandResponse &response);
esp_err_t GetPortConfig(const pb_CommandRequest &request,
                        pb_CommandResponse &response);
esp_err_t SetPortCompatibilitySettings(const pb_CommandRequest &request,
                                       pb_CommandResponse &response);
esp_err_t GetPortCompatibilitySettings(const pb_CommandRequest &request,
                                       pb_CommandResponse &response);
esp_err_t SetTemperatureMode(const pb_CommandRequest &request,
                             pb_CommandResponse &response);
esp_err_t SetTemporaryAllocator(const pb_CommandRequest &request,
                                pb_CommandResponse &response);
esp_err_t SubscribePDPcapStreaming(const pb_CommandRequest &request,
                                   pb_CommandResponse &response);
esp_err_t SetPortType(const pb_CommandRequest &request,
                      pb_CommandResponse &response);
esp_err_t SetCustomPdoVoltage(const pb_CommandRequest &request,
                              pb_CommandResponse &response);
esp_err_t SetCableCompensation(const pb_CommandRequest &request,
                               pb_CommandResponse &response);
esp_err_t GetCableCompensation(const pb_CommandRequest &request,
                               pb_CommandResponse &response);
esp_err_t ToggleMcuGPIO(const pb_AdminCommandRequest &request,
                        pb_AdminCommandResponse &response);

esp_err_t ReadADCValue(const pb_AdminCommandRequest &request,
                       pb_AdminCommandResponse &response);
};  // namespace FromMQTT
};  // namespace PowerHandler

#endif
