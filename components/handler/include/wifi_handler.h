#ifndef WIFI_HANDLER_H
#define WIFI_HANDLER_H

#include <cstdint>
#include <vector>

#include "command.pb.h"
#include "esp_err.h"

namespace WiFiHandler {
namespace FromBLE {
esp_err_t ScanWiFi(const std::vector<uint8_t> &request,
                   std::vector<uint8_t> &response);
esp_err_t SetWiFiSSID(const std::vector<uint8_t> &request,
                      std::vector<uint8_t> &response);
esp_err_t SetWiFiPassword(const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
esp_err_t ResetWiFi(const std::vector<uint8_t> &request,
                    std::vector<uint8_t> &response);
esp_err_t GetWiFiStatus(const std::vector<uint8_t> &request,
                        std::vector<uint8_t> &response);
esp_err_t GetDeviceWiFiAddress(const std::vector<uint8_t> &request,
                               std::vector<uint8_t> &response);
esp_err_t SetWiFiSSIDAndPassword(const std::vector<uint8_t> &request,
                                 std::vector<uint8_t> &response);
esp_err_t GetWiFiStateMachine(const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
};  // namespace FromBLE

namespace FromMQTT {
esp_err_t ScanWiFi(const pb_CommandRequest &request,
                   pb_CommandResponse &response);
esp_err_t ResetWiFi(const pb_CommandRequest &request,
                    pb_CommandResponse &response);
esp_err_t GetWiFiStatus(const pb_CommandRequest &request,
                        pb_CommandResponse &response);
esp_err_t GetDeviceWiFiAddress(const pb_CommandRequest &request,
                               pb_CommandResponse &response);
esp_err_t SetWiFiSSIDAndPassword(const pb_CommandRequest &request,
                                 pb_CommandResponse &response);
esp_err_t GetWiFiRecords(const pb_CommandRequest &request,
                         pb_CommandResponse &response);
esp_err_t OperateWiFiRecord(const pb_CommandRequest &request,
                            pb_CommandResponse &response);
esp_err_t GetWiFiStateMachine(const pb_CommandRequest &request,
                              pb_CommandResponse &response);
esp_err_t SetWiFiStateMachine(const pb_CommandRequest &request,
                              pb_CommandResponse &response);
};  // namespace FromMQTT
};  // namespace WiFiHandler

#endif
