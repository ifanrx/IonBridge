#ifndef WIFI_HANDLER_H
#define WIFI_HANDLER_H

#include <cstdint>
#include <vector>

#include "esp_err.h"

namespace WiFiHandler {
esp_err_t ScanWifi(const std::vector<uint8_t> &request,
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
esp_err_t GetWiFiRecords(const std::vector<uint8_t> &request,
                         std::vector<uint8_t> &response);
esp_err_t OperateWiFiRecord(const std::vector<uint8_t> &request,
                            std::vector<uint8_t> &response);
esp_err_t GetWiFiStateMachine(const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
esp_err_t SetWiFiStateMachine(const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);

};  // namespace WiFiHandler

#endif
