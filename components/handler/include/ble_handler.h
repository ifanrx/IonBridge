#ifndef BLE_HANDLER_H_
#define BLE_HANDLER_H_

#include <vector>

#include "esp_err.h"

namespace BleHandler {
esp_err_t BLEEchoTest(const std::vector<uint8_t> &request,
                      std::vector<uint8_t> &response);
esp_err_t GetDeviceBleAddress(const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
esp_err_t SetBleState(const std::vector<uint8_t> &request,
                      std::vector<uint8_t> &response);
esp_err_t GetBleRSSI(const std::vector<uint8_t> &request,
                     std::vector<uint8_t> &response);
esp_err_t GetBleMTU(const std::vector<uint8_t> &request,
                    std::vector<uint8_t> &response);
};  // namespace BleHandler

#endif
