#ifndef BLE_HANDLER_H_
#define BLE_HANDLER_H_

#include <vector>

#include "app.h"
#include "esp_err.h"

namespace BleHandler {
esp_err_t BLEEchoTest(AppContext &ctx, const std::vector<uint8_t> &request,
                      std::vector<uint8_t> &response);
esp_err_t GetDeviceBleAddress(AppContext &ctx,
                              const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
esp_err_t SetBleState(AppContext &ctx, const std::vector<uint8_t> &request,
                      std::vector<uint8_t> &response);
esp_err_t GetBleRSSI(AppContext &ctx, const std::vector<uint8_t> &request,
                     std::vector<uint8_t> &response);
};  // namespace BleHandler

#endif
