#ifndef BLE_HANDLER_H_
#define BLE_HANDLER_H_

#include <vector>

#include "command.pb.h"
#include "esp_err.h"

namespace BleHandler {
namespace FromBLE {
esp_err_t EchoTest(const std::vector<uint8_t> &request,
                   std::vector<uint8_t> &response);
esp_err_t GetBleAddress(const std::vector<uint8_t> &request,
                        std::vector<uint8_t> &response);
esp_err_t SetBleState(const std::vector<uint8_t> &request,
                      std::vector<uint8_t> &response);
esp_err_t GetBleRSSI(const std::vector<uint8_t> &request,
                     std::vector<uint8_t> &response);
esp_err_t GetBleMTU(const std::vector<uint8_t> &request,
                    std::vector<uint8_t> &response);
};  // namespace FromBLE
namespace FromMQTT {
esp_err_t SetBleState(const pb_CommandRequest &request,
                      pb_CommandResponse &response);
};
};  // namespace BleHandler

#endif
