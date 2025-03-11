#ifndef WIFI_HANDLER_H
#define WIFI_HANDLER_H

#include <cstdint>
#include <vector>

#include "app.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

namespace WiFiHandler {
esp_err_t ScanWifi(AppContext &ctx, const std::vector<uint8_t> &request,
                   std::vector<uint8_t> &response);
esp_err_t SetWiFiSSID(AppContext &ctx, const std::vector<uint8_t> &request,
                      std::vector<uint8_t> &response);
esp_err_t SetWiFiPassword(AppContext &ctx, const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
esp_err_t ResetWiFi(AppContext &ctx, const std::vector<uint8_t> &request,
                    std::vector<uint8_t> &response);
esp_err_t GetWiFiStatus(AppContext &ctx, const std::vector<uint8_t> &request,
                        std::vector<uint8_t> &response);
esp_err_t GetDeviceWiFiAddress(AppContext &ctx,
                               const std::vector<uint8_t> &request,
                               std::vector<uint8_t> &response);
esp_err_t SetWiFiSSIDAndPassword(AppContext &ctx,
                                 const std::vector<uint8_t> &request,
                                 std::vector<uint8_t> &response);
esp_err_t GetWiFiRecords(AppContext &ctx, const std::vector<uint8_t> &request,
                         std::vector<uint8_t> &response);
esp_err_t OperateWiFiRecord(AppContext &ctx,
                            const std::vector<uint8_t> &request,
                            std::vector<uint8_t> &response);
esp_err_t GetWiFiStateMachine(AppContext &ctx,
                              const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
esp_err_t SetWiFiStateMachine(AppContext &ctx,
                              const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);

};  // namespace WiFiHandler

#ifdef __cplusplus
}
#endif

#endif
