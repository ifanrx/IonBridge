#ifndef DEVICE_HANDLER_H_
#define DEVICE_HANDLER_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "app.h"
#include "esp_err.h"

#define PORT_NUMBER CONFIG_SW3566_COUNT

namespace DeviceHandler {
esp_err_t AssociateDevice(AppContext &ctx, const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
esp_err_t ResetDevice(AppContext &ctx, const std::vector<uint8_t> &request,
                      std::vector<uint8_t> &response);
esp_err_t RebootDevice(AppContext &ctx, const std::vector<uint8_t> &request,
                       std::vector<uint8_t> &response);
esp_err_t GetAPVersion(AppContext &ctx, const std::vector<uint8_t> &request,
                       std::vector<uint8_t> &response);
esp_err_t GetBPVersion(AppContext &ctx, const std::vector<uint8_t> &request,
                       std::vector<uint8_t> &response);
esp_err_t GetFPGAVersion(AppContext &ctx, const std::vector<uint8_t> &request,
                         std::vector<uint8_t> &response);
esp_err_t GetZRLIBVersion(AppContext &ctx, const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
esp_err_t GetDeviceSerialNumber(AppContext &ctx,
                                const std::vector<uint8_t> &request,
                                std::vector<uint8_t> &response);
esp_err_t GetDeviceUpTime(AppContext &ctx, const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
esp_err_t SwitchDevice(AppContext &ctx, const std::vector<uint8_t> &request,
                       std::vector<uint8_t> &response);
esp_err_t GetDeviceSwitch(AppContext &ctx, const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
esp_err_t GetDeviceModel(AppContext &ctx, const std::vector<uint8_t> &request,
                         std::vector<uint8_t> &response);
esp_err_t GetSecureBootDigest(AppContext &ctx,
                              const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
esp_err_t PingMQTTTelemetry(AppContext &ctx,
                            const std::vector<uint8_t> &request,
                            std::vector<uint8_t> &response);
esp_err_t PushLicense(AppContext &ctx, const std::vector<uint8_t> &request,
                      std::vector<uint8_t> &response);
esp_err_t PingHTTP(AppContext &ctx, const std::vector<uint8_t> &request,
                   std::vector<uint8_t> &response);
esp_err_t GetDeviceInfo(AppContext &ctx, const std::vector<uint8_t> &request,
                        std::vector<uint8_t> &response);
esp_err_t SetSyslogState(AppContext &ctx, const std::vector<uint8_t> &request,
                         std::vector<uint8_t> &response);
esp_err_t GetDevicePassword(AppContext &ctx,
                            const std::vector<uint8_t> &request,
                            std::vector<uint8_t> &response);
#ifdef CONFIG_ENABLE_RFTEST
esp_err_t SetTestModeA(AppContext &ctx, const std::vector<uint8_t> &request,
                       std::vector<uint8_t> &response);
esp_err_t SetTestModeB(AppContext &ctx, const std::vector<uint8_t> &request,
                       std::vector<uint8_t> &response);
#endif

esp_err_t ManageFPGAConfig(AppContext &ctx, const std::vector<uint8_t> &request,
                           std::vector<uint8_t> &response);
esp_err_t ManagePowerAllocatorEnabled(AppContext &ctx,
                                      const std::vector<uint8_t> &request,
                                      std::vector<uint8_t> &response);
esp_err_t ManagePowerConfig(AppContext &ctx,
                            const std::vector<uint8_t> &request,
                            std::vector<uint8_t> &response);
esp_err_t SetSystemTime(AppContext &ctx, const std::vector<uint8_t> &request,
                        std::vector<uint8_t> &response);
esp_err_t GetDebugLog(AppContext &ctx, const std::vector<uint8_t> &request,
                      std::vector<uint8_t> &response);
esp_err_t ManageFeatureToggle(AppContext &ctx,
                              const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
esp_err_t EnableReleaseMode(AppContext &ctx,
                            const std::vector<uint8_t> &request,
                            std::vector<uint8_t> &response);
};  // namespace DeviceHandler

namespace DeviceAuth {
esp_err_t getPassword(uint8_t *password, size_t passwordSize);
bool validatePassword(const uint8_t *password, size_t passwordSize);
};  // namespace DeviceAuth

#endif
