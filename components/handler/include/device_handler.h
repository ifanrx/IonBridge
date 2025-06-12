#ifndef DEVICE_HANDLER_H_
#define DEVICE_HANDLER_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "esp_err.h"
#include "sdkconfig.h"

namespace DeviceHandler {
esp_err_t AssociateDevice(const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
esp_err_t ResetDevice(const std::vector<uint8_t> &request,
                      std::vector<uint8_t> &response);
esp_err_t RebootDevice(const std::vector<uint8_t> &request,
                       std::vector<uint8_t> &response);
esp_err_t GetAPVersion(const std::vector<uint8_t> &request,
                       std::vector<uint8_t> &response);
#ifdef CONFIG_MCU_MODEL_SW3566
esp_err_t GetBPVersion(const std::vector<uint8_t> &request,
                       std::vector<uint8_t> &response);
esp_err_t GetFPGAVersion(const std::vector<uint8_t> &request,
                         std::vector<uint8_t> &response);
esp_err_t GetZRLIBVersion(const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
#endif
esp_err_t GetDeviceSerialNumber(const std::vector<uint8_t> &request,
                                std::vector<uint8_t> &response);
esp_err_t GetDeviceUpTime(const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
esp_err_t SwitchDevice(const std::vector<uint8_t> &request,
                       std::vector<uint8_t> &response);
esp_err_t GetDeviceSwitch(const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
esp_err_t GetDeviceModel(const std::vector<uint8_t> &request,
                         std::vector<uint8_t> &response);
esp_err_t GetSecureBootDigest(const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
esp_err_t PingMQTTTelemetry(const std::vector<uint8_t> &request,
                            std::vector<uint8_t> &response);
esp_err_t PushLicense(const std::vector<uint8_t> &request,
                      std::vector<uint8_t> &response);
esp_err_t PingHTTP(const std::vector<uint8_t> &request,
                   std::vector<uint8_t> &response);
esp_err_t GetDeviceInfo(const std::vector<uint8_t> &request,
                        std::vector<uint8_t> &response);
esp_err_t SetSyslogState(const std::vector<uint8_t> &request,
                         std::vector<uint8_t> &response);
esp_err_t GetDevicePassword(const std::vector<uint8_t> &request,
                            std::vector<uint8_t> &response);

esp_err_t ManageFPGAConfig(const std::vector<uint8_t> &request,
                           std::vector<uint8_t> &response);
esp_err_t ManagePowerAllocatorEnabled(const std::vector<uint8_t> &request,
                                      std::vector<uint8_t> &response);
esp_err_t ManagePowerConfig(const std::vector<uint8_t> &request,
                            std::vector<uint8_t> &response);
esp_err_t SetSystemTime(const std::vector<uint8_t> &request,
                        std::vector<uint8_t> &response);
esp_err_t GetDebugLog(const std::vector<uint8_t> &request,
                      std::vector<uint8_t> &response);
esp_err_t ManageFeatureToggle(const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
esp_err_t EnableReleaseMode(const std::vector<uint8_t> &request,
                            std::vector<uint8_t> &response);
esp_err_t GetHardwareRevision(const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
};  // namespace DeviceHandler

namespace DeviceAuth {
esp_err_t getPassword(uint8_t *password, size_t passwordSize);
bool validatePassword(const uint8_t *password, size_t passwordSize);
};  // namespace DeviceAuth

#endif
