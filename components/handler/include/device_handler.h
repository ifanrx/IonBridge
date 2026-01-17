#ifndef DEVICE_HANDLER_H_
#define DEVICE_HANDLER_H_

#include <cstdint>
#include <vector>

#include "admin.pb.h"
#include "command.pb.h"
#include "esp_err.h"
#include "sdkconfig.h"

namespace DeviceHandler {
namespace FromBLE {
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
esp_err_t SetPowerState(const std::vector<uint8_t> &request,
                        std::vector<uint8_t> &response);
esp_err_t GetPowerState(const std::vector<uint8_t> &request,
                        std::vector<uint8_t> &response);
esp_err_t GetSecureBootDigest(const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
esp_err_t PingMQTTTelemetry(const std::vector<uint8_t> &request,
                            std::vector<uint8_t> &response);
esp_err_t PingHTTP(const std::vector<uint8_t> &request,
                   std::vector<uint8_t> &response);
#ifdef CONFIG_ENABLE_RFTEST
esp_err_t SetTestModeA(const std::vector<uint8_t> &request,
                       std::vector<uint8_t> &response);
esp_err_t SetTestModeB(const std::vector<uint8_t> &request,
                       std::vector<uint8_t> &response);
#endif
esp_err_t SetDisplayTest(const std::vector<uint8_t> &request,
                         std::vector<uint8_t> &response);
esp_err_t SwitchFPGAFirmware(const std::vector<uint8_t> &request,
                             std::vector<uint8_t> &response);

esp_err_t GetDebugLog(const std::vector<uint8_t> &request,
                      std::vector<uint8_t> &response);
esp_err_t EnableReleaseMode(const std::vector<uint8_t> &request,
                            std::vector<uint8_t> &response);
esp_err_t GetHardwareRevision(const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
esp_err_t GetDeviceUpTime(const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
esp_err_t GetACDCSelfTestResult(const std::vector<uint8_t> &request,
                                std::vector<uint8_t> &response);
esp_err_t RunACDCSelfTest(const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response);
};  // namespace FromBLE

namespace FromMQTT {
esp_err_t GetDevicePassword(const pb_AdminCommandRequest &request,
                            pb_AdminCommandResponse &response);
esp_err_t ManageFPGAConfig(const pb_AdminCommandRequest &request,
                           pb_AdminCommandResponse &response);
esp_err_t ManagePowerConfig(const pb_AdminCommandRequest &request,
                            pb_AdminCommandResponse &response);
esp_err_t ManageFeatureToggle(const pb_AdminCommandRequest &request,
                              pb_AdminCommandResponse &response);
esp_err_t EnableReleaseMode(const pb_CommandRequest &request,
                            pb_CommandResponse &response);
esp_err_t GetHardwareRevision(const pb_CommandRequest &request,
                              pb_CommandResponse &response);
esp_err_t ResetDevice(const pb_CommandRequest &request,
                      pb_CommandResponse &response);
esp_err_t RebootDevice(const pb_CommandRequest &request,
                       pb_CommandResponse &response);
esp_err_t GetAPVersion(const pb_CommandRequest &request,
                       pb_CommandResponse &response);
esp_err_t GetDeviceSerialNumber(const pb_CommandRequest &request,
                                pb_CommandResponse &response);
esp_err_t SetPowerState(const pb_CommandRequest &request,
                        pb_CommandResponse &response);
esp_err_t GetPowerState(const pb_CommandRequest &request,
                        pb_CommandResponse &response);
esp_err_t SetSystemTime(const pb_AdminCommandRequest &request,
                        pb_AdminCommandResponse &response);
esp_err_t PushLicense(const pb_AdminCommandRequest &request,
                      pb_AdminCommandResponse &response);
esp_err_t GetDeviceInfo(const pb_CommandRequest &request,
                        pb_CommandResponse &response);
esp_err_t SetSyslogState(const pb_AdminCommandRequest &request,
                         pb_AdminCommandResponse &response);

};  // namespace FromMQTT
};  // namespace DeviceHandler

#endif
