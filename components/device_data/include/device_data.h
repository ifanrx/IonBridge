#pragma once

#include <cstdint>
#include <string>

#include "data_types.pb.h"
#include "esp_err.h"

namespace DeviceData {
namespace DeviceModel {

enum class Model : uint8_t {
  DEVB = 0,
  PRO = 1,
  ULTRA = 2,
  BIRD = 0xf0,
  FAKE = 0xff
};

esp_err_t Get(Model& model);

esp_err_t FromHardwareCode(uint8_t code, Model& model);
esp_err_t FromProtobuf(pb_DeviceModel pb_model, Model& model);
esp_err_t FromString(const std::string& modelStr, Model& model);

esp_err_t Set(Model model);
esp_err_t Set(pb_DeviceModel model);
esp_err_t Set(uint8_t model);
esp_err_t CheckFlag();
esp_err_t SetFlag();
esp_err_t ResetFlag();

esp_err_t ToU8(uint8_t& result);
esp_err_t ToPB(pb_DeviceModel& result);
const char* ToString(Model model);

bool IsValid(Model model);

}  // namespace DeviceModel
}  // namespace DeviceData
