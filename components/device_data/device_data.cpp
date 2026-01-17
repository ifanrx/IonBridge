#include "device_data.h"

#include <algorithm>
#include <cctype>
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_key.h"
#include "nvs_namespace.h"

static const char* TAG = "DeviceData";

using namespace nvs_wrapper;
using namespace nvs_namespace;

namespace DeviceData {
namespace DeviceModel {

esp_err_t Get(Model& model) {
  std::string model_str;
  ESP_RETURN_ON_ERROR(
      protected_data::DEVICE_DATA.Get(nvs_keys::DEVICE_MODEL, model_str), TAG,
      "Failed to get device model from NVS");

  return FromString(model_str, model);
}

esp_err_t FromHardwareCode(uint8_t code, Model& model) {
  switch (code) {
    case 0x0:
      model = Model::DEVB;
      return ESP_OK;
    case 0x1:
      model = Model::PRO;
      return ESP_OK;
    case 0x2:
      model = Model::ULTRA;
      return ESP_OK;
    case 0xf0:
      model = Model::BIRD;
      return ESP_OK;
    case 0xff:
      model = Model::FAKE;
      return ESP_OK;
    default:
      ESP_LOGE(TAG, "Invalid hardware code: 0x%02x", code);
      return ESP_ERR_INVALID_ARG;
  }
}

esp_err_t FromProtobuf(pb_DeviceModel pb_model, Model& model) {
  switch (pb_model) {
    case pb_DeviceModel_DEVB:
      model = Model::DEVB;
      return ESP_OK;
    case pb_DeviceModel_PRO:
      model = Model::PRO;
      return ESP_OK;
    case pb_DeviceModel_ULTRA:
      model = Model::ULTRA;
      return ESP_OK;
    case pb_DeviceModel_BIRD:
      model = Model::BIRD;
      return ESP_OK;
    case pb_DeviceModel_FAKE:
      model = Model::FAKE;
      return ESP_OK;
    default:
      ESP_LOGE(TAG, "Invalid protobuf device model: %d", pb_model);
      return ESP_ERR_INVALID_ARG;
  }
}

esp_err_t FromString(const std::string& modelStr, Model& model) {
  std::string lowerStr = modelStr;
  std::transform(lowerStr.begin(), lowerStr.end(), lowerStr.begin(),
                 [](unsigned char c) { return std::tolower(c); });

  if (lowerStr == "devb") {
    model = Model::DEVB;
    return ESP_OK;
  }
  if (lowerStr == "pro") {
    model = Model::PRO;
    return ESP_OK;
  }
  if (lowerStr == "ultra") {
    model = Model::ULTRA;
    return ESP_OK;
  }
  if (lowerStr == "bird") {
    model = Model::BIRD;
    return ESP_OK;
  }
  if (lowerStr == "fake") {
    model = Model::FAKE;
    return ESP_OK;
  }

  ESP_LOGE(TAG, "Invalid device model string: %s", modelStr.c_str());
  return ESP_ERR_INVALID_ARG;
}

esp_err_t CheckFlag() {
  bool flag = false;  // Initialize to false
  esp_err_t err =
      protected_data::DEVICE_DATA.Get(nvs_keys::DEVICE_MODEL_SET_FLAG, flag);

  // Only return error for real failures (not including NOT_FOUND)
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
    ESP_RETURN_ON_ERROR(err, TAG, "Failed to get device model flag");
  }

  // Check flag value for both NOT_FOUND (flag=false) and successful read
  ESP_RETURN_ON_FALSE(!flag, ESP_ERR_INVALID_STATE, TAG,
                      "Device model already set via BLE");
  return ESP_OK;
}

esp_err_t Set(Model model) {
  ESP_RETURN_ON_FALSE(IsValid(model), ESP_ERR_INVALID_ARG, TAG,
                      "Invalid device model");

  const char* model_str = ToString(model);
  ESP_RETURN_ON_ERROR(protected_data::DEVICE_DATA.Set(nvs_keys::DEVICE_MODEL,
                                                      std::string(model_str)),
                      TAG, "Failed to set device model");

  ESP_LOGI(TAG, "Device model set to %s", model_str);
  return ESP_OK;
}

esp_err_t Set(pb_DeviceModel model) {
  Model device_model;
  ESP_RETURN_ON_ERROR(FromProtobuf(model, device_model), TAG,
                      "Invalid protobuf model");
  return Set(device_model);
}

esp_err_t Set(uint8_t model) {
  Model device_model;
  ESP_RETURN_ON_ERROR(FromHardwareCode(model, device_model), TAG,
                      "Invalid hardware code");
  return Set(device_model);
}

esp_err_t SetFlag() {
  ESP_RETURN_ON_ERROR(
      protected_data::DEVICE_DATA.Set(nvs_keys::DEVICE_MODEL_SET_FLAG, true),
      TAG, "Failed to set device model flag");

  ESP_LOGI(TAG, "Device model flag set");
  return ESP_OK;
}

esp_err_t ResetFlag() {
  ESP_RETURN_ON_ERROR(
      protected_data::DEVICE_DATA.Set(nvs_keys::DEVICE_MODEL_SET_FLAG, false),
      TAG, "Failed to reset BLE flag");

  ESP_LOGI(TAG, "Device model BLE flag reset");
  return ESP_OK;
}

esp_err_t ToU8(uint8_t& result) {
  Model model;
  ESP_RETURN_ON_ERROR(Get(model), TAG, "Failed to get device model");
  result = static_cast<uint8_t>(model);
  return ESP_OK;
}

esp_err_t ToPB(pb_DeviceModel& result) {
  Model model;
  ESP_RETURN_ON_ERROR(Get(model), TAG, "Failed to get device model");

  switch (model) {
    case Model::DEVB:
      result = pb_DeviceModel_DEVB;
      break;
    case Model::PRO:
      result = pb_DeviceModel_PRO;
      break;
    case Model::ULTRA:
      result = pb_DeviceModel_ULTRA;
      break;
    case Model::BIRD:
      result = pb_DeviceModel_BIRD;
      break;
    case Model::FAKE:
      result = pb_DeviceModel_FAKE;
      break;
    default:
      ESP_LOGE(TAG, "Invalid device model: %d", static_cast<int>(model));
      return ESP_ERR_INVALID_STATE;
  }
  return ESP_OK;
}

const char* ToString(Model model) {
  switch (model) {
    case Model::DEVB:
      return "devb";
    case Model::PRO:
      return "pro";
    case Model::ULTRA:
      return "ultra";
    case Model::BIRD:
      return "bird";
    case Model::FAKE:
      return "fake";
    default:
      return "unknown";
  }
}

bool IsValid(Model model) {
  switch (model) {
    case Model::DEVB:
    case Model::PRO:
    case Model::ULTRA:
    case Model::BIRD:
    case Model::FAKE:
      return true;
    default:
      return false;
  }
}

}  // namespace DeviceModel
}  // namespace DeviceData
