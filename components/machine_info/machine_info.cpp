#include "machine_info.h"

#include <sys/types.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <memory>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "ionbridge.h"
#include "nvs.h"
#include "nvs_key.h"
#include "nvs_namespace.h"
#include "sdkconfig.h"
#include "utils.h"
#include "version.h"

static const char* TAG = "MachineInfo";

using nvs_namespace::protected_data::DEVICE_DATA;
using nvs_namespace::user_data::SYSTEM;

MachineInfo& MachineInfo::GetInstance() {
  static MachineInfo instance;
  return instance;
}

MachineInfo::MachineInfo()
    : psn_("UNK"),
      ble_mac_{},
      wifi_mac_{},
      hwrev_("UNK"),
      device_model_("UNK"),
      product_family_("UNK"),
      pin_{},
      esp32_version_{},
      mcu_version_{},
      fpga_version_{},
      zrlib_version_{},
      country_code_{},
      mdns_hostname_(""),
      system_time_(0) {}

const std::string& MachineInfo::GetPSN() const { return psn_; }

const std::array<uint8_t, 6>& MachineInfo::GetBleMac() const {
  return ble_mac_;
}

const std::array<uint8_t, 6>& MachineInfo::GetWifiMac() const {
  return wifi_mac_;
}

const std::string& MachineInfo::GetHwRev() const { return hwrev_; }

const std::string& MachineInfo::GetDeviceModel() const { return device_model_; }

const std::string& MachineInfo::GetDeviceName() const { return device_name_; }

const std::string& MachineInfo::GetProductFamily() const {
  return product_family_;
}

const std::array<uint8_t, 4>& MachineInfo::GetPin() const { return pin_; }

const std::array<uint8_t, 3>& MachineInfo::GetESP32Version() const {
  return esp32_version_;
}

const std::array<uint8_t, 3>& MachineInfo::GetMCUVersion() const {
  return mcu_version_;
}

const std::array<uint8_t, 3>& MachineInfo::GetFPGAVersion() const {
  return fpga_version_;
}

const std::array<uint8_t, 3>& MachineInfo::GetZRLIBVersion() const {
  return zrlib_version_;
}

bool MachineInfo::IsInChina() const {
  return (country_code_ == WIFI_COUNTRY_CN ||
          country_code_ == WIFI_COUNTRY_UNKNOWN);
}

bool MachineInfo::IsValidCountryCode() const {
  return !(country_code_ == WIFI_COUNTRY_UNKNOWN);
}

esp_err_t MachineInfo::SetCountryCode(std::string country_code) {
  country_code_ = (country_code.find_first_not_of('\0') == std::string::npos)
                      ? WIFI_COUNTRY_UNKNOWN
                      : country_code;
  return ESP_OK;
}

const std::string& MachineInfo::GetCountryCode() const { return country_code_; }

const std::string& MachineInfo::GetMDNSHostname() const {
  return mdns_hostname_;
}

const uint64_t& MachineInfo::GetSystemTime() const { return system_time_; }

// Formats the version number into "a.b.c" format
std::string MachineInfo::FormatVersion(
    const std::array<uint8_t, 3>& version) const {
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%d.%d.%d", static_cast<int>(version[0]),
           static_cast<int>(version[1]), static_cast<int>(version[2]));
  return std::string(buffer);
}

esp_err_t MachineInfo::LoadFromStorage() {
  ESP_ERROR_COMPLAIN(ReadVersion(), "read_version");
  ESP_ERROR_COMPLAIN(ReadMAC(), "read_mac");

  // Read from NVS, return on error instead of complaining to prevent
  // corruption
  ESP_RETURN_ON_ERROR(ReadPSN(), TAG, "read_psn");
  ESP_RETURN_ON_ERROR(ReadHWRev(), TAG, "read_hw_rev");
  ESP_RETURN_ON_ERROR(ReadDeviceModel(), TAG, "read_device_model");
  ESP_RETURN_ON_ERROR(ReadDeviceName(), TAG, "read_device_name");
  ESP_RETURN_ON_ERROR(ReadProductFamily(), TAG, "read_product_family");
  ReadMDNSHostname();  // should not be error
  ESP_RETURN_ON_ERROR(ReadProductColor(), TAG, "read_product_color");
  ESP_RETURN_ON_ERROR(ReadPIN(), TAG, "read_pin");
  ESP_RETURN_ON_ERROR(ReadSystemTime(), TAG, "read_system_time");
  return ESP_OK;
}

void MachineInfo::PrintInfo() const {
  ESP_LOGI(TAG, "PSN: %s", psn_.c_str());
  ESP_LOGI(TAG, "BLE MAC: %s", FORMAT_MAC(ble_mac_));
  ESP_LOGI(TAG, "Wi-Fi MAC: %s", FORMAT_MAC(wifi_mac_));
  ESP_LOGI(TAG, "Hardware Revision: %s", hwrev_.c_str());
  ESP_LOGI(TAG, "Device Model: %s", device_model_.c_str());
  ESP_LOGI(TAG, "Device Name: %s", device_name_.c_str());
  ESP_LOGI(TAG, "Product Family: %s", product_family_.c_str());
  ESP_LOGI(TAG, "Product Color: %s", product_color_.c_str());
  ESP_LOGI(TAG, "PIN: %d %d %d %d", pin_[0], pin_[1], pin_[2], pin_[3]);
  ESP_LOGI(TAG, "ESP32: %s", FormatVersion(esp32_version_).c_str());
  ESP_LOGI(TAG, "MCU: %s", FormatVersion(mcu_version_).c_str());
#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
  ESP_LOGI(TAG, "FPGA: %s", FormatVersion(fpga_version_).c_str());
  ESP_LOGI(TAG, "ZRLib: %s", FormatVersion(zrlib_version_).c_str());
#endif
  ESP_LOGI(TAG, "Current Country: %s", country_code_.c_str());
  ESP_LOGI(TAG, "mDNS Hostname: %s", mdns_hostname_.c_str());
}

std::string MachineInfo::FormatAsHTMLTable() const {
  std::string html;
  html.reserve(1024);  // Pre-allocate to reduce reallocations
  html.append("<table>");

  // Helper function to add a table row
  auto addRow = [&html](const std::string& field, const std::string& value) {
    html.append("<tr><th>");
    html.append(field);
    html.append("</th><td>");
    html.append(value);
    html.append("</td></tr>");
  };

  addRow("PSN", psn_);
  addRow("BLE MAC", FORMAT_MAC(ble_mac_));
  addRow("Wi-Fi MAC", FORMAT_MAC(wifi_mac_));
  addRow("Hardware Revision", hwrev_);
  addRow("Device Model", device_model_);
  addRow("Device Name", device_name_);
  addRow("Product Family", product_family_);
  addRow("Product Color", product_color_);

  // Format PIN as space-separated values
  std::string pinStr;
  pinStr.reserve(8);
  for (int i = 0; i < 4; ++i) {
    pinStr += std::to_string(pin_[i]);
    if (i < 3) pinStr += " ";
  }
  addRow("PIN", pinStr);

  addRow("ESP32 Version", FormatVersion(esp32_version_));
  addRow("MCU Version", FormatVersion(mcu_version_));
  addRow("FPGA Version", FormatVersion(fpga_version_));
  addRow("ZRLib Version", FormatVersion(zrlib_version_));
  addRow("Current Country", country_code_);
  addRow("mDNS Hostname", mdns_hostname_);

  html.append("</table>");
  return html;
}

esp_err_t MachineInfo::ReadPSN() {
  // Read PSN (Product Serial Number)
  ESP_RETURN_ON_ERROR(DEVICE_DATA.Get(nvs_keys::DEVICE_SERIAL_NUMBER, psn_),
                      TAG, "Failed to get device serial number");

  if (psn_.length() > 16) {
#ifndef CONFIG_IDF_TARGET_LINUX
    ESP_LOGE(TAG, "Invalid PSN size: %d", psn_.length());
#endif
    psn_.clear();
    return ESP_ERR_INVALID_SIZE;
  }
  return ESP_OK;
}

esp_err_t MachineInfo::ReadMAC() {
  ESP_RETURN_ON_ERROR(esp_read_mac(ble_mac_.data(), ESP_MAC_BT), TAG,
                      "esp_read_mac");
  ESP_RETURN_ON_ERROR(esp_read_mac(wifi_mac_.data(), ESP_MAC_WIFI_STA), TAG,
                      "esp_read_mac");
  return ESP_OK;
}

esp_err_t MachineInfo::ReadHWRev() {
  ESP_RETURN_ON_ERROR(DEVICE_DATA.Get(nvs_keys::DEVICE_HARDWARE_REV, hwrev_),
                      TAG, "get_device_hardware_rev");
#ifdef CONFIG_ENABLE_HW_REVISION_MODIFICATION
  if (strlen(CONFIG_HW_REVISION_VALUE) != 0 &&
      strlen(CONFIG_HW_REVISION_VALUE) < 8 &&
      hwrev_.compare(CONFIG_HW_REVISION_VALUE) != 0) {
    ESP_LOGW(TAG, "Setting HW revision to '%s'", CONFIG_HW_REVISION_VALUE);
    hwrev_ = CONFIG_HW_REVISION_VALUE;
    DEVICE_DATA.Set(nvs_keys::DEVICE_HARDWARE_REV, hwrev_);
  }
#endif
  return ESP_OK;
}

esp_err_t MachineInfo::ReadDeviceModel() {
  ESP_RETURN_ON_ERROR(DEVICE_DATA.Get(nvs_keys::DEVICE_MODEL, device_model_),
                      TAG, "get_device_model");
#ifdef CONFIG_ENABLE_DEVICE_MODEL_MODIFICATION
  if (strlen(CONFIG_DEVICE_MODEL_VALUE) != 0 &&
      strlen(CONFIG_DEVICE_MODEL_VALUE) < 8 &&
      device_model_.compare(CONFIG_DEVICE_MODEL_VALUE) != 0) {
    ESP_LOGW(TAG, "Setting device model to '%s'", CONFIG_DEVICE_MODEL_VALUE);
    device_model_ = CONFIG_DEVICE_MODEL_VALUE;
    DEVICE_DATA.Set(nvs_keys::DEVICE_MODEL, device_model_);
  }
#endif

  bool model_corrected = false;
  if (device_model_.compare("prow") == 0) {
    // Override device model for pro version
    // w for white shell
    ESP_LOGW(TAG, "Correct device model from 'prow' to 'pro'");
    device_model_ = "pro";
    model_corrected = true;
  } else if (device_model_.compare("ultrag") == 0) {
    // Override device model for ultra version
    // g for gray shell
    ESP_LOGW(TAG, "Correct device model from 'ultrag' to 'ultra'");
    device_model_ = "ultra";
    model_corrected = true;
  }
  if (model_corrected) {
    // Override, write to NVS
    ESP_RETURN_ON_ERROR(DEVICE_DATA.Set(nvs_keys::DEVICE_MODEL, device_model_),
                        TAG, "set_device_model");
  }
  return ESP_OK;
}

esp_err_t MachineInfo::ReadDeviceName() {
  device_name_.clear();
  esp_err_t err = DEVICE_DATA.Get(nvs_keys::DEVICE_NAME, device_name_);
  if (err != ESP_OK) {
    device_name_ = CONFIG_BLE_DEFAULT_DEVICE_NAME;
  }
  return ESP_OK;
}

esp_err_t MachineInfo::ReadProductFamily() {
  esp_err_t ret =
      DEVICE_DATA.Get(nvs_keys::DEVICE_PRODUCT_FAMILY, product_family_);

#if !defined(CONFIG_ENABLE_PRODUCT_FAMILY_MODIFICATION) || \
    !defined(CONFIG_PRODUCT_FAMILY_VALUE)
  ESP_RETURN_ON_ERROR(ret, TAG, "get_product_family");
#else
  // Handle other errors first (not ESP_ERR_NVS_NOT_FOUND)
  if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
    ESP_RETURN_ON_ERROR(ret, TAG, "get_product_family");
  }

  // Set to empty string if not found, so we can focus on string comparison
  if (ret == ESP_ERR_NVS_NOT_FOUND) {
    product_family_.clear();
  }

  // Verify config value exists and is valid
  if (strlen(CONFIG_PRODUCT_FAMILY_VALUE) == 0) {
    ESP_LOGE(TAG, "CONFIG_PRODUCT_FAMILY_VALUE is empty");
    return ESP_ERR_INVALID_ARG;
  }

  // Simple string comparison - update if different from config
  if (product_family_.compare(CONFIG_PRODUCT_FAMILY_VALUE) != 0) {
    ESP_LOGW(TAG, "Setting product family to '%s'",
             CONFIG_PRODUCT_FAMILY_VALUE);
    product_family_ = CONFIG_PRODUCT_FAMILY_VALUE;
    ESP_RETURN_ON_ERROR(
        DEVICE_DATA.Set(nvs_keys::DEVICE_PRODUCT_FAMILY, product_family_), TAG,
        "set_product_family");
  }
#endif

  return ESP_OK;
}

esp_err_t MachineInfo::ReadProductColor() {
  // if not found in NVS, set color based on device model
  esp_err_t ret =
      DEVICE_DATA.Get(nvs_keys::DEVICE_PRODUCT_COLOR, product_color_);
  if (ret == ESP_ERR_NVS_NOT_FOUND && product_family_.compare("CP02") == 0) {
    ESP_LOGW(TAG,
             "Product color not found in NVS, setting based on CP02 device "
             "model: %s",
             device_model_.c_str());
    if (device_model_.compare("pro") == 0) {
      product_color_ = "white";
    } else if (device_model_.compare("ultra") == 0) {
      product_color_ = "gray";
    } else {
      product_color_ = "unknown";
    }
    ESP_RETURN_ON_ERROR(
        DEVICE_DATA.Set(nvs_keys::DEVICE_PRODUCT_COLOR, product_color_), TAG,
        "set_product_color");
    ret = ESP_OK;
  }

  // Validate and correct color for CP02 devices based on device model
  if (product_family_.compare("CP02") == 0) {
    std::string expected_color;
    if (device_model_.compare("pro") == 0) {
      expected_color = "white";
    } else if (device_model_.compare("ultra") == 0) {
      expected_color = "gray";
    }

    if (!expected_color.empty() &&
        product_color_.compare(expected_color) != 0) {
      ESP_LOGW(TAG,
               "Correcting product color from '%s' to '%s' for device model "
               "'%s'",
               product_color_.c_str(), expected_color.c_str(),
               device_model_.c_str());
      product_color_ = expected_color;
      ESP_RETURN_ON_ERROR(
          DEVICE_DATA.Set(nvs_keys::DEVICE_PRODUCT_COLOR, product_color_), TAG,
          "set_product_color");
    }
  }

  return ret;
}

esp_err_t MachineInfo::ReadPIN() {
  std::string password;
  ESP_RETURN_ON_ERROR(DEVICE_DATA.Get(nvs_keys::DEVICE_PASSWORD, password), TAG,
                      "Failed to get device password");

  if (password.length() < 4) {
    return ESP_ERR_INVALID_SIZE;
  }

  // Convert password to PIN digits
  for (int i = 0; i < 4; i++) {
    pin_[i] = std::isdigit(password[i]) ? password[i] - '0' : 0;
  }

  ESP_LOGD(TAG, "Device PIN: %d %d %d %d", pin_[0], pin_[1], pin_[2], pin_[3]);
  return ESP_OK;
}

esp_err_t MachineInfo::ReadVersion() {
  Version ESP32_FIRMWARE_VERSION(CONFIG_APP_PROJECT_VER);
#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
  Version MCU_FIRMWARE_VERSION(CONFIG_SW3566_FIRMWARE_SEMVER);
  Version FPGA_FIRMWARE_VERSION(CONFIG_FPGA_FIRMWARE_SEMVER);
  Version ZRLIB_VERSION(CONFIG_ZRLIB_SEMVER);
#else
  Version MCU_FIRMWARE_VERSION;  // TODO: Read actual MCU version
  Version FPGA_FIRMWARE_VERSION;
  Version ZRLIB_VERSION;
#endif

  esp32_version_ = ESP32_FIRMWARE_VERSION.ToArray();
  mcu_version_ = MCU_FIRMWARE_VERSION.ToArray();
  fpga_version_ = FPGA_FIRMWARE_VERSION.ToArray();
  zrlib_version_ = ZRLIB_VERSION.ToArray();
  return ESP_OK;
}

esp_err_t MachineInfo::ReadMDNSHostname() {
  std::string lower_product_family;
  lower_product_family.reserve(product_family_.length());
  for (char c : product_family_) {
    lower_product_family += tolower(c);
  }
  mdns_hostname_ = lower_product_family + "-" + psn_;
  return ESP_OK;
}

esp_err_t MachineInfo::ReadSystemTime() {
  ESP_RETURN_ON_ERROR(SYSTEM.Get(nvs_keys::SYSTEM_TIME, system_time_), TAG,
                      "Failed to get system time");
  return ESP_OK;
}

esp_err_t MachineInfo::SetSystemTime(uint64_t system_time) {
  sync_system_time(system_time);
  ESP_RETURN_ON_ERROR(SYSTEM.Set(nvs_keys::SYSTEM_TIME, system_time), TAG,
                      "Failed to set system time");
  return ReadSystemTime();
}

uint8_t MachineInfo::GetDeviceModelEnumVal() const {
  if (device_model_.compare("pro") == 0) {
    return 0x01;
  }
  if (device_model_.compare("ultra") == 0) {
    return 0x02;
  }
  if (device_model_.compare("bird") == 0) {
    return 0xF0;
  }
  if (device_model_.compare("fake") == 0) {
    return 0xFF;
  }
  return 0x00;  // Development Board
}

uint8_t MachineInfo::GetProductFamilyEnumVal() const {
  if (product_family_.compare("CP02") == 0) {
    return 0x00;
  }
  if (product_family_.compare("PA768s") == 0) {
    return 0x01;
  }
  if (product_family_.compare("CP02s") == 0) {
    return 0x02;
  }
  return 0xFF;  // Unknown product family
}

uint8_t MachineInfo::GetProductColorEnumVal() const {
  if (product_color_.compare("white") == 0) {
    return 0x01;
  }
  if (product_color_.compare("gray") == 0) {
    return 0x02;
  }
  if (product_color_.compare("black") == 0) {
    return 0x03;
  }
  return 0x00;  // No product color
}

