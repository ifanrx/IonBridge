#include "machine_info.h"

#include <string.h>
#include <sys/types.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "ionbridge.h"
#include "nvs.h"
#include "nvs_default.h"
#include "nvs_namespace.h"
#include "sdkconfig.h"
#include "version.h"

static const char* TAG = "MachineInfo";

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
      mdns_hostname_("") {}

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
  ReadMDNSHostname();  // should not be error
  ESP_RETURN_ON_ERROR(ReadHWRev(), TAG, "read_hw_rev");
  ESP_RETURN_ON_ERROR(ReadDeviceModel(), TAG, "read_device_model");
  ESP_RETURN_ON_ERROR(ReadDeviceName(), TAG, "read_device_name");
  ESP_RETURN_ON_ERROR(ReadProductFamily(), TAG, "read_product_family");
  ESP_RETURN_ON_ERROR(ReadProductColor(), TAG, "read_product_color");
  ESP_RETURN_ON_ERROR(ReadPIN(), TAG, "read_pin");
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
  char psn_buffer[17] = {0};  // Buffer for 16 characters plus a null terminator
  size_t required_size = sizeof(psn_buffer);

  ESP_RETURN_ON_ERROR(
      DeviceNVSGet(psn_buffer, &required_size, NVSKey::DEVICE_SERIAL_NUMBER),
      TAG, "Failed to get device serial number");

  if (required_size > sizeof(psn_buffer)) {
    ESP_LOGE(TAG, "Insufficient buffer size for PSN: %zu", sizeof(psn_buffer));
    return ESP_ERR_INVALID_SIZE;
  }

  psn_buffer[required_size - 1] = '\0';  // Ensure the string is null-terminated
  psn_ = std::string(psn_buffer);

  return ESP_OK;
}

esp_err_t MachineInfo::ReadMAC() {
  std::array<uint8_t, 6> ble_mac_array;
  ESP_RETURN_ON_ERROR(esp_read_mac(ble_mac_array.data(), ESP_MAC_BT), TAG,
                      "esp_read_mac");
  ble_mac_ = ble_mac_array;

  std::array<uint8_t, 6> wifi_mac_array;
  ESP_RETURN_ON_ERROR(esp_read_mac(wifi_mac_array.data(), ESP_MAC_WIFI_STA),
                      TAG, "esp_read_mac");
  wifi_mac_ = wifi_mac_array;
  return ESP_OK;
}

esp_err_t MachineInfo::ReadHWRev() {
#ifdef CONFIG_ENABLE_HW_REVISION_MODIFICATION
  if (strlen(CONFIG_HW_REVISION_VALUE) > 0 &&
      strlen(CONFIG_HW_REVISION_VALUE) < 8) {
    ESP_LOGW(TAG, "Setting HW revision to '%s'", CONFIG_HW_REVISION_VALUE);
    DeviceNVSSet(CONFIG_HW_REVISION_VALUE, NVSKey::DEVICE_HARDWARE_REV,
                 strlen(CONFIG_HW_REVISION_VALUE));
  }
#endif
  char hw_version[8] = "DEV";
  size_t hw_version_length = sizeof(hw_version);
  ESP_RETURN_ON_ERROR(
      DeviceNVSGet(hw_version, &hw_version_length, NVSKey::DEVICE_HARDWARE_REV),
      TAG, "get_device_hardware_rev");
  hwrev_ = std::string(hw_version);
  return ESP_OK;
}

esp_err_t MachineInfo::ReadDeviceModel() {
#ifdef CONFIG_ENABLE_DEVICE_MODEL_MODIFICATION
  if (strlen(CONFIG_DEVICE_MODEL_VALUE) > 0 &&
      strlen(CONFIG_DEVICE_MODEL_VALUE) < 8) {
    ESP_LOGW(TAG, "Setting device model to '%s'", CONFIG_DEVICE_MODEL_VALUE);
    DeviceNVSSet(CONFIG_DEVICE_MODEL_VALUE, NVSKey::DEVICE_MODEL,
                 strlen(CONFIG_DEVICE_MODEL_VALUE));
  }
#endif
  char model[8] = "";
  size_t length = sizeof(model);

  ESP_RETURN_ON_ERROR(DeviceNVSGet(model, &length, NVSKey::DEVICE_MODEL), TAG,
                      "get_device_model");
#ifndef CONFIG_ENABLE_DEVICE_MODEL_MODIFICATION
  if (strncmp(model, "prow", length) == 0) {
    // Override device model for pro version
    // w for white shell
    ESP_LOGW(TAG, "Correct device model from 'prow' to 'pro'");
    strncpy(model, "pro", 4);
  } else if (strncmp(model, "ultra", length) == 0) {
    // Override device model for ultra version
    // g for gray shell
    ESP_LOGW(TAG, "Correct device model from 'ultrag' to 'ultra'");
    strncpy(model, "ultra", 6);
  }
  if (strlen(model) != length) {
    // Override, write to NVS
    length = strlen(model);
    ESP_RETURN_ON_ERROR(DeviceNVSSet(model, NVSKey::DEVICE_MODEL, length), TAG,
                        "set_device_model");
  }
#endif
  device_model_ = model;
  return ESP_OK;
}

esp_err_t MachineInfo::ReadDeviceName() {
  char device_name[32] = {0};
  size_t deviceNameLen = sizeof(device_name);
  esp_err_t err =
      DeviceNVSGet(device_name, &deviceNameLen, NVSKey::DEVICE_NAME);
  if (err != ESP_OK) {
    strcpy(device_name, CONFIG_BLE_DEFAULT_DEVICE_NAME);
  }
  device_name_ = device_name;
  return ESP_OK;
}

esp_err_t MachineInfo::ReadProductFamily() {
  char product_family[8] = {0};
  size_t product_family_length = sizeof(product_family);
  // if not found in NVS, set CP02 as default
#ifdef CONFIG_ENABLE_PRODUCT_FAMILY_MODIFICATION
  esp_err_t ret = DeviceNVSGet(product_family, &product_family_length,
                               NVSKey::DEVICE_PRODUCT_FAMILY);
  // if product family is not set, set it to default
  if (ret == ESP_ERR_NVS_NOT_FOUND ||
      strncmp(product_family, CONFIG_PRODUCT_FAMILY_VALUE,
              product_family_length) != 0) {
    product_family_length =
        std::min(product_family_length, strlen(CONFIG_PRODUCT_FAMILY_VALUE));
    strncpy(product_family, CONFIG_PRODUCT_FAMILY_VALUE, product_family_length);
    // Ensure null-terminated string
    product_family[product_family_length] = '\0';
    ESP_LOGW(TAG, "Setting product family to '%s'", product_family);
    DeviceNVSSet(product_family, NVSKey::DEVICE_PRODUCT_FAMILY,
                 product_family_length);
  }
#else
  ESP_RETURN_ON_ERROR(DeviceNVSGet(product_family, &product_family_length,
                                   NVSKey::DEVICE_PRODUCT_FAMILY),
                      TAG, "get_product_family");
#endif

  product_family_ = product_family;
  return ESP_OK;
}

esp_err_t MachineInfo::ReadProductColor() {
  char product_color[8] = {0};
  size_t product_color_length = sizeof(product_color);
  // if not found in NVS, set color based on device model
  esp_err_t ret = DeviceNVSGet(product_color, &product_color_length,
                               NVSKey::DEVICE_PRODUCT_COLOR);
  if (ret == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(
        TAG,
        "Product color not found in NVS, setting based on device model: %s",
        device_model_.c_str());
    if (device_model_.compare("pro") == 0) {
      strcpy(product_color, "white");
    } else if (device_model_.compare("ultra") == 0) {
      strcpy(product_color, "gray");
    } else {
      strcpy(product_color, "unknown");
    }
    product_color_length = strlen(product_color);
    ESP_RETURN_ON_ERROR(
        DeviceNVSSet(product_color, NVSKey::DEVICE_PRODUCT_COLOR,
                     product_color_length),
        TAG, "set_product_color");
  }

  product_color_ = product_color;
  return ESP_OK;
}

esp_err_t MachineInfo::ReadPIN() {
  std::array<uint8_t, 4> pin = {0};
  std::string password_str;
  const size_t expected_length = 4;

  // Read device password
  size_t length = expected_length + 1;  // Include space for null terminator
  password_str.resize(length);
  esp_err_t err =
      DeviceNVSGet(&password_str[0], &length, NVSKey::DEVICE_PASSWORD);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get device password: %s", esp_err_to_name(err));
    return err;
  }

  // Ensure the string is null-terminated
  if (password_str.length() >= length) {
    password_str[length - 1] = '\0';
    password_str.resize(length - 1);
  }

  // Validate and convert PIN
  if (password_str.length() < expected_length) {
    ESP_LOGE(TAG, "Insufficient password length: %zu", password_str.length());
    return ESP_ERR_INVALID_SIZE;
  }

  for (size_t i = 0; i < expected_length; ++i) {
    char c = password_str[i];
    if (c >= '0' && c <= '9') {
      pin[i] = static_cast<uint8_t>(c - '0');
    } else {
      ESP_LOGE(TAG, "Invalid character in password: 0x%X, defaulting to 0",
               static_cast<uint8_t>(c));
      pin[i] = 0;  // Use 0 as the default value for non-numeric characters
    }
  }

  ESP_LOGD(TAG, "Device PIN: %d %d %d %d", pin[0], pin[1], pin[2], pin[3]);
  pin_ = pin;

  return ESP_OK;
}

esp_err_t MachineInfo::ReadVersion() {
  Version ESP32_FIRMWARE_VERSION(CONFIG_APP_PROJECT_VER);
#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
  Version MCU_FIRMWARE_VERSION(CONFIG_SW3566_FIRMWARE_SEMVER);
  Version FPGA_FIRMWARE_VERSION(CONFIG_FPGA_FIRMWARE_SEMVER);
  Version ZRLIB_VERSION(CONFIG_ZRLIB_SEMVER);
#else
  Version MCU_FIRMWARE_VERSION(0, 0, 0);  // TODO: Read actual MCU version
  Version FPGA_FIRMWARE_VERSION(0, 0, 0);
  Version ZRLIB_VERSION(0, 0, 0);
#endif

  esp32_version_ = ESP32_FIRMWARE_VERSION.toArray();
  mcu_version_ = MCU_FIRMWARE_VERSION.toArray();
  fpga_version_ = FPGA_FIRMWARE_VERSION.toArray();
  zrlib_version_ = ZRLIB_VERSION.toArray();
  return ESP_OK;
}

esp_err_t MachineInfo::ReadMDNSHostname() {
  mdns_hostname_ = "cp02-" + psn_;
  return ESP_OK;
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
  if (strcmp(product_family_.c_str(), "CP02") == 0) {
    return 0x00;
  }
  if (strcmp(product_family_.c_str(), "PA768S") == 0) {
    return 0x01;
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
