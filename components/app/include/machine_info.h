#ifndef MACHINE_INFO_H
#define MACHINE_INFO_H

#include <array>
#include <cstdint>
#include <mutex>
#include <string>

#include "esp_err.h"

class MachineInfo {
 public:
  // Retrieve singleton instance
  static MachineInfo& GetInstance();

  // Delete copy constructor and assignment operator
  MachineInfo(const MachineInfo&) = delete;
  MachineInfo& operator=(const MachineInfo&) = delete;

  // Public member variable access methods
  const std::string& GetPSN() const;
  const std::array<uint8_t, 6>& GetBleMac() const;
  const std::array<uint8_t, 6>& GetWifiMac() const;
  const std::string& GetHwRev() const;
  const std::string& GetDeviceModel() const;
  const std::string& GetDeviceName() const;
  const std::string& GetProductFamily() const;
  const std::string& GetProductColor() const;
  const std::array<uint8_t, 4>& GetPin() const;
  const std::array<uint8_t, 3>& GetESP32Version() const;
  const std::array<uint8_t, 3>& GetMCUVersion() const;
  const std::array<uint8_t, 3>& GetFPGAVersion() const;
  const std::array<uint8_t, 3>& GetZRLIBVersion() const;
  const std::string& GetCountryCode() const;
  bool IsChina() const;

  uint8_t GetDeviceModelEnumVal() const;
  uint8_t GetProductFamilyEnumVal() const;
  uint8_t GetProductColorEnumVal() const;

  // Formatting methods
  std::string FormatVersion(const std::array<uint8_t, 3>& version) const;

  // Initialization method
  esp_err_t LoadFromStorage();  // Implement as needed, e.g., load from NVS
  esp_err_t SetCountryCode(std::string country_code);

  void PrintInfo() const;
  std::string FormatAsHTMLTable() const;

 private:
  // Private constructor
  MachineInfo();

  // Member variables
  std::string psn_;                       // Machine Serial Number
  std::array<uint8_t, 6> ble_mac_;        // BLE MAC Address
  std::array<uint8_t, 6> wifi_mac_;       // Wi-Fi MAC Address
  std::string hwrev_;                     // Hardware Revision
  std::string device_model_;              // Device Model
  std::string device_name_;               // Device Name
  std::string product_family_;            // Product Family
  std::string product_color_;             // Product Color
  std::array<uint8_t, 4> pin_;            // PIN
  std::array<uint8_t, 3> esp32_version_;  // ESP32 Version
  std::array<uint8_t, 3> mcu_version_;    // MCU Version
  std::array<uint8_t, 3> fpga_version_;   // FPGA Version
  std::array<uint8_t, 3> zrlib_version_;  // ZRLib Version
  std::string country_code_;           // Current Country Code

  // Mutex to ensure thread safety (if accessed by multiple threads)
  mutable std::mutex mutex_;

  // Helper methods
  esp_err_t ReadPSN();
  esp_err_t ReadMAC();
  esp_err_t ReadHWRev();
  esp_err_t ReadDeviceModel();
  esp_err_t ReadDeviceName();
  esp_err_t ReadProductFamily();
  esp_err_t ReadProductColor();
  esp_err_t ReadPIN();
  esp_err_t ReadVersion();
};

#endif  // MACHINE_INFO_H
