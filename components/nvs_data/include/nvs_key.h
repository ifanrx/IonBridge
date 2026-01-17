#ifndef NVS_KEY_H_
#define NVS_KEY_H_

#include <array>
#include <cstring>
#include <functional>
#include <string>

#include "sdkconfig.h"

namespace nvs_wrapper {
class Key {
 public:
  // Template constructor for string literals
  template <size_t N>
  explicit Key(const char (&key_str)[N]) : key_(key_str) {
    static_assert(N <= 16 && N > 0,
                  "NVS key length must be between 1 and 16 characters.");
  }

  const char* c_str() const { return key_.c_str(); }
  const std::string& str() const { return key_; }

  // Add equality operator for Key comparison (required for hash maps)
  bool operator==(const Key& other) const { return key_ == other.key_; }

 private:
  std::string key_;
};
};  // namespace nvs_wrapper

// Specialize std::hash for nvs_wrapper::Key
namespace std {
template <>
struct hash<nvs_wrapper::Key> {
  std::size_t operator()(const nvs_wrapper::Key& key) const {
    return std::hash<std::string>()(key.str());
  }
};
};  // namespace std

namespace nvs_keys {
using Key = nvs_wrapper::Key;

#define PORT_CFG_KEY(port) \
  Key { "port_config_" #port }
#define WIFI_CREDENTIAL_KEY(n) Key("credential_" #n)

// Device related keys
extern const Key DEVICE_SERIAL_NUMBER;
extern const Key DEVICE_PASSWORD;
extern const Key DEVICE_NAME;
extern const Key DEVICE_HARDWARE_REV;
extern const Key DEVICE_PRODUCT_FAMILY;
extern const Key DEVICE_PRODUCT_COLOR;
extern const Key DEVICE_AES_KEY;
extern const Key DEVICE_MODEL;
extern const Key DEVICE_WIFI_MAC;
extern const Key DEVICE_BLUETOOTH_MAC;
extern const Key DEVICE_TOKEN;
extern const Key DEVICE_MODEL_SET_FLAG;
extern const Key DEVICE_DATA_MATRIX_FLAG;
extern const Key DEVICE_SWITCH_APP;

// Certificate & MQTT keys
extern const Key CA_CERT;
extern const Key MQTT_CA_CERT;
extern const Key MQTT_CERT;
extern const Key MQTT_KEY;
extern const Key MQTT_BROKER;
extern const Key MQTT_CUSTOM_BROKER;

// System related keys
extern const Key SYSTEM_TIME;
extern const Key SYSLOG_REPORT_STATE;
#ifdef CONFIG_ENABLE_RFTEST
extern const Key TEST_MODE_A;
#endif

// FPGA related keys
extern const Key FPGA_CONFIGS;
extern const Key FPGA_POWER_CONTROL;

// License related keys
extern const Key LICENSE_FEATURES;
extern const Key LICENSE_EXPIRE_TIME;

// Display related keys
extern const Key DISPLAY_MODE;
extern const Key DISPLAY_INTENSITY;
extern const Key DISPLAY_FLIP;
extern const Key DISPLAY_ROTATION;
extern const Key IDLE_ANIMATION;

// Power & port related keys
extern const Key POWER_CONFIG;
extern const Key POWER_ALLOCATOR_ENABLED;
extern const Key POWER_PORT_PRIORITY;
inline constexpr const size_t PORT_CFG_SIZE = 8;
extern const std::array<Key, PORT_CFG_SIZE> PORT_CFG_KEYS;

// Wi-Fi related keys
extern const Key WIFI_HASH;
inline constexpr const size_t WIFI_CREDENTIALS_SIZE = 64;
extern const std::array<Key, WIFI_CREDENTIALS_SIZE> WIFI_CREDENTIAL_KEYS;

// OTA related keys
extern const Key OTA_CONFIRM_RESULT;

};  // namespace nvs_keys

#endif  // NVS_KEY_H_
