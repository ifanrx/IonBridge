#ifndef NVS_NAMESPACE_H_
#define NVS_NAMESPACE_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "esp_check.h"
#include "esp_err.h"
#include "nvs_base.h"
#include "nvs_default.h"

#define PROTECTED_DATA_NVS_PARTITION "protected_data"
#define DEVICE_DATA_NVS_NAMESPACE "device_data"
#define CERTS_NVS_NAMESPACE "certs"
#define MQTT_NVS_NAMESPACE "mqtt"

#define USER_DATA_NVS_PARTITION "nvs"
#define POWER_DATA_NVS_NAMESPACE "power"
#define DISPLAY_DATA_NVS_NAMESPACE "display_data"
#define BLE_SERVICE_NAMESPACE "BLE-SERVICE"
#define LICENSE_NVS_NAMESPACE "license"
#define SYSLOG_NVS_NAMESPACE "syslog"
#define WIFI_NAMESPACE "Wi-Fi"
#define FPGA_NAMESPACE "fpga"
#ifdef CONFIG_ENABLE_RFTEST
#define TEST_MODE_NAMESPACE "test_mode"
#endif

#define NVS_NAMESPACE_LOG_TAG "NVSNamespace"

class NVSNamespace {
 public:
  explicit NVSNamespace(const std::string &partition_name,
                        const std::string &namespace_name,
                        std::unique_ptr<NVSBase> nvs_base)
      : nvs_base_(std::move(nvs_base)),
        partition_name_(partition_name),
        namespace_name_(namespace_name) {}

  template <typename T>
  esp_err_t Get(T *value, size_t *size, NVSKey key) const {
    return nvs_base_->Get(value, size, nvs_key_to_string(key).c_str());
  }

  template <typename T>
  esp_err_t GetOrDefault(T *value, NVSKey key) const {
    return nvs_base_->GetOrDefault(value, key);
  }
  template <typename T>
  esp_err_t GetOrDefault(T *value, size_t *size, NVSKey key) const {
    return nvs_base_->GetOrDefault(value, size, key);
  }

  template <typename T>
  esp_err_t Set(const T *value, NVSKey key, size_t size) const {
    return nvs_base_->Set(value, key, size);
  }
  template <typename T>
  esp_err_t Set(const T &value, NVSKey key) const {
    return nvs_base_->Set(value, key, sizeof(T));
  }
  template <typename T>
  esp_err_t SetIfDifferent(const T &value, NVSKey key) const {
    T current_value;
    ESP_RETURN_ON_ERROR(GetOrDefault(&current_value, key),
                        NVS_NAMESPACE_LOG_TAG, "Get %s failed",
                        nvs_key_to_string(key).c_str());
    if (current_value == value) {
      return ESP_OK;
    }
    return nvs_base_->Set(value, key, sizeof(T));
  }

  esp_err_t EraseKey(NVSKey key) const {
    return nvs_base_->EraseKey(nvs_key_to_string(key).c_str());
  };

  esp_err_t EraseAll() const { return nvs_base_->EraseAll(); };

  // Factory method to open an NVSNamespace using partition name directly
  static std::unique_ptr<NVSNamespace> Open(const std::string &partition_name,
                                            const std::string &namespace_name,
                                            nvs_open_mode open_mode);

  template <typename T>
  static esp_err_t SGetOrDefault(T *value, size_t *size, NVSKey key,
                                 const std::string &partition_name,
                                 const std::string &namespace_name) {
    std::unique_ptr<NVSNamespace> ns =
        NVSNamespace::Open(partition_name, namespace_name, NVS_READWRITE);
    ESP_RETURN_ON_FALSE(ns != nullptr, ESP_FAIL, NVS_NAMESPACE_LOG_TAG,
                        "open %s namespace failed", namespace_name.c_str());
    return ns->GetOrDefault(value, size, key);
  }

  template <typename T>
  static esp_err_t SGetOrDefault(T *value, NVSKey key,
                                 const std::string &partition_name,
                                 const std::string &namespace_name) {
    std::unique_ptr<NVSNamespace> ns =
        NVSNamespace::Open(partition_name, namespace_name, NVS_READWRITE);
    ESP_RETURN_ON_FALSE(ns != nullptr, ESP_FAIL, NVS_NAMESPACE_LOG_TAG,
                        "open %s namespace failed", namespace_name.c_str());
    return ns->GetOrDefault(value, key);
  }

  template <typename T>
  static esp_err_t SGet(T *value, size_t *size, NVSKey key,
                        const std::string &partition_name,
                        const std::string &namespace_name) {
    std::unique_ptr<NVSNamespace> ns =
        NVSNamespace::Open(partition_name, namespace_name, NVS_READWRITE);
    ESP_RETURN_ON_FALSE(ns != nullptr, ESP_FAIL, NVS_NAMESPACE_LOG_TAG,
                        "open %s namespace failed", namespace_name.c_str());
    return ns->Get(value, size, key);
  }

  template <typename T>
  static esp_err_t SGet(T *value, NVSKey key, const std::string &partition_name,
                        const std::string &namespace_name) {
    std::unique_ptr<NVSNamespace> ns =
        NVSNamespace::Open(partition_name, namespace_name, NVS_READWRITE);
    ESP_RETURN_ON_FALSE(ns != nullptr, ESP_FAIL, NVS_NAMESPACE_LOG_TAG,
                        "open %s namespace failed", namespace_name.c_str());
    size_t size = sizeof(T);
    return ns->Get(value, &size, key);
  };

  template <typename T>
  static esp_err_t SSet(const T *value, NVSKey key, size_t size,
                        const std::string &partition_name,
                        const std::string &namespace_name) {
    std::unique_ptr<NVSNamespace> ns =
        NVSNamespace::Open(partition_name, namespace_name, NVS_READWRITE);
    ESP_RETURN_ON_FALSE(ns != nullptr, ESP_FAIL, NVS_NAMESPACE_LOG_TAG,
                        "open %s namespace failed", namespace_name.c_str());
    return ns->Set(value, key, size);
  }
  template <typename T>
  static esp_err_t SSet(const T &value, NVSKey key,
                        const std::string &partition_name,
                        const std::string &namespace_name) {
    std::unique_ptr<NVSNamespace> ns =
        NVSNamespace::Open(partition_name, namespace_name, NVS_READWRITE);
    ESP_RETURN_ON_FALSE(ns != nullptr, ESP_FAIL, NVS_NAMESPACE_LOG_TAG,
                        "open %s namespace failed", namespace_name.c_str());
    return ns->Set(value, key);
  }
  template <typename T>
  static esp_err_t SSetIfDifferent(const T &value, NVSKey key,
                                   const std::string &partition_name,
                                   const std::string &namespace_name) {
    std::unique_ptr<NVSNamespace> ns =
        NVSNamespace::Open(partition_name, namespace_name, NVS_READWRITE);
    ESP_RETURN_ON_FALSE(ns != nullptr, ESP_FAIL, NVS_NAMESPACE_LOG_TAG,
                        "open %s namespace failed", namespace_name.c_str());
    return ns->SetIfDifferent(value, key);
  }

  static esp_err_t SEraseKey(NVSKey key, const std::string &partition_name,
                             const std::string &namespace_name);
  static esp_err_t SEraseAll(const std::string &partition_name,
                             const std::string &namespace_name);

 private:
  std::unique_ptr<NVSBase> nvs_base_;
  std::string partition_name_, namespace_name_;
};

#define DeviceNVSGet(...)                                       \
  NVSNamespace::SGet(__VA_ARGS__, PROTECTED_DATA_NVS_PARTITION, \
                     DEVICE_DATA_NVS_NAMESPACE)

#define CertsNVSGet(...)                                        \
  NVSNamespace::SGet(__VA_ARGS__, PROTECTED_DATA_NVS_PARTITION, \
                     CERTS_NVS_NAMESPACE)

#define MQTTNVSGet(...)                                         \
  NVSNamespace::SGet(__VA_ARGS__, PROTECTED_DATA_NVS_PARTITION, \
                     MQTT_NVS_NAMESPACE)

#define BleNVSGet(...)                                     \
  NVSNamespace::SGet(__VA_ARGS__, USER_DATA_NVS_PARTITION, \
                     BLE_SERVICE_NAMESPACE)

#define BleNVSSet(...)                                     \
  NVSNamespace::SSet(__VA_ARGS__, USER_DATA_NVS_PARTITION, \
                     BLE_SERVICE_NAMESPACE)

#define DisplayNVSGetOrDefault(...)                                 \
  NVSNamespace::SGetOrDefault(__VA_ARGS__, USER_DATA_NVS_PARTITION, \
                              DISPLAY_DATA_NVS_NAMESPACE)

#define DisplayNVSSet(...)                                 \
  NVSNamespace::SSet(__VA_ARGS__, USER_DATA_NVS_PARTITION, \
                     DISPLAY_DATA_NVS_NAMESPACE)
#define DisplayNVSSetIfDifferent(...)                                 \
  NVSNamespace::SSetIfDifferent(__VA_ARGS__, USER_DATA_NVS_PARTITION, \
                                DISPLAY_DATA_NVS_NAMESPACE)

#define PowerNVSGetOrDefault(...)                                   \
  NVSNamespace::SGetOrDefault(__VA_ARGS__, USER_DATA_NVS_PARTITION, \
                              POWER_DATA_NVS_NAMESPACE)

#define PowerNVSGet(...)                                   \
  NVSNamespace::SGet(__VA_ARGS__, USER_DATA_NVS_PARTITION, \
                     POWER_DATA_NVS_NAMESPACE)

#define PowerNVSSet(...)                                   \
  NVSNamespace::SSet(__VA_ARGS__, USER_DATA_NVS_PARTITION, \
                     POWER_DATA_NVS_NAMESPACE)

#define PowerNVSEraseKey(...)                                   \
  NVSNamespace::SEraseKey(__VA_ARGS__, USER_DATA_NVS_PARTITION, \
                          POWER_DATA_NVS_NAMESPACE)

#define LicenseNVSSet(...)                                      \
  NVSNamespace::SSet(__VA_ARGS__, PROTECTED_DATA_NVS_PARTITION, \
                     LICENSE_NVS_NAMESPACE)

#define SyslogNVSGetOrDefault(...)                                  \
  NVSNamespace::SGetOrDefault(__VA_ARGS__, USER_DATA_NVS_PARTITION, \
                              SYSLOG_NVS_NAMESPACE)

#define SyslogNVSSet(...) \
  NVSNamespace::SSet(__VA_ARGS__, USER_DATA_NVS_PARTITION, SYSLOG_NVS_NAMESPACE)

#define FPGANVSGet(...) \
  NVSNamespace::SGet(__VA_ARGS__, USER_DATA_NVS_PARTITION, FPGA_NAMESPACE)
#define FPGANVSGetOrDefault(...)                                    \
  NVSNamespace::SGetOrDefault(__VA_ARGS__, USER_DATA_NVS_PARTITION, \
                              FPGA_NAMESPACE)
#define FPGANVSSet(...) \
  NVSNamespace::SSet(__VA_ARGS__, USER_DATA_NVS_PARTITION, FPGA_NAMESPACE)
#define FPGANVSEraseKey(...) \
  NVSNamespace::SEraseKey(__VA_ARGS__, USER_DATA_NVS_PARTITION, FPGA_NAMESPACE)

#define WifiNVSGet(...) \
  NVSNamespace::SGet(__VA_ARGS__, USER_DATA_NVS_PARTITION, WIFI_NAMESPACE)
#define WifiNVSSet(...) \
  NVSNamespace::SSet(__VA_ARGS__, USER_DATA_NVS_PARTITION, WIFI_NAMESPACE)
#define WifiNVSEraseKey(...) \
  NVSNamespace::SEraseKey(__VA_ARGS__, USER_DATA_NVS_PARTITION, WIFI_NAMESPACE)
#define WifiNVSEraseAll() \
  NVSNamespace::SEraseAll(USER_DATA_NVS_PARTITION, WIFI_NAMESPACE)

esp_err_t NVSGetAuthToken(uint8_t *token, bool always_generate_new = false);
esp_err_t ResetUserData();

#ifdef CONFIG_ENABLE_RFTEST
#define TestModeNVSGet(...) \
  NVSNamespace::SGet(__VA_ARGS__, USER_DATA_NVS_PARTITION, TEST_MODE_NAMESPACE)

#define TestModeNVSSet(...) \
  NVSNamespace::SSet(__VA_ARGS__, USER_DATA_NVS_PARTITION, TEST_MODE_NAMESPACE)
#define TestModeNVSEraseKey(...)                                \
  NVSNamespace::SEraseKey(__VA_ARGS__, USER_DATA_NVS_PARTITION, \
                          TEST_MODE_NAMESPACE)
namespace TestModeNVSData {
esp_err_t SaveTestModeAArgs(uint32_t args[6]);
esp_err_t GetAndEraseTestModeAArgs(uint32_t args[6]);
}  // namespace TestModeNVSData
#endif

#endif
