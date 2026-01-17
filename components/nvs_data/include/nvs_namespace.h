#ifndef NVS_NAMESPACE_H_
#define NVS_NAMESPACE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>
#include <utility>

#include "esp_err.h"
#include "nvs.h"
#include "nvs_default.h"
#include "nvs_key.h"

namespace nvs_wrapper {
template <typename T>
constexpr bool is_nvs_string_v =
    (std::is_same_v<std::decay_t<T>, std::string> ||
     std::is_same_v<std::decay_t<T>, const char *> ||
     std::is_same_v<std::decay_t<T>, char *>);

class Namespace {
 public:
  // Template constructor for namespace string literals with compile-time
  // validation
  template <size_t N>
  explicit Namespace(const std::string &partition_name,
                     const char (&namespace_name)[N],
                     nvs_open_mode open_mode = NVS_READWRITE)
      : handle_(0),
        partition_name_(partition_name),
        namespace_name_(namespace_name),
        open_mode_(open_mode) {
    static_assert(N <= 16 && N > 0,
                  "NVS namespace name length must be between 1 and 15 "
                  "characters (including null terminator).");
  }

  ~Namespace() {
    if (handle_ != 0) {
      nvs_close(handle_);
      handle_ = 0;
    }
  }

  // Disable copy constructor
  Namespace(const Namespace &) = delete;
  Namespace &operator=(const Namespace &) = delete;
  Namespace(Namespace &&other) noexcept
      : handle_(other.handle_),
        partition_name_(std::move(other.partition_name_)),
        namespace_name_(std::move(other.namespace_name_)),
        open_mode_(other.open_mode_) {
    other.handle_ = 0;
  }
  Namespace &operator=(Namespace &&other) noexcept {
    if (this != &other) {
      if (handle_ != 0) {
        nvs_close(handle_);
      }
      handle_ = other.handle_;
      partition_name_ = std::move(other.partition_name_);
      namespace_name_ = std::move(other.namespace_name_);
      open_mode_ = other.open_mode_;
      other.handle_ = 0;
    }
    return *this;
  }

  esp_err_t Open() { return OpenHandle(); }

  esp_err_t Open(nvs_open_mode open_mode) {
    open_mode_ = open_mode;
    return OpenHandle();
  }

  // Get a single value
  template <typename T>
  std::enable_if_t<std::is_arithmetic_v<T> && !std::is_pointer_v<T> &&
                       !std::is_array_v<T>,
                   esp_err_t>
  Get(const Key &key, T &value) const {
    size_t size = sizeof(T);
    return GetBlob(key, &value, size);
  }

  // Get a string value
  esp_err_t Get(const Key &key, std::string &value) const {
    return GetString(key, value);
  }
  esp_err_t Get(const Key &key, char *buffer, size_t &size) const {
    return GetString(key, buffer, size);
  }
  // Convenience: Get string with automatic memory management
  std::pair<esp_err_t, std::string> GetString(const Key &key) const {
    std::string result;
    esp_err_t err = GetString(key, result);
    return {err, std::move(result)};
  }

  // Get array or pointer value (updates count with actual elements read)
  template <typename T>
  std::enable_if_t<std::is_arithmetic_v<T>, esp_err_t> Get(
      const Key &key, T *buffer, size_t &count) const {
    size_t size = count * sizeof(T);
    esp_err_t err = GetBlob(key, buffer, size);
    if (err == ESP_OK) {
      count = size / sizeof(T);  // Update count to the number of elements read
    }
    return err;
  }
  // Get fixed-size buffer (accepts literals)
  template <typename T>
  std::enable_if_t<std::is_arithmetic_v<T>, esp_err_t> Get(
      const Key &key, T *buffer, const size_t &count) const {
    size_t size = count * sizeof(T);
    return GetBlob(key, buffer, size);
  }
  template <typename T, size_t N>
  std::enable_if_t<std::is_arithmetic_v<T> && !is_nvs_string_v<T>, esp_err_t>
  Get(const Key &key, T (&buffer)[N]) const {
    size_t size = N * sizeof(T);
    return GetBlob(key, buffer, size);
  }
  template <typename T, size_t N>
  esp_err_t Get(const Key &key, std::array<T, N> &arr) const {
    size_t size = N * sizeof(T);
    return GetBlob(key, arr.data(), size);
  }
  template <size_t N>
  esp_err_t Get(const Key &key, char (&buffer)[N]) const {
    size_t size = N;
    return GetString(key, buffer, size);
  }

  // Get or default value
  template <typename T>
  std::enable_if_t<std::is_arithmetic_v<T>, esp_err_t> GetOrDefault(
      const Key &key, T &value) const {
    esp_err_t err = Get(key, value);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
      value = DefaultValueStore::Get<T>(key);
      return ESP_OK;
    }
    return err;
  }
  template <typename T>
  std::enable_if_t<std::is_pointer_v<T>, esp_err_t> GetOrDefault(
      const Key &key, T buffer, size_t &size) const {
    esp_err_t err = Get(key, buffer, size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
      size_t default_count;
      using ElementType = std::remove_pointer_t<T>;
      const ElementType *default_array =
          DefaultValueStore::Get<ElementType>(key, default_count);
      if (default_array) {
        size = std::min(size / sizeof(ElementType), default_count) *
               sizeof(ElementType);
        std::memcpy(buffer, default_array, size);
        return ESP_OK;
      }
      size = 0;  // No default value found, set size to 0
      return ESP_ERR_NOT_FOUND;
    }
    return err;
  }
  template <typename T, size_t N>
  esp_err_t GetOrDefault(const Key &key, T (&buffer)[N]) const {
    esp_err_t err = Get(key, buffer);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
      size_t default_count;
      const T *default_array =
          DefaultValueStore::GetArray<T>(key, default_count);
      if (default_array) {
        size_t count = std::min(N, default_count);
        std::memcpy(buffer, default_array, count * sizeof(T));
        if (count < N) {
          std::memset(buffer + count, 0,
                      (N - count) * sizeof(T));  // Zero-fill remaining
        }
        return ESP_OK;
      }
      std::memset(buffer, 0, N * sizeof(T));  // Zero-fill if no default
      return ESP_ERR_NOT_FOUND;
    }
    return err;
  }

  // Set a single value
  template <typename T>
  std::enable_if_t<std::is_arithmetic_v<T> && !std::is_pointer_v<T> &&
                       !std::is_array_v<T>,
                   esp_err_t>
  Set(const Key &key, const T &value) const {
    size_t size = sizeof(T);
    return SetBlob(key, &value, size);
  }

  // Set a string value
  template <typename T>
  std::enable_if_t<is_nvs_string_v<T>, esp_err_t> Set(const Key &key,
                                                      const T &value) const {
    return SetString(key, value);
  }

  // Set array or pointer value
  template <typename T>
  std::enable_if_t<std::is_arithmetic_v<T>, esp_err_t> Set(const Key &key,
                                                           const T *buffer,
                                                           size_t count) const {
    return SetBlob(key, buffer, count * sizeof(T));
  }

  // Set a char array as string
  template <size_t N>
  esp_err_t Set(const Key &key, const char (&buffer)[N]) const {
    return SetString(key, buffer);
  }

  template <typename T, size_t N>
  std::enable_if_t<std::is_arithmetic_v<T> && !std::is_same_v<T, char>,
                   esp_err_t>
  Set(const Key &key, const T (&buffer)[N]) const {
    return Set(key, buffer, N);
  }
  template <typename T, size_t N>
  esp_err_t Set(const Key &key, const std::array<T, N> &arr) const {
    size_t size = arr.size() * sizeof(T);
    return SetBlob(key, arr.data(), size);
  }

  esp_err_t EraseKey(const Key &key) const;
  esp_err_t EraseAll() const;
  esp_err_t Reset();

  const std::string &GetPartitionName() const { return partition_name_; }
  const std::string &GetName() const { return namespace_name_; }

 private:
  // Low-level NVS operations (formerly in Base class)
  esp_err_t OpenHandle();
  esp_err_t OpenHandle(nvs_open_mode_t mode);
  esp_err_t GetBlob(const Key &key, void *buffer, size_t &size) const;
  esp_err_t SetBlob(const Key &key, const void *buffer, size_t size) const;
  esp_err_t GetString(const Key &key, std::string &value) const;
  esp_err_t GetString(const Key &key, char *buffer, size_t &size) const;
  esp_err_t SetString(const Key &key, const std::string &value) const;
  esp_err_t SetString(const Key &key, const char *buffer) const;
  esp_err_t Commit() const;
  esp_err_t GetSize(const Key &key, size_t &size) const;

  bool IsValid() const { return handle_ != 0; }
  bool IsWritable() const;

  nvs_handle_t handle_ = 0;
  std::string partition_name_;
  std::string namespace_name_;
  nvs_open_mode open_mode_ = NVS_READWRITE;
};
};  // namespace nvs_wrapper

namespace nvs_namespace {
using namespace nvs_wrapper;

esp_err_t Initialize();

namespace protected_data {
extern Namespace DEVICE_DATA;
extern Namespace CERTS;
extern Namespace MQTT;

extern Namespace *NAMESPACES[];

esp_err_t Initialize();
};  // namespace protected_data

namespace user_data {
extern Namespace MQTT_CUSTOM;
extern Namespace BLE;
extern Namespace DISPLAY_DATA;
extern Namespace POWER;
extern Namespace LICENSE;
extern Namespace SYSLOG;
extern Namespace FPGA;
extern Namespace WIFI;
extern Namespace OTA;
extern Namespace SYSTEM;

#ifdef CONFIG_ENABLE_RFTEST
extern Namespace TEST_MODE;
#endif

extern Namespace *NAMESPACES[];

esp_err_t Initialize();
};  // namespace user_data
};  // namespace nvs_namespace

namespace nvs_utils {
esp_err_t GetBLEAuthToken(uint8_t *token, bool always_generate_new = false);

#ifdef IONBRIDGE_NVS_DATA_HOST_TEST
void ResetBLEAuthTokenCache();
#endif

#ifdef CONFIG_ENABLE_RFTEST
esp_err_t SaveTestModeAArgs(uint32_t args[6]);
esp_err_t GetAndEraseTestModeAArgs(uint32_t args[6]);
#endif
};  // namespace nvs_utils

#endif
