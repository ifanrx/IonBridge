#ifndef NVS_DEFAULT_H_
#define NVS_DEFAULT_H_

#include <type_traits>
#include <unordered_map>

#include "nvs_key.h"

namespace nvs_wrapper {
class DefaultValue {
  const void *ptr;
  size_t size;

 public:
  // Template constructor for single values
  template <typename T>
  DefaultValue(const T &value)
      : ptr(static_cast<const void *>(&value)), size(sizeof(T)) {}
  // Template constructor for arrays
  template <typename T, size_t N>
  DefaultValue(const T (&array)[N])
      : ptr(static_cast<const void *>(array)), size(sizeof(T) * N) {}
  // Manual constructor
  DefaultValue(const void *ptr, size_t size) : ptr(ptr), size(size) {}

  template <typename T>
  T as() const {
    return *static_cast<const T *>(ptr);
  }

  template <typename T>
  const T *as_array(size_t &count) const {
    count = size / sizeof(T);
    return static_cast<const T *>(ptr);
  }
};

class DefaultValueStore {
 public:
  // Unified getter method with template parameter
  template <typename T>
  static T Get(const Key &key) {
    auto it = DEFAULT_VALUES.find(key);
    if (it != DEFAULT_VALUES.end()) {
      T result = it->second.as<T>();
      return result;
    }
    return GetFallbackDefault<T>();
  }

  // Array getter method
  template <typename T>
  static const T *GetArray(const Key &key, size_t &count) {
    auto it = DEFAULT_VALUES.find(key);
    if (it != DEFAULT_VALUES.end()) {
      return it->second.as_array<T>(count);
    }
    count = 0;
    return nullptr;
  }

 private:
  static const std::unordered_map<Key, DefaultValue> DEFAULT_VALUES;

  // Default value provider for different types
  template <typename T>
  static T GetFallbackDefault() {
    if constexpr (std::is_integral_v<T>) {
      return T{0};
    } else if constexpr (std::is_same_v<T, bool>) {
      return false;
    } else {
      static_assert(sizeof(T) == 0, "Unsupported type");
    }
  }
};
};  // namespace nvs_wrapper
#endif
