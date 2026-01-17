#ifndef NVS_PARTITION_H_
#define NVS_PARTITION_H_

#include <string>

#include "esp_err.h"

namespace nvs_wrapper {
esp_err_t Initialize();

namespace partition {
constexpr const char* PROTECTED_DATA_NVS_PARTITION = "protected_data";
constexpr const char* USER_DATA_NVS_PARTITION = "nvs";

inline bool ValidateName(const std::string& name) {
  return name == PROTECTED_DATA_NVS_PARTITION ||
         name == USER_DATA_NVS_PARTITION;
}
inline bool ValidateName(const char* name) {
  return ValidateName(std::string(name));
}

esp_err_t Initialize(const std::string& name, bool erase_at_failure = false);
inline esp_err_t InitializeProtectedData() {
  return Initialize(PROTECTED_DATA_NVS_PARTITION, false);
}
inline esp_err_t InitializeUserData() {
  return Initialize(USER_DATA_NVS_PARTITION, true);
}
esp_err_t ReinitalizeUserData();
esp_err_t EraseUserData();
};  // namespace partition
};  // namespace nvs_wrapper

#endif
