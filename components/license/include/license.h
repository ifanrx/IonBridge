#ifndef H_LICENSE_
#define H_LICENSE_

#include "data_types.pb.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "singleton.h"

class LicenseController : public Singleton<LicenseController> {
  friend class Singleton<LicenseController>;

  // FEATURE_COUNT = 256, feature range is 10000 ~ 10255
  static constexpr uint32_t FEATURE_ID_MIN = 10000;
  static constexpr uint32_t FEATURE_ID_MAX = 10255;
  static constexpr uint32_t FEATURE_COUNT = 256;

  esp_timer_handle_t license_timer_handle_;

  esp_err_t ProcessFeatures(const pb_LicenseData* licenses, size_t count);
  esp_err_t SaveMinExpireTime(const pb_LicenseData* licenses, size_t count);

 public:
  esp_err_t CheckLicenseExpiredTime();
  void CheckLicenseStatus();

  esp_err_t ProcessLicenses(const pb_LicenseData* licenses, size_t count);
};

#endif  // !H_LICENSE_
