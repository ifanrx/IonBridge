#include "license.h"

#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

#include <cinttypes>
#include <cstdint>
#include <cstring>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "feature_bits.h"
#include "nvs_key.h"
#include "nvs_namespace.h"
#include "telemetry_task.h"

static const char *TAG = "License";

using nvs_namespace::user_data::LICENSE;

void timerCallback(void *arg) {
  ESP_LOGI(TAG, "Checking license status");
  LicenseController::GetInstance().CheckLicenseStatus();
}

esp_err_t LicenseController::CheckLicenseExpiredTime() {
  if (license_timer_handle_ != nullptr) {
    esp_timer_stop(license_timer_handle_);
    esp_timer_delete(license_timer_handle_);
    license_timer_handle_ = nullptr;
  }

  esp_timer_create_args_t timer_args = {
      .callback = &timerCallback,
      .arg = this,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "license_expiry_check_timer",
      .skip_unhandled_events = true,
  };

  const uint64_t timer_interval_us = 3 * 60 * 60 * 1e6;  // 3 hours

  esp_err_t ret = esp_timer_create(&timer_args, &license_timer_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create license expiry check timer: %s",
             esp_err_to_name(ret));
    return ret;
  }

  ret = esp_timer_start_periodic(license_timer_handle_, timer_interval_us);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start license expiry check timer: %s",
             esp_err_to_name(ret));
    esp_timer_delete(license_timer_handle_);
    return ret;
  }

  CheckLicenseStatus();

  FeatureBitsManager &fbm = FeatureBitsManager::GetInstance();
  return fbm.LoadFeatureBits();
}

void LicenseController::CheckLicenseStatus() {
  ESP_LOGD(TAG, "Checking license expired time");
  uint64_t license_expire_time = 0;
  esp_err_t ret =
      LICENSE.Get(nvs_keys::LICENSE_EXPIRE_TIME, license_expire_time);

  if (ret != ESP_OK) {
    ESP_LOGD(TAG, "No license expiry time found, skipping check");
    return;
  }

  if (license_expire_time != 0) {
    uint32_t time_ = static_cast<uint32_t>(license_expire_time);
    ESP_LOGI(TAG, "License expire time: %" PRIu32, time_);
  }

  struct timeval tv;
  gettimeofday(&tv, NULL);
  uint64_t current_time = tv.tv_sec;

  if (current_time > license_expire_time) {
    ESP_LOGW(TAG, "License has expired!");
    TelemetryTask::GetInstance().RequestLicense();
  }
}

esp_err_t LicenseController::ProcessLicenses(const pb_LicenseData *licenses,
                                             size_t count) {
  if (count == 0 || licenses == nullptr) {
    ESP_LOGE(TAG, "No licenses to process");
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "ProcessLicenses: Processing %d licenses", count);

  // Process features
  ESP_RETURN_ON_ERROR(ProcessFeatures(licenses, count), TAG,
                      "Failed to process features");

  // Save feature bits to NVS
  FeatureBitsManager &fbm = FeatureBitsManager::GetInstance();
  ESP_RETURN_ON_ERROR(fbm.SaveFeatureBits(), TAG,
                      "Failed to save feature bits to NVS");

  return SaveMinExpireTime(licenses, count);
}

esp_err_t LicenseController::ProcessFeatures(const pb_LicenseData *licenses,
                                             size_t count) {
  if (count == 0 || licenses == nullptr) {
    ESP_LOGE(TAG, "No features to process");
    return ESP_ERR_INVALID_ARG;
  }

  FeatureBitsManager &fbm = FeatureBitsManager::GetInstance();
  fbm.ClearFeatureBits();

  for (size_t i = 0; i < count; ++i) {
    const pb_LicenseData &license = licenses[i];
    ESP_LOGI(TAG, "ProcessFeatures: License %d has %d features", i,
             license.f_count);

    for (size_t j = 0; j < license.f_count; ++j) {
      uint32_t fid = license.f[j];
      ESP_LOGD(TAG, "ProcessFeatures: Feature ID %" PRIu32, fid);
      if (fid >= FEATURE_ID_MIN && fid <= FEATURE_ID_MAX) {
        uint8_t bit_index = fid - FEATURE_ID_MIN;
        ESP_LOGI(TAG,
                 "ProcessFeatures: Setting feature bit %d (ID %" PRIu32 ")",
                 bit_index, fid);
        fbm.SetFeatureBit(bit_index, true);
      } else {
        ESP_LOGD(TAG,
                 "ProcessFeatures: Feature ID %" PRIu32
                 " out of range [%" PRIu32 "-%" PRIu32 "]",
                 fid, FEATURE_ID_MIN, FEATURE_ID_MAX);
      }
    }
  }

  ESP_LOGI(TAG, "ProcessFeatures: Complete");
  return ESP_OK;
}

esp_err_t LicenseController::SaveMinExpireTime(const pb_LicenseData *licenses,
                                               size_t count) {
  if (count == 0 || licenses == nullptr) {
    ESP_LOGE(TAG, "No licenses to process for expiry time");
    return ESP_ERR_INVALID_ARG;
  }

  uint64_t min_expire_time = UINT64_MAX;

  for (size_t i = 0; i < count; ++i) {
    const pb_LicenseData &license = licenses[i];
    if (license.has_a) {
      uint64_t ts = license.a;
      if (ts < min_expire_time) {
        min_expire_time = ts;
      }
    }
  }

  if (min_expire_time != UINT64_MAX) {
    ESP_RETURN_ON_ERROR(
        LICENSE.Set(nvs_keys::LICENSE_EXPIRE_TIME, min_expire_time), TAG,
        "Failed to save license expire time");
  } else {
    ESP_RETURN_ON_ERROR(LICENSE.EraseKey(nvs_keys::LICENSE_EXPIRE_TIME), TAG,
                        "Failed to erase license expire time");
  }

  return ESP_OK;
}
