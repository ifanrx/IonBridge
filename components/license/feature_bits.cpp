#include "feature_bits.h"

#include <bitset>

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_key.h"
#include "nvs_namespace.h"

static const char* TAG = "FBM";

using namespace nvs_namespace::user_data;

FeatureBitsManager& FeatureBitsManager::GetInstance() {
  static FeatureBitsManager instance;
  return instance;
}

esp_err_t FeatureBitsManager::LoadFeatureBits() {
  esp_err_t err = LICENSE.Get(nvs_keys::LICENSE_FEATURES, feature_bits_);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "Feature Bits:");
    for (int i = 0; i < feature_bits_.size(); ++i) {
      if (feature_bits_[i] != 0) {
        std::bitset<8> bits(feature_bits_[i]);
        ESP_LOGI(TAG, "Byte %d: 0x%02X (0b%s)", i, feature_bits_[i],
                 bits.to_string().c_str());
      }
    }
  }
  return err;
}

esp_err_t FeatureBitsManager::SaveFeatureBits() const {
  return LICENSE.Set(nvs_keys::LICENSE_FEATURES, feature_bits_);
}

esp_err_t FeatureBitsManager::SetFeatureBit(uint8_t feature_id, bool state) {
  if (feature_id >= feature_bits_.size() * 8) {
    return ESP_ERR_INVALID_ARG;  // Feature ID out of range
  }

  if (state) {
    feature_bits_[feature_id / 8] |= (1 << (feature_id % 8));
  } else {
    feature_bits_[feature_id / 8] &= ~(1 << (feature_id % 8));
  }

  uint8_t byte_index = feature_id / 8;
  std::bitset<8> bits(feature_bits_[byte_index]);
  ESP_LOGI(TAG, "Feature bit %d %s - Byte %d: 0x%02X (0b%s)", feature_id,
           state ? "enabled" : "disabled", byte_index,
           feature_bits_[byte_index], bits.to_string().c_str());
  return ESP_OK;
}

bool FeatureBitsManager::IsFeatureEnabled(Feature feature) const {
  uint8_t feature_id = static_cast<uint8_t>(feature);
  if (feature_id >= feature_bits_.size() * 8) {
    ESP_LOGE(TAG, "Invalid feature index: %d", feature_id);
    return false;  // Invalid feature ID
  }
  return (feature_bits_[feature_id / 8] & (1 << (feature_id % 8))) != 0;
}
