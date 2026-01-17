#ifndef FEATURE_BITS_H_
#define FEATURE_BITS_H_

#include <array>
#include <cstdint>

#include "esp_err.h"

using FeatureBits = std::array<uint8_t, 32>;

enum Feature : uint8_t {
  MQTT_BROKER_CONFIG = 0,
  METRICS_JSON_PD_STATUS = 1,
};

class FeatureBitsManager {
 public:
  // Retrieve singleton instance
  static FeatureBitsManager& GetInstance();

  // Delete copy constructor and assignment operator
  FeatureBitsManager(const FeatureBitsManager&) = delete;
  FeatureBitsManager& operator=(const FeatureBitsManager&) = delete;

  // Public member variable access methods
  const FeatureBits& GetFeatureBits() const { return feature_bits_; }
  esp_err_t SetFeatureBit(uint8_t feature_id, bool state);
  bool IsFeatureEnabled(Feature feature) const;
  void ClearFeatureBits() { feature_bits_.fill(0); }
  esp_err_t LoadFeatureBits();  // Implement as needed, e.g., load from NVS
  esp_err_t SaveFeatureBits() const;

 private:
  // Private constructor
  FeatureBitsManager() { ClearFeatureBits(); }

  // Member variable
  FeatureBits feature_bits_;
};

#endif  // FEATURE_BITS_H_
