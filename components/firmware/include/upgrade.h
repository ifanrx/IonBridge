#ifndef FIRMWARE_UPGRADE_H_
#define FIRMWARE_UPGRADE_H_

#include <cstddef>
#include <functional>
#include <vector>

#include "display_animation.h"
#include "esp_err.h"
#include "firmware.h"
#include "sdkconfig.h"
#include "singleton.h"
#include "version.h"

typedef struct {
#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
  Version sw3566;
  Version fpga;
#endif
  Version esp32;
  bool force;
} FirmwareVersions;

class UpgradeManager : public Singleton<UpgradeManager> {
  // Grant access to Singleton
  friend class Singleton<UpgradeManager>;

 private:
  static constexpr size_t CERT_BUFFER_SZ = 2048;
  static constexpr size_t URL_BUFFER_SZ = 256;

  // Nested struct for internal use
  struct FirmwareInfo {
    Version version;
    Version current_version;
    FirmwareType type;
    const char *name;
    OTAStage ota_stage;
    const char *product_family = nullptr;
    const char *model = nullptr;
  };

  // Instance state
  bool upgrading = false;
  bool downloading = false;
  char cert_buffer[CERT_BUFFER_SZ];
  char url_buffer[URL_BUFFER_SZ];

  // Instance storage for version data
  FirmwareVersions new_version_storage;

  // Private constructor for singleton pattern
  UpgradeManager();
  ~UpgradeManager() = default;

  // Helper methods
  std::vector<FirmwareInfo> GetFirmwareList(
      const FirmwareVersions &new_version);
  void LogVersions(const FirmwareVersions &new_version);
  esp_err_t LoadCertificate();
  esp_err_t ProcessFirmware(const FirmwareInfo &fw, bool force);

  // Common operation execution with hooks
  esp_err_t ExecuteOperation(bool &operation_flag, const char *operation_name,
                             std::function<void()> pre_process,
                             std::function<void()> on_success,
                             std::function<void(esp_err_t)> on_error);

 public:
  esp_err_t StartUpgrade(const FirmwareVersions &new_version);
  esp_err_t StartDownload(const FirmwareVersions &new_version);

  // Methods called by task wrappers
  esp_err_t ExecuteUpgrade();
  esp_err_t ExecuteDownload();
  esp_err_t ExecuteProgrammerDownload(Version version,
                                      const char *product_family);
};

#endif  // FIRMWARE_UPGRADE_H_
