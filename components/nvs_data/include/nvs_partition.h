#ifndef NVS_PARTITION_H_
#define NVS_PARTITION_H_

#include <memory>
#include <string>

#include "esp_err.h"
#include "nvs.h"
#include "nvs_namespace.h"

class NVSPartition {
 public:
  static esp_err_t Init();
  static esp_err_t InitUserData();

  NVSPartition(const std::string& name);

  std::unique_ptr<NVSNamespace> OpenNamespace(const std::string& ns,
                                              nvs_open_mode open_mode);

  esp_err_t GetErrorCode() const { return error_code_; }
  esp_err_t EraseAll();

 private:
  static bool initialized_;
  std::string name_;
  esp_err_t error_code_ = ESP_OK;
};

#endif
