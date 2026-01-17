#include "service.h"

#include "esp_log.h"

#define DEVICE_HIGH_TEMP_THRESHOLD (power_config.device_high_temp_threshold)

static const char *TAG = "IonBridgeService";

bool token_required(ServiceCommand service) {
  return service != ServiceCommand::ASSOCIATE_DEVICE;
}

bool is_service_available_at_high_temp(ServiceCommand service, bool pre_check) {
  switch (service) {
    // Device
    case ServiceCommand::RESET_DEVICE:
    // WIFI
    case ServiceCommand::SET_WIFI_SSID:
    case ServiceCommand::SET_WIFI_PASSWORD:
    case ServiceCommand::RESET_WIFI:
    case ServiceCommand::SET_WIFI_SSID_AND_PASSWORD:
    case ServiceCommand::OPERATE_WIFI_RECORD:
    // Power
    case ServiceCommand::SET_PORT_PRIORITY:
    case ServiceCommand::SET_PORT_COMPATIBILITY_SETTINGS:
    case ServiceCommand::SET_PORT_CONFIG:
    // Display
    case ServiceCommand::SET_DISPLAY_FLIP:
    case ServiceCommand::SET_DISPLAY_CONFIG:
    // ota
    case ServiceCommand::START_OTA:
      if (pre_check && service == ServiceCommand::SET_DISPLAY_CONFIG) {
        return true;
      }
      return true;
    default:
      return true;
  }
}

bool is_service_available_at_high_temp(AdminCommand service) {
  return true;
}
