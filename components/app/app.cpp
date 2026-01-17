#include "app.h"

#include <sys/stat.h>
#include <sys/types.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "controller.h"
#include "esp_err.h"
#include "esp_log.h"
#include "ionbridge.h"
#include "power_allocator.h"
#include "service.h"

static const char *TAG = "App";

constexpr int POWER_CMD_MIN_ID = 0x40;
constexpr int POWER_CMD_MAX_ID = 0x6F;

bool App::HasBLESrv(uint8_t service) {
  ServiceCommand command = static_cast<ServiceCommand>(service);
  return m_ble_services.find(command) != m_ble_services.end();
}

bool App::HasMQTTSrv(uint8_t service) {
  bool has_service = false;
  ServiceCommand command = static_cast<ServiceCommand>(service);
  has_service = m_mqtt_services.find(command) != m_mqtt_services.end();
  if (!has_service) {
    AdminCommand admin_command = static_cast<AdminCommand>(service);
    has_service = m_mqtt_admin_services.find(admin_command) !=
                  m_mqtt_admin_services.end();
  }
  return has_service;
}

pb_CommandStatus App::PreCheckSrv(ServiceCommand command) {
  if (!is_service_available_at_high_temp(command, true)) {
    return pb_CommandStatus_FLASH_NOT_WRITABLE_AT_HIGH_TEMPERATURE;
  }

  uint8_t command_var = static_cast<uint8_t>(command);
  DeviceController &controller = DeviceController::GetInstance();
  if (!controller.is_power_on()) {
    switch (command) {
      case ServiceCommand::SET_CHARGING_STRATEGY:
      case ServiceCommand::SET_STATIC_ALLOCATOR:
      case ServiceCommand::SET_DISPLAY_CONFIG:
      case ServiceCommand::SET_DISPLAY_INTENSITY:
      case ServiceCommand::SET_DISPLAY_MODE:
      case ServiceCommand::SET_DISPLAY_FLIP: {
        ESP_LOGW(TAG, "Device is power off. Service 0x%02x is disabled",
                 command_var);
        return pb_CommandStatus_FAILURE;
      }
      default:
        break;
    }
  }
  if (controller.is_waiting_for_confirmation() &&
      command == ServiceCommand::START_OTA) {
    ESP_LOGW(
        TAG,
        "Device is waiting for OTA confirmation. Service 0x%02x is disabled",
        command_var);
    return pb_CommandStatus_FAILURE;
  }
  // Check if power related commands
  if (command_var >= POWER_CMD_MIN_ID && command_var <= POWER_CMD_MAX_ID &&
      !PowerAllocator::GetInstance().IsConfigured()) {
    ESP_LOGE(TAG,
             "Device power allocator is not configured, disabling "
             "service 0x%02x",
             command_var);
    return pb_CommandStatus_FAILURE;
  }
  return pb_CommandStatus_SUCCESS;
}

size_t App::ExecBLESrv(uint8_t service, uint8_t *payload, size_t payloadSize,
                       uint8_t *response, size_t responseSize) {
  // Retrieve the handler from the service mapping
  ServiceCommand command = static_cast<ServiceCommand>(service);

  if (m_ble_services.find(command) == m_ble_services.end()) {
    ESP_LOGW(TAG, "Service 0x%02x not found", service);
    // Return failure if the service does not exist
    response[0] = pb_CommandStatus_FAILURE;
    return 1;
  }

  pb_CommandStatus res = PreCheckSrv(command);
  if (res != pb_CommandStatus_SUCCESS) {
    ESP_LOGW(TAG, "Service 0x%02x is disabled", service);
    response[0] = res;
    return 1;
  }

  // Obtain the corresponding handler
  auto handler = m_ble_services.at(command);

  // Encapsulate request and response in vectors to avoid overflow before
  // passing them to the business logic handlers
  std::vector<uint8_t> request(payload, payload + payloadSize);
  std::vector<uint8_t> data{};
  esp_err_t err = handler(request, data);

  // Check if the response size is too large; truncate if necessary
  if (data.size() > responseSize) {
#ifndef CONFIG_IDF_TARGET_LINUX
    ESP_LOGW(TAG, "Response too large: received %d, expected <= %d",
             data.size(), responseSize);
#endif
    data.resize(responseSize);
  }

  // Fill the result and return
  if (err == ESP_OK) {
    response[0] = pb_CommandStatus_SUCCESS;
    std::copy(data.begin(), data.end(), response + 1);
    return data.size() + 1;
  }

  ESP_ERROR_COMPLAIN(err, "Failed to execute BLE App service 0x%02x", service);

  if (!is_service_available_at_high_temp(command)) {
    response[0] = pb_CommandStatus_FLASH_NOT_WRITABLE_AT_HIGH_TEMPERATURE;
  } else {
    response[0] = pb_CommandStatus_FAILURE;
  }

  return 1;
}
