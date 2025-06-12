#include "app.h"

#include <sys/stat.h>
#include <sys/types.h>

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <utility>
#include <vector>

#include "controller.h"
#include "esp_err.h"
#include "esp_log.h"
#include "ionbridge.h"
#include "power_allocator.h"
#include "service.h"

static const char *TAG = "App";

void App::Srv(ServiceCommand command, SrvFunc func, ServiceScope scope) {
  switch (scope) {
    case ServiceScope::SERVICE_SCOPE_BLE:
      m_ble_services[command] = std::move(func);
      break;

    case ServiceScope::SERVICE_SCOPE_MQTT:
      m_mqtt_services[command] = std::move(func);
      break;

    case ServiceScope::SERVICE_SCOPE_ALL:
      m_ble_services[command] = func;              // Use copy for BLE
      m_mqtt_services[command] = std::move(func);  // Move original to MQTT
      break;
  }
}

bool App::HasSrv(uint8_t service, ServiceScope scope) {
  ServiceCommand command = static_cast<ServiceCommand>(service);
  switch (scope) {
    case ServiceScope::SERVICE_SCOPE_BLE:
      return m_ble_services.find(command) != m_ble_services.end();
    case ServiceScope::SERVICE_SCOPE_MQTT:
      return m_mqtt_services.find(command) != m_mqtt_services.end();
    case ServiceScope::SERVICE_SCOPE_ALL:
      return m_ble_services.find(command) != m_ble_services.end() &&
             m_mqtt_services.find(command) != m_mqtt_services.end();
    default:
      return false;
  }
}

ResponseStatus App::PreCheckSrv(ServiceCommand command) {
  if (!is_service_available_at_high_temp(command, true)) {
    return ResponseStatus::FLASH_NOT_WRITABLE_AT_HIGH_TEMPERATURE;
  }
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
                 command);
        return ResponseStatus::FAILURE;
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
        command);
    return ResponseStatus::FAILURE;
  }
  // Check if power related commands
  if (command >= ServiceCommand::TOGGLE_PORT_POWER &&
      command < ServiceCommand::SET_DISPLAY_INTENSITY &&
      !PowerAllocator::GetInstance().IsConfigured()) {
    ESP_LOGE(TAG,
             "Device power allocator is not configured, disabling "
             "service 0x%02x",
             command);
    return ResponseStatus::FAILURE;
  }
  return ResponseStatus::SUCCESS;
}

void App::ExecSrvFromMQTT(uint8_t service, const std::vector<uint8_t> &request,
                          std::vector<uint8_t> &response) {
  // Retrieve the appropriate handler from the service mapping
  ServiceCommand command = static_cast<ServiceCommand>(service);

  if (m_mqtt_services.find(command) == m_mqtt_services.end()) {
    ESP_LOGW(TAG, "Service not found: %d", service);
    // Return failure if the service does not exist
    response.emplace_back(ResponseStatus::FAILURE);
    return;
  }

  ResponseStatus res = PreCheckSrv(command);
  if (res != ResponseStatus::SUCCESS) {
    response.emplace_back(res);
    return;
  }

  // Obtain the corresponding handler
  auto handler = m_mqtt_services.at(command);
  esp_err_t err = handler(request, response);
  if (err != ESP_OK) {
    ESP_ERROR_COMPLAIN(err, "Failed to execute MQTT App service 0x%02x",
                       service);

    ResponseStatus res;
    if (!is_service_available_at_high_temp(command)) {
      res = ResponseStatus::FLASH_NOT_WRITABLE_AT_HIGH_TEMPERATURE;
    } else {
      res = ResponseStatus::FAILURE;
    }
    response.insert(response.begin(), res);
  } else {
    response.insert(response.begin(), ResponseStatus::SUCCESS);
  }
}

size_t App::ExecSrvFromBle(uint8_t service, uint8_t *payload,
                           size_t payloadSize, uint8_t *response,
                           size_t responseSize) {
  // Retrieve the handler from the service mapping
  ServiceCommand command = static_cast<ServiceCommand>(service);

  if (m_ble_services.find(command) == m_ble_services.end()) {
    ESP_LOGW(TAG, "Service 0x%02x not found", service);
    // Return failure if the service does not exist
    response[0] = ResponseStatus::FAILURE;
    return 1;
  }

  ResponseStatus res = PreCheckSrv(command);
  if (res != ResponseStatus::SUCCESS) {
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
    response[0] = ResponseStatus::SUCCESS;
    std::copy(data.begin(), data.end(), response + 1);
    return data.size() + 1;
  }

  ESP_ERROR_COMPLAIN(err, "Failed to execute BLE App service 0x%02x", service);

  if (!is_service_available_at_high_temp(command)) {
    response[0] = ResponseStatus::FLASH_NOT_WRITABLE_AT_HIGH_TEMPERATURE;
  } else {
    response[0] = ResponseStatus::FAILURE;
  }

  return 1;
}

void App::String(const std::string &str, std::vector<uint8_t> &response) {
  response.resize(str.size());
  std::copy(str.begin(), str.end(), response.begin());
}
