#ifndef APP_H_
#define APP_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller.h"
#include "esp_err.h"
#include "power_allocator.h"
#include "service.h"

#ifdef __cplusplus
extern "C" {
#endif

enum ServiceScope {
  SERVICE_SCOPE_BLE,
  SERVICE_SCOPE_MQTT,
  SERVICE_SCOPE_ALL,
};

class AppContext {
 public:
  DeviceController &controller;
  PowerAllocator &pAllocator;

  AppContext(DeviceController &controller, PowerAllocator &pAllocator);
};

using SrvFunc = std::function<esp_err_t(
    AppContext &, const std::vector<uint8_t> &, std::vector<uint8_t> &)>;

class App {
 private:
  AppContext m_ctx;
  std::unordered_map<ServiceCommand, SrvFunc> m_ble_services;
  std::unordered_map<ServiceCommand, SrvFunc> m_mqtt_services;

  // App(ESPController& controller, PowerAllocator& allocator, UARTClient&
  // client);
 public:
  // App(const App&) = delete;
  // App& operator=(const App&) = delete;
  App(DeviceController &controller, PowerAllocator &pAllocator);

  static App *m_instance;
  static void Init(DeviceController &controller, PowerAllocator &pAllocator);

  static App *GetInstance();

  void String(const std::string &str, std::vector<uint8_t> &response);

  bool HasSrv(uint8_t service, ServiceScope scope);

  void Srv(ServiceCommand command, SrvFunc func, ServiceScope scope);

  ResponseStatus PreCheckSrv(ServiceCommand command);

  size_t ExecSrvFromBle(uint8_t service, uint8_t *payload, size_t payloadSize,
                        uint8_t *response, size_t responseSize);

  void ExecSrvFromMQTT(uint8_t service, std::vector<uint8_t> &request,
                       std::vector<uint8_t> &response);

  void ProcessMQTTMessage(const char *data, int length);
};

#ifdef __cplusplus
}
#endif

#endif
