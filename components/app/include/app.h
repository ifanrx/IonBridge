#ifndef APP_H_
#define APP_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
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
  std::unique_ptr<AppContext> m_ctx;
  std::unordered_map<ServiceCommand, SrvFunc> m_ble_services;
  std::unordered_map<ServiceCommand, SrvFunc> m_mqtt_services;

  App() = default;
  ~App() = default;

  bool is_initialized = false;

 public:
  App(const App &) = delete;
  App &operator=(const App &) = delete;

  static App &GetInstance() {
    static App instance;
    return instance;
  }

  void Init(DeviceController &controller, PowerAllocator &pAllocator);
  bool IsInitialized() { return is_initialized; }

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
