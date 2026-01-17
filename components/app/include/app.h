#ifndef APP_H_
#define APP_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

#include "admin.pb.h"
#include "command.pb.h"
#include "esp_err.h"
#include "service.h"

using SrvFunc = std::function<esp_err_t(const std::vector<uint8_t> &,
                                        std::vector<uint8_t> &)>;
using EnduserSrv =
    std::function<esp_err_t(const pb_CommandRequest &, pb_CommandResponse &)>;
using AdminSrv = std::function<esp_err_t(const pb_AdminCommandRequest &,
                                         pb_AdminCommandResponse &)>;

class App {
 private:
  std::unordered_map<ServiceCommand, SrvFunc> m_ble_services;
  std::unordered_map<ServiceCommand, EnduserSrv> m_mqtt_services;
  std::unordered_map<AdminCommand, AdminSrv> m_mqtt_admin_services;

  App() = default;
  ~App() = default;

  template <typename CmdEnumT, typename PbRequestT, typename PbResponseT,
            typename HandlerMapT>
  void ExecMQTTSrvGeneric(CmdEnumT command, const PbRequestT &request,
                          PbResponseT &response,
                          const HandlerMapT &handler_map);

 public:
  App(const App &) = delete;
  App &operator=(const App &) = delete;

  static App &GetInstance() {
    static App instance;
    return instance;
  }

  bool HasBLESrv(uint8_t service);
  bool HasMQTTSrv(uint8_t service);

  void SrvBLE(ServiceCommand command, SrvFunc func) {
    m_ble_services[command] = std::move(func);
  }

  void SrvMQTT(ServiceCommand command, EnduserSrv func) {
    m_mqtt_services[command] = std::move(func);
  }
  void SrvMQTT(AdminCommand command, AdminSrv func) {
    m_mqtt_admin_services[command] = std::move(func);
  }

  pb_CommandStatus PreCheckSrv(ServiceCommand command);

  size_t ExecBLESrv(uint8_t service, uint8_t *payload, size_t payloadSize,
                    uint8_t *response, size_t responseSize);

  void ExecMQTTSrv(ServiceCommand command, const pb_CommandRequest &request,
                   pb_CommandResponse &response) {
    ExecMQTTSrvGeneric(command, request, response, m_mqtt_services);
  }
  void ExecMQTTSrv(AdminCommand command, const pb_AdminCommandRequest &request,
                   pb_AdminCommandResponse &response) {
    ExecMQTTSrvGeneric(command, request, response, m_mqtt_admin_services);
  }
};

#include "app.tpp"

#endif
