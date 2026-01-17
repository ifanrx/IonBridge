#include "power_handler.h"

#include <endian.h>
#include <stdlib.h>

#include <bitset>
#include <cstdint>
#include <cstring>
#include <string>
#include <unordered_set>
#include <vector>

#include "admin.pb.h"
#include "command.pb.h"
#include "config_sync_manager.h"
#include "controller.h"
#include "data_types.h"
#include "data_types.pb.h"
#include "display_manager.h"
#include "display_types.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "ionbridge.h"
#include "machine_info.h"
#include "nvs_key.h"
#include "nvs_namespace.h"
#include "pb_encode.h"
#include "pb_utils.h"
#include "port.h"
#include "port_data.h"
#include "port_manager.h"
#include "power_allocator.h"
#include "sdkconfig.h"
#include "strategy.h"
#include "utils.h"

#ifdef CONFIG_INTERFACE_TYPE_UART
#include "rpc.h"
#endif

#ifdef CONFIG_MCU_MODEL_SW3566
#include <array>
#endif

static const char *TAG = "PowerHandler";

using namespace nvs_namespace::user_data;
using namespace PowerHandler;

namespace {
struct CompatibilitySettings {
  bool isTfcpEnabled : 1;   //!< whether to enable tfcp
  bool isFcpEnabled : 1;    //!< whether to enable FCP
  bool isUfcsEnabled : 1;   //!< whether to enable ufcs
  bool isHvScpEnabled : 1;  //!< whether to enable high voltage scp
  bool isLvScpEnabled : 1;  //!< whether to enable low voltage scp
  uint8_t unused : 3;
};

struct PowerStatsData {
  uint8_t protocol;
  uint8_t temperature;
  uint16_t current;
  uint16_t voltage;
  uint16_t last_full_capacity;
  uint16_t present_capacity;
  int port_id;
};

static std::vector<PowerStatsData> CollectPowerStats() {
  std::vector<PowerStatsData> stats;
  PortManager &pm = PortManager::GetInstance();

  for (const Port &port : pm) {
    ClientPDStatus pdstatus = {};
    PortPowerData data = port.GetData();
    port.GetPDStatus(&pdstatus);

    PowerStatsData stat = {
        .protocol = data.GetFCProtocol(),
        .temperature = data.GetTemperature(),
        .current = data.GetCurrent(),
        .voltage = data.GetVoltage(),
        .last_full_capacity = pdstatus.battery_last_full_charge_capacity,
        .present_capacity = pdstatus.battery_present_capacity,
        .port_id = port.Id()};

    stats.push_back(stat);

    ESP_LOGD(TAG,
             "PD stats[%d], current: %d, voltage: %d, temperature: %d, "
             "fc_protocol: %d, Last Full Capacity: %d, Present Capacity: %d",
             stat.port_id, stat.current, stat.voltage, stat.temperature,
             stat.protocol, stat.last_full_capacity * 100,
             stat.present_capacity * 100);
  }

  return stats;
}

static int get_port_config_size(uint8_t version) {
  switch (version) {
    case 0:
      return 3;
    default:
      return sizeof(PortConfig);
  }
}

static bool get_power_historical_stats_encode_callback(pb_ostream_t *stream,
                                                       const pb_field_t *field,
                                                       void *const *arg) {
  const pb_CommandRequest *request =
      static_cast<const pb_CommandRequest *>(*arg);
  PortManager &portManager = PortManager::GetInstance();
  uint8_t port_id = request->payload.get_power_historical_stats.port;
  Port *port = portManager.GetPort(port_id);
  if (port == nullptr) {
    ESP_LOGE(TAG, "Invalid port number: %d", port_id);
    return false;
  }
  size_t offset = request->payload.get_power_historical_stats.offset;
  size_t length = port->GetHistoricalStatsSize();
  ESP_LOGD(TAG, "offset: %d, buffer length: %d", offset, length);

  std::vector<PortStatsData> data =
      port->GetHistoricalStats().GetPointsInRange(offset, length);

  for (const PortStatsData &point : data) {
    if (!pb_encode_tag_for_field(stream, field)) {
      return false;
    }
    pb_PowerHistoricalStats pb_power_historical_stats =
        pb_PowerHistoricalStats_init_zero;
    pb_power_historical_stats.current = point.GetAmperage();
    pb_power_historical_stats.voltage = point.GetVoltage();

    if (!pb_encode_submessage(stream, pb_PowerHistoricalStats_fields,
                              &pb_power_historical_stats)) {
      return false;
    }
  }
  return true;
}

static esp_err_t OperatePorts(const uint32_t *ports, size_t ports_count,
                              bool open) {
  PortManager &pm = PortManager::GetInstance();
  std::unordered_set<uint32_t> target_ports(ports, ports + ports_count);
  esp_err_t result = ESP_OK;
  pm.ForEach([&](Port &port) {
    if (open ? port.IsOpen() : port.IsClose()) {
      return;
    }
    if (target_ports.count(port.Id()) && !(open ? port.Open() : port.Close())) {
      ESP_LOGE(TAG, "Failed to %s port %d", open ? "open" : "close", port.Id());
      result = ESP_FAIL;
    }
  });

  if (result == ESP_OK) {
    ConfigSyncManager::GetInstance().NotifyConfigChange();
  }
  return result;
}

static esp_err_t validate_power_allocation_table(
    const PowerAllocationTable &table) {
  PortManager &pm = PortManager::GetInstance();

  int entry_count = sizeof(table.entries) / sizeof(table.entries[0]);
  for (int entry_index = 0; entry_index < entry_count; entry_index++) {
    PowerAllocationEntry entry = table.entries[entry_index];
    int sum = 0;
    for (const Port &port : pm) {
      bool port_used = ((entry_index >> port.Id()) & 0x1) == 1;
      uint16_t allocation = entry.power_allocation[port.Id()];
      if (!port_used && allocation != 0) {
        ESP_LOGE(TAG, "Power allocation should be 0 at entry %d, port %d",
                 entry_index, port.Id());
        return ESP_ERR_INVALID_ARG;
      }
      if (port_used && (allocation < PORT_MIN_CAP(port.Id()) ||
                        allocation > PORT_MAX_CAP(port.Id()))) {
        ESP_LOGE(TAG,
                 "Power allocation %d over range(%d-%d) at entry %d, port %d",
                 allocation, PORT_MIN_CAP(port.Id()), PORT_MAX_CAP(port.Id()),
                 entry_index, port.Id());
        return ESP_ERR_INVALID_ARG;
      }
      sum += allocation;
    }
    if (sum > CONFIG_MAX_POWER_BUDGET) {
      ESP_LOGE(TAG, "Sum(%d) exceed max power budget(%d) at entry %d", sum,
               CONFIG_MAX_POWER_BUDGET, entry_index);
      return ESP_ERR_INVALID_ARG;
    }
  }
  return ESP_OK;
}

static esp_err_t ApplyPortConfigs(uint8_t port_mask,
                                  const PortConfig configs[8],
                                  bool prohibit_syncing = false) {
  PortManager &pm = PortManager::GetInstance();

  for (Port &port : pm.GetAlivePorts()) {
    // Skip ports that are not selected in the mask
    if (((port_mask >> port.Id()) & 0x1) == 0) {
      continue;
    }

    // Apply the configuration directly to the port
    ESP_RETURN_ON_ERROR(port.SetConfig(configs[port.Id()]), TAG,
                        "Port::SetConfig %d", port.Id());
  }

  if (!prohibit_syncing) {
    ConfigSyncManager::GetInstance().NotifyConfigChange();
  }

  return ESP_OK;
}

static esp_err_t GetPortConfigs(uint8_t version, PortConfig configs[8]) {
  size_t i = 0;
  for (const Port &port : PortManager::GetInstance()) {
    configs[i] = port.GetConfig();
    configs[i].version = version;
    i++;
  }
  return ESP_OK;
}
}  // anonymous namespace

esp_err_t FromBLE::GetChargeDuration(const std::vector<uint8_t> &request,
                                     std::vector<uint8_t> &response) {
  uint16_t minute =
      PortManager::GetInstance().GetChargingDurationSeconds() / 60;
  EMPLACE_BACK_INT16(response, minute);
  return ESP_OK;
}

esp_err_t FromBLE::GetChargingStatus(const std::vector<uint8_t> &request,
                                     std::vector<uint8_t> &response) {
  uint8_t chargingStatus = PortManager::GetInstance().GetAttachedPortsBitMask();
  response.emplace_back(chargingStatus);
  ESP_LOGI(TAG, "Charging status: %s",
           std::bitset<5>(chargingStatus).to_string().c_str());
  return ESP_OK;
}

esp_err_t FromBLE::GetAllPowerStats(const std::vector<uint8_t> &request,
                                    std::vector<uint8_t> &response) {
  for (const PowerStatsData &stat : CollectPowerStats()) {
    response.emplace_back(stat.protocol);
    response.emplace_back(PortStatsData::ScaleAmperage(stat.current));
    response.emplace_back(PortStatsData::ScaleVoltage(stat.voltage));
    response.emplace_back(stat.temperature);
    EMPLACE_BACK_INT16(response, stat.last_full_capacity);
    EMPLACE_BACK_INT16(response, stat.present_capacity);
  }
  return ESP_OK;
}

esp_err_t FromBLE::ForwardPort(const std::vector<uint8_t> &request,
                               std::vector<uint8_t> &response) {
  if (request.size() < 3) {
    ESP_LOGE(TAG, "Invalid request size: %d", request.size());
    return ESP_ERR_INVALID_SIZE;
  }
#ifdef CONFIG_MCU_MODEL_SW3566
  uint8_t port = request[0];

  uint16_t command;
  memcpy(&command, &request[1], sizeof(uint16_t));
  ESP_LOGD(TAG, "Forward to port: %d, command: 0x%04X", port, command);

  response.resize(0xFF);
  uint8_t res_size = response.size();

  ESP_RETURN_ON_ERROR(
      rpc::mcu::send_command(port, command, (const uint8_t *)request.data() + 3,
                             request.size() - 3, response.data(), &res_size),
      TAG, "sw3566::send_command port=%d, cmd=0x%x", port, command);

  response.resize(res_size);
  response.insert(response.begin(), command & 0xFF);
  response.insert(response.begin() + 1, command >> 8);
#endif
  return ESP_OK;
}

esp_err_t FromBLE::GetChargingStrategy(const std::vector<uint8_t> &request,
                                       std::vector<uint8_t> &response) {
  uint8_t strategy =
      static_cast<uint8_t>(PowerAllocator::GetInstance().GetStrategyType());
  response.emplace_back(strategy);
  return ESP_OK;
}

esp_err_t FromBLE::GetStaticAllocator(const std::vector<uint8_t> &request,
                                      std::vector<uint8_t> &response) {
  auto static_strategy = PowerAllocator::GetInstance()
                             .GetStrategyAs<PowerStaticChargingStrategy>();
  if (static_strategy == nullptr) {
    ESP_LOGE(TAG, "Not static strategy type");
    return ESP_ERR_INVALID_STATE;
  }

  uint16_t identifier = static_strategy->GetIdentifier();
  uint8_t version = static_strategy->GetVersion();
  EMPLACE_BACK_INT16(response, identifier);
  response.emplace_back(version);
  return ESP_OK;
}

esp_err_t FromBLE::SetPortConfig(const std::vector<uint8_t> &request,
                                 std::vector<uint8_t> &response) {
  if (request.size() < 1 + 8) {
    ESP_LOGE(TAG, "Invalid SetPortConfigs request size: %d", request.size());
    return ESP_ERR_INVALID_SIZE;
  }

  uint8_t port_mask = request[0];
  uint8_t version = request[1];
  int config_size = get_port_config_size(version);
  size_t expected_base_size = 1 + 8 * config_size;

  if (request.size() < expected_base_size ||
      request.size() > expected_base_size + 1) {
    ESP_LOGE(TAG, "Invalid SetPortConfigs request size: %d", request.size());
    return ESP_ERR_INVALID_SIZE;
  }

  PortConfig configs[8];
  for (int i = 0; i < 8; i++) {
    std::memcpy(&configs[i], &request[1 + i * config_size], config_size);
  }

  // Parse optional prohibit_syncing byte
  bool prohibit_syncing = (request.size() > expected_base_size) &&
                          (request[expected_base_size] != 0);

  return ApplyPortConfigs(port_mask, configs, prohibit_syncing);
}

esp_err_t FromBLE::GetPortConfig(const std::vector<uint8_t> &request,
                                 std::vector<uint8_t> &response) {
  uint8_t version = 0;
  if (request.size() == 1) {
    version = request[0];
  }
  int config_size = get_port_config_size(version);

  PortConfig configs[8] = {};
  ESP_RETURN_ON_ERROR(GetPortConfigs(version, configs), TAG,
                      "GetPortConfigs failed");

  for (size_t i = 0; i < 8; i++) {
    uint8_t *data_ptr = reinterpret_cast<uint8_t *>(&configs[i]);
    response.insert(response.end(), data_ptr, data_ptr + config_size);
  }
  return ESP_OK;
}

esp_err_t FromBLE::GetPortsVin(const std::vector<uint8_t> &request,
                               std::vector<uint8_t> &response) {
  PortManager &pm = PortManager::GetInstance();
  response.clear();
  response.emplace_back(pm.PortCount());
  for (const Port &port : pm) {
    const PortPowerData &data = port.GetData();
    uint16_t vin = data.GetVinValue();
    response.emplace_back(vin & 0xFF);
    response.emplace_back((vin >> 8) & 0xFF);
  }
  return ESP_OK;
}

esp_err_t FromMQTT::GetPowerHistoricalStats(const pb_CommandRequest &request,
                                            pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_get_power_historical_stats_tag;
  PortManager &portManager = PortManager::GetInstance();
  uint8_t port_id = request.payload.get_power_historical_stats.port;
  Port *port = portManager.GetPort(port_id);
  ESP_RETURN_ON_FALSE(port != nullptr, ESP_ERR_NOT_FOUND, TAG,
                      "Invalid port number: %d", port_id);

  size_t offset = request.payload.get_power_historical_stats.offset;

  response.payload.get_power_historical_stats.offset = offset;
  response.payload.get_power_historical_stats.data.funcs.encode =
      get_power_historical_stats_encode_callback;
  response.payload.get_power_historical_stats.data.arg =
      const_cast<void *>(static_cast<const void *>(&request));
  return ESP_OK;
}

esp_err_t FromMQTT::GetChargingStatus(const pb_CommandRequest &request,
                                      pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_get_charging_status_tag;
  PortManager &pm = PortManager::GetInstance();
  response.payload.get_charging_status.status =
      pm.GetAttachedPortsBitMask() & 0x1F;
  ESP_LOGI(TAG, "Charging status: %s",
           std::bitset<5>(response.payload.get_charging_status.status)
               .to_string()
               .c_str());
  return ESP_OK;
}

esp_err_t FromMQTT::SetChargingStrategy(const pb_CommandRequest &request,
                                        pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_set_charging_strategy_tag;
  uint8_t strategy = request.payload.set_charging_strategy.charging_strategy;
  PowerAllocator &allocator = PowerAllocator::GetInstance();
  switch (strategy) {
    case StrategyType::SLOW_CHARGING:
      allocator.SetStrategy<PowerSlowChargingStrategy>();
      break;
    case StrategyType::USBA_CHARGING:
      allocator.SetStrategy<PowerUSBAChargingStrategy>();
      break;
    case StrategyType::HIGH_PERFORMANCE_CHARGING:
      allocator.SetStrategy<PowerHighPerformanceStrategy>();
      break;
    case StrategyType::ULTRA_FAST_SINGLE_PORT_CHARGING:
      allocator.SetStrategy<PowerUltraFastSinglePortStrategy>();
      break;
    default:
      ESP_LOGE(TAG, "Invalid charging strategy: %d", strategy);
      return ESP_ERR_INVALID_ARG;
  }
  ConfigSyncManager::GetInstance().NotifyConfigChange();
  return ESP_OK;
}

esp_err_t FromMQTT::SetPortPriority(const pb_CommandRequest &request,
                                    pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_set_port_priority_tag;
  esp_err_t __attribute__((unused)) ret;
#ifdef CONFIG_MCU_MODEL_SW3566
  std::array<bool, NUM_PORTS> portsAlreadySet = {};
  std::array<uint8_t, NUM_PORTS> priority;
  for (uint8_t port = 0; port < NUM_PORTS; port++) {
    ESP_LOGI(TAG, "port: %d, request: %d", (int)port,
             (int)request.payload.set_port_priority.priority[port]);
    if (portsAlreadySet[port]) {
      return ESP_FAIL;
    }
    portsAlreadySet[port] = true;
    priority[port] = request.payload.set_port_priority.priority[port];
  }

  uint8_t oldPriorities[NUM_PORTS] = {};
  POWER.GetOrDefault(nvs_keys::POWER_PORT_PRIORITY, oldPriorities);

  for (uint8_t i = 0; i < NUM_PORTS; i++) {
    ESP_RETURN_ON_ERROR(rpc::fpga::set_mcu_priority(i, priority[i]), TAG,
                        "rpc::fpga::set_mcu_priority: %d=%d", i, priority[i]);
  }

  ESP_GOTO_ON_ERROR(POWER.Set(nvs_keys::POWER_PORT_PRIORITY, priority),
                    ROLLBACK_AND_FAIL, TAG, "PowerNVSData::SavePortPriority");
  return ESP_OK;

ROLLBACK_AND_FAIL:
  for (uint8_t i = 0; i < NUM_PORTS; i++) {
    ESP_ERROR_COMPLAIN(rpc::fpga::set_mcu_priority(i, oldPriorities[i]),
                       "rpc::fpga::set_mcu_priority: %d=%d", i,
                       oldPriorities[i]);
  }
  return ESP_FAIL;
#else
  return ESP_OK;
#endif
}

esp_err_t FromMQTT::GetPortPriority(const pb_CommandRequest &request,
                                    pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_get_port_priority_tag;
  uint8_t priorities[NUM_PORTS] = {};
  POWER.GetOrDefault(nvs_keys::POWER_PORT_PRIORITY, priorities);

  for (uint8_t i = 0; i < NUM_PORTS; i++) {
    response.payload.get_port_priority.priority[i] = priorities[i];
  }
  return ESP_OK;
}

esp_err_t FromMQTT::GetChargingStrategy(const pb_CommandRequest &request,
                                        pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_get_charging_strategy_tag;
  uint8_t strategy =
      static_cast<uint8_t>(PowerAllocator::GetInstance().GetStrategyType());
  response.payload.get_charging_strategy.charging_strategy =
      static_cast<pb_ChargingStrategy>(strategy);
  return ESP_OK;
}

esp_err_t FromMQTT::GetPortPDStatus(const pb_CommandRequest &request,
                                    pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_get_port_pd_status_tag;
  uint8_t port_index = request.payload.get_port_pd_status.port;
  ClientPDStatus pdstatus = {};
  PortManager &pm = PortManager::GetInstance();
  Port *port = pm.GetPort(port_index);
  ESP_RETURN_ON_FALSE(port != nullptr, ESP_ERR_NOT_FOUND, TAG,
                      "Invalid port index: %d", port_index);
  port->GetPDStatus(&pdstatus);
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, &pdstatus, sizeof(ClientPDStatus),
                           ESP_LOG_DEBUG);
  ESP_LOGD(TAG,
           "Battery VID: 0x%04X, PID: 0x%04X, Design Cap: %dmWh, Last Full "
           "Cap: %dmWh, Present Cap: %dmWh, Invalid: %d, Present: %d, Status: "
           "%d, Temperature: %d, Manufacturer VID: 0x%04X, PID: 0x%04X",
           pdstatus.battery_vid, pdstatus.battery_pid,
           pdstatus.battery_design_capacity * 100,
           pdstatus.battery_last_full_charge_capacity * 100,
           pdstatus.battery_present_capacity * 100, pdstatus.battery_invalid,
           pdstatus.battery_present, pdstatus.battery_status,
           pdstatus.status_temperature, pdstatus.manufacturer_vid,
           pdstatus.manufacturer_pid);

  pb_ClientPDStatus &pb_pdstatus =
      response.payload.get_port_pd_status.pd_status;
  ClientPDStatusToPB(pb_pdstatus, pdstatus);
  return ESP_OK;
}

esp_err_t FromMQTT::GetAllPowerStats(const pb_CommandRequest &request,
                                     pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_get_all_power_statistics_tag;

  for (const PowerStatsData &stat : CollectPowerStats()) {
    pb_PowerStatistics &power_stats =
        response.payload.get_all_power_statistics.stats[stat.port_id];
    power_stats.charging_protocol_version = stat.protocol;
    power_stats.current = stat.current;
    power_stats.voltage = stat.voltage;
    power_stats.temperature = stat.temperature;
    power_stats.battery_last_full_charge_capacity = stat.last_full_capacity;
    power_stats.battery_present_capacity = stat.present_capacity;
  }

  return ESP_OK;
}

esp_err_t FromMQTT::TurnOnPort(const pb_CommandRequest &request,
                               pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_turn_on_port_tag;
  DeviceController &controller = DeviceController::GetInstance();
  ESP_RETURN_ON_FALSE(controller.is_power_on(), ESP_ERR_NOT_SUPPORTED, TAG,
                      "Device is power off. Turning on port is forbidden");

  return OperatePorts(request.payload.turn_on_port.ports,
                      request.payload.turn_on_port.ports_count, true);
}

esp_err_t FromMQTT::TurnOffPort(const pb_CommandRequest &request,
                                pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_turn_off_port_tag;
  DeviceController &controller = DeviceController::GetInstance();
  ESP_RETURN_ON_FALSE(controller.is_power_on(), ESP_ERR_NOT_SUPPORTED, TAG,
                      "Device is power off. Turning off port is forbidden");

  return OperatePorts(request.payload.turn_off_port.ports,
                      request.payload.turn_off_port.ports_count, false);
}

esp_err_t FromMQTT::ForwardPort(const pb_AdminCommandRequest &request,
                                pb_AdminCommandResponse &response) {
  response.which_payload = pb_AdminCommandResponse_forward_port_tag;
  response.payload.forward_port.data.size = 0;
#ifdef CONFIG_MCU_MODEL_SW3566
  uint8_t port = request.payload.forward_port.port;

  uint16_t command = request.payload.forward_port.command;
  ESP_LOGD(TAG, "Forward to port: %d, command: 0x%04X", port, command);

  response.payload.forward_port.command = command;
  uint8_t res_size = 0xFF;

  ESP_RETURN_ON_ERROR(
      rpc::mcu::send_command(
          port, command,
          (const uint8_t *)request.payload.forward_port.data.bytes,
          request.payload.forward_port.data.size,
          response.payload.forward_port.data.bytes, &res_size),
      TAG, "sw3566::send_command port=%d, cmd=0x%x", port, command);

  if (res_size <= sizeof(response.payload.forward_port.data.bytes)) {
    response.payload.forward_port.data.size = res_size;
  } else {
    ESP_LOGE(TAG, "Response size %d exceeds buffer size %d", res_size,
             sizeof(response.payload.forward_port.data.bytes));
    return ESP_ERR_INVALID_SIZE;
  }
#endif
  return ESP_OK;
}

esp_err_t FromMQTT::SetStaticAllocator(const pb_CommandRequest &request,
                                       pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_set_static_allocator_tag;
  uint16_t identifier = request.payload.set_static_allocator.identifier;
  PowerAllocator &allocator = PowerAllocator::GetInstance();
  PowerAllocationTable table;
  std::memcpy(&table, &request.payload.set_static_allocator.table,
              sizeof(PowerAllocationTable));
  ESP_RETURN_ON_ERROR(validate_power_allocation_table(table), TAG,
                      "Invalid power allocation table");
  allocator.SetStrategy<PowerStaticChargingStrategy>(table, identifier);
  response.payload.set_static_allocator.identifier = identifier;
  ESP_LOGI(TAG, "Power allocation table set");
  ConfigSyncManager::GetInstance().NotifyConfigChange();
  return ESP_OK;
}

esp_err_t FromMQTT::GetStaticAllocator(const pb_CommandRequest &request,
                                       pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_get_static_allocator_tag;
  auto static_strategy = PowerAllocator::GetInstance()
                             .GetStrategyAs<PowerStaticChargingStrategy>();
  if (static_strategy == nullptr) {
    ESP_LOGE(TAG, "Not static strategy type");
    return ESP_ERR_INVALID_STATE;
  }

  uint16_t identifier = static_strategy->GetIdentifier();
  uint8_t version = static_strategy->GetVersion();
  response.payload.get_static_allocator.version = version;
  response.payload.get_static_allocator.identifier = identifier;
  return ESP_OK;
}

esp_err_t FromMQTT::SetPortConfig(const pb_CommandRequest &request,
                                  pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_set_port_config_tag;
  uint8_t port_mask = request.payload.set_port_config.ports;
  PortConfig configs[8] = {};
  for (int i = 0; i < request.payload.set_port_config.configs_count; i++) {
    configs[i].version = request.payload.set_port_config.configs[i].version;
    PowerFeaturesFromPB(configs[i].features,
                        request.payload.set_port_config.configs[i].features);
  }

  return ApplyPortConfigs(port_mask, configs);
}

esp_err_t FromMQTT::GetPortConfig(const pb_CommandRequest &request,
                                  pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_get_port_config_tag;
  uint8_t version = request.payload.get_port_config.version;

  PortConfig configs[8] = {};
  ESP_RETURN_ON_ERROR(GetPortConfigs(version, configs), TAG,
                      "GetPortConfigs failed");

  response.payload.get_port_config.configs_count = 0;
  for (size_t i = 0; i < 8; i++) {
    response.payload.get_port_config.configs[i].version = configs[i].version;
    pb_PowerFeatures &pb_features =
        response.payload.get_port_config.configs[i].features;
    PowerFeaturesToPB(pb_features, configs[i].features);
    response.payload.get_port_config.configs_count++;
  }
  return ESP_OK;
}

esp_err_t FromMQTT::SetPortCompatibilitySettings(
    const pb_CommandRequest &request, pb_CommandResponse &response) {
  response.which_payload =
      pb_CommandResponse_set_port_compatibility_settings_tag;
  CompatibilitySettings settings;
  settings.isTfcpEnabled =
      request.payload.set_port_compatibility_settings.settings.enable_tfcp;
  settings.isFcpEnabled =
      request.payload.set_port_compatibility_settings.settings.enable_fcp;
  settings.isUfcsEnabled =
      request.payload.set_port_compatibility_settings.settings.enable_ufcs;
  settings.isHvScpEnabled =
      request.payload.set_port_compatibility_settings.settings.enable_hv_scp;
  settings.isLvScpEnabled =
      request.payload.set_port_compatibility_settings.settings.enable_lv_scp;
  settings.unused = 0;

  PortManager &pm = PortManager::GetInstance();

  for (Port &port : pm) {
    PortConfig config = port.GetConfig();

    config.features.EnableTfcp = settings.isTfcpEnabled;
    config.features.EnableFcp = settings.isFcpEnabled;
    config.features.EnableUfcs = settings.isUfcsEnabled;
    config.features.EnableHvScp = settings.isHvScpEnabled;
    config.features.EnableLvScp = settings.isLvScpEnabled;

    if (port.Dead()) {
      continue;
    }

    ESP_RETURN_ON_ERROR(port.SetConfig(config), TAG, "Port::SetConfig %d",
                        port.Id());
  }

  // Notify config sync manager about the change
  ConfigSyncManager::GetInstance().NotifyConfigChange();

  return ESP_OK;
}

esp_err_t FromMQTT::GetPortCompatibilitySettings(
    const pb_CommandRequest &request, pb_CommandResponse &response) {
  response.which_payload =
      pb_CommandResponse_get_port_compatibility_settings_tag;
  CompatibilitySettings settings = {
      .isTfcpEnabled = false,
      .isFcpEnabled = false,
      .isUfcsEnabled = false,
      .isHvScpEnabled = false,
      .isLvScpEnabled = false,
      .unused = 0,
  };
  PortManager &pm = PortManager::GetInstance();
  for (const Port &port : pm) {
    const PortConfig &config = port.GetConfig();
    settings.isTfcpEnabled =
        settings.isTfcpEnabled || config.features.EnableTfcp;
    settings.isFcpEnabled = settings.isFcpEnabled || config.features.EnableFcp;
    settings.isUfcsEnabled =
        settings.isUfcsEnabled || config.features.EnableUfcs;
    settings.isHvScpEnabled =
        settings.isHvScpEnabled || config.features.EnableHvScp;
    settings.isLvScpEnabled =
        settings.isLvScpEnabled || config.features.EnableLvScp;
  }
  response.payload.get_port_compatibility_settings.settings.enable_tfcp =
      settings.isTfcpEnabled;
  response.payload.get_port_compatibility_settings.settings.enable_fcp =
      settings.isFcpEnabled;
  response.payload.get_port_compatibility_settings.settings.enable_ufcs =
      settings.isUfcsEnabled;
  response.payload.get_port_compatibility_settings.settings.enable_hv_scp =
      settings.isHvScpEnabled;
  response.payload.get_port_compatibility_settings.settings.enable_lv_scp =
      settings.isLvScpEnabled;
  return ESP_OK;
}

esp_err_t FromMQTT::SetTemperatureMode(const pb_CommandRequest &request,
                                       pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_set_temperature_mode_tag;
  uint8_t mode = request.payload.set_temperature_mode.mode;
  ESP_RETURN_ON_FALSE(mode == TemperatureMode::POWER_PRIORITY ||
                          mode == TemperatureMode::TEMPERATURE_PRIORITY,
                      ESP_ERR_INVALID_ARG, TAG, "Invalid request value: %d",
                      mode);
  esp_err_t result = PowerAllocator::GetInstance().SetTemperatureMode(
      static_cast<TemperatureMode>(mode));
  if (result == ESP_OK) {
    ConfigSyncManager::GetInstance().NotifyConfigChange();
  }
  return result;
}

esp_err_t FromMQTT::SetTemporaryAllocator(const pb_CommandRequest &request,
                                          pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_set_temporary_allocator_tag;
  PortManager &pm = PortManager::GetInstance();
  uint8_t allocation_table[NUM_PORTS] = {};
  uint8_t sum = 0;
  for (const Port &port : pm) {
    uint8_t allocation =
        request.payload.set_temporary_allocator.power_allocation[port.Id()];
    if (allocation > PORT_MAX_CAP(port.Id()) ||
        (allocation < PORT_MIN_CAP(port.Id()) && allocation != 0)) {
      ESP_LOGE(TAG, "Port %d power allocation(%d) out of range(%d-%d)",
               port.Id(), allocation, PORT_MIN_CAP(port.Id()),
               PORT_MAX_CAP(port.Id()));
      return ESP_ERR_INVALID_ARG;
    }
    allocation_table[port.Id()] = allocation;
    sum += allocation;
  }

  if (sum > CONFIG_MAX_POWER_BUDGET) {
    ESP_LOGE(TAG, "Power allocation sum(%d) exceeds max power budget(%d)", sum,
             CONFIG_MAX_POWER_BUDGET);
    return ESP_ERR_INVALID_ARG;
  }

  PowerAllocator &allocator = PowerAllocator::GetInstance();

  // Create and configure temporary charging strategy
  auto strategy =
      std::make_unique<PowerTemporaryChargingStrategy>(allocation_table);
  strategy->OnExit([]() {
    DisplayManager &display = DisplayManager::GetInstance();
    display.SetIntensity(50);
    display.SetMode(DisplayMode::POWER_METER);
  });
  allocator.SetStrategy(std::move(strategy));

  return ESP_OK;
}

esp_err_t FromMQTT::SubscribePDPcapStreaming(const pb_CommandRequest &request,
                                             pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_subscribe_pdpcap_streaming_tag;
#ifdef CONFIG_MCU_MODEL_SW3566
  // port id + subscribe
  uint8_t port_id = request.payload.subscribe_pdpcap_streaming.port;
  bool subscribe = request.payload.subscribe_pdpcap_streaming.subscribe;
  PortManager &pm = PortManager::GetInstance();
  Port *port = pm.GetPort(port_id);
  ESP_RETURN_ON_FALSE(port != nullptr, ESP_ERR_NOT_FOUND, TAG,
                      "Invalid port id: %d", port_id);
  ESP_RETURN_ON_ERROR(port->SubscribePDPcapStreaming(subscribe), TAG,
                      "Failed to subscribe PDPcap streaming for port %d",
                      port_id);
  ESP_LOGI(TAG, "Subscribe PDPcap streaming for port %d: %s", port_id,
           subscribe ? "true" : "false");
  return ESP_OK;
#else
  ESP_LOGW(TAG, "PD Pcap streaming is not supported");
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t FromMQTT::SetPortType(const pb_CommandRequest &request,
                                pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_set_port_type_tag;
  uint8_t port_id = request.payload.set_port_type.port;
  uint8_t port_type = request.payload.set_port_type.port_type;
  ESP_RETURN_ON_FALSE(port_type < PortType::PORT_TYPE_MAX, ESP_ERR_INVALID_ARG,
                      TAG, "Invalid port type: %d", port_type);
  PortManager &pm = PortManager::GetInstance();
  Port *port = pm.GetPort(port_id);
  ESP_RETURN_ON_FALSE(port != nullptr, ESP_ERR_INVALID_ARG, TAG,
                      "Invalid port id: %d", port_id);
  ESP_RETURN_ON_ERROR(port->SetPortType(static_cast<PortType>(port_type)), TAG,
                      "Port::SetPortType %d", port_id);
  ESP_RETURN_ON_ERROR(port->Reconnect(), TAG, "Port::Reconnect %d", port_id);
  return ESP_OK;
}

esp_err_t FromMQTT::SetCustomPdoVoltage(const pb_CommandRequest &request,
                                        pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_set_custom_pdo_voltage_tag;
  uint8_t port_id = request.payload.set_custom_pdo_voltage.port;
  uint16_t voltage_mv = request.payload.set_custom_pdo_voltage.voltage_mv;

  PortManager &pm = PortManager::GetInstance();
  Port *p = pm.GetPort(port_id);
  if (p == nullptr) {
    ESP_LOGE(TAG, "Invalid port %d", port_id);
    return ESP_ERR_NOT_FOUND;
  }

  // Use the Port's SetCustomPdoVoltage method which includes validation
  esp_err_t result = p->SetCustomPdoVoltage(voltage_mv);
  if (result == ESP_OK) {
    ConfigSyncManager::GetInstance().NotifyConfigChange();
  }
  return result;
}

esp_err_t FromMQTT::SetCableCompensation(const pb_CommandRequest &request,
                                         pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_set_cable_compensation_tag;
  response.payload.set_cable_compensation =
      pb_SetCableCompensationResp_init_zero;

  uint8_t port_mask = request.payload.set_cable_compensation.port_mask;
  bool enable = !request.payload.set_cable_compensation.disable;

  // Convert protobuf values to hardware enums
  // Protobuf: 0=65mΩ, 1=100mΩ -> Hardware: 0=100mΩ, 1=65mΩ (so invert)
  CableCompensation resistance =
      request.payload.set_cable_compensation.resistance == 0
          ? CableCompensation::VALUE_65MOHM
          : CableCompensation::VALUE_100MOHM;

  FixedVoltageOffset voltage_offset = static_cast<FixedVoltageOffset>(
      request.payload.set_cable_compensation.voltage_offset);

  PortManager &pm = PortManager::GetInstance();
  esp_err_t result = ESP_OK;

  // Apply settings to all ports specified in the bitmask
  while (port_mask != 0) {
    uint8_t port_id = static_cast<uint8_t>(__builtin_ctz(port_mask));
    port_mask &= ~(1ULL << port_id);
    Port *p = pm.GetPort(port_id);
    if (!p) continue;
    esp_err_t err =
        p->SetCableCompensation(enable, resistance, voltage_offset, false);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "Failed to set cable compensation for port %d: 0x%x",
               port_id, err);
      result = err;  // Store the last error, but continue with other ports
    }
  }

  return result;
}

esp_err_t FromMQTT::GetCableCompensation(const pb_CommandRequest &request,
                                         pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_get_cable_compensation_tag;
  response.payload.get_cable_compensation =
      pb_GetCableCompensationResp_init_zero;

  uint8_t port_id = request.payload.get_cable_compensation.port;
  PortManager &pm = PortManager::GetInstance();
  Port *p = pm.GetPort(port_id);

  ESP_RETURN_ON_FALSE(p != nullptr, ESP_ERR_INVALID_ARG, TAG, "Invalid port %d",
                      port_id);
  uint8_t resistance, voltage_offset;
  p->GetCableCompensation(response.payload.get_cable_compensation.enable,
                          resistance, voltage_offset);

  // Convert hardware resistance to proto resistance
  // Hardware: 0=100mΩ, 1=65mΩ -> Proto: 0=65mΩ, 1=100mΩ
  response.payload.get_cable_compensation.resistance =
      static_cast<pb_CableCompensation>(resistance ? 0 : 1);
  response.payload.get_cable_compensation.voltage_offset =
      static_cast<pb_FixedVoltageOffset>(voltage_offset);

  return ESP_OK;
}

esp_err_t FromMQTT::ToggleMcuGPIO(const pb_AdminCommandRequest &request,
                                  pb_AdminCommandResponse &response) {
#ifdef CONFIG_INTERFACE_TYPE_UART
  response.which_payload = pb_AdminCommandResponse_toggle_mcu_gpio_tag;
  response.payload.toggle_mcu_gpio = pb_ToggleMcuGPIOResp_init_zero;
  uint8_t port_id = request.payload.toggle_mcu_gpio.port;

  PortManager &pm = PortManager::GetInstance();
  Port *port = pm.GetPort(port_id);
  ESP_RETURN_ON_FALSE(port != nullptr, ESP_ERR_INVALID_ARG, TAG,
                      "Invalid port number: %d", port_id);

  ESP_RETURN_ON_ERROR(rpc::fpga::toggle_mcu_gpio(port_id), TAG,
                      "Failed to toggle SW3566 GPIO2 for port %d", port_id);

  return ESP_OK;
#else
  ESP_LOGW(TAG, "ToggleMcuGPIO not supported on this model");
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t FromMQTT::ReadADCValue(const pb_AdminCommandRequest &request,
                                 pb_AdminCommandResponse &response) {
  response.which_payload = pb_AdminCommandResponse_read_adc_tag;
  response.payload.read_adc = pb_ReadADCResp_init_zero;
#ifdef CONFIG_INTERFACE_TYPE_UART
  uint8_t adc_value = 0;
  ESP_RETURN_ON_ERROR(rpc::fpga::read_adc_value(&adc_value), TAG,
                      "rpc::fpga::read_adc_value");
  response.payload.read_adc.adc_value = adc_value;
  return ESP_OK;
#else
  ESP_LOGW(TAG, "ReadADCValue is not supported on this model");
  return ESP_ERR_NOT_SUPPORTED;
#endif
}
