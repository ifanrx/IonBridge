#include "power_handler.h"

#include <endian.h>
#include <stdlib.h>

#include <bitset>
#include <cinttypes>
#include <cstdint>
#include <cstring>
#include <vector>

#include "controller.h"
#include "data_types.h"
#include "display_manager.h"
#include "display_types.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ionbridge.h"
#include "machine_info.h"
#include "nvs.h"
#include "nvs_default.h"
#include "nvs_namespace.h"
#include "port.h"
#include "port_data.h"
#include "port_manager.h"
#include "power_allocator.h"
#include "sdkconfig.h"
#include "strategy.h"
#include "utils.h"

#ifdef CONFIG_MCU_MODEL_SW3566
#include <array>

#include "rpc.h"
#endif

static const char *TAG = "PowerHandler";

static NVSKey port_config_keys[8] = {
    NVSKey::POWER_PORT0_CONFIG, NVSKey::POWER_PORT1_CONFIG,
    NVSKey::POWER_PORT2_CONFIG, NVSKey::POWER_PORT3_CONFIG,
    NVSKey::POWER_PORT4_CONFIG, NVSKey::POWER_PORT5_CONFIG,
    NVSKey::POWER_PORT6_CONFIG, NVSKey::POWER_PORT7_CONFIG,
};

esp_err_t PowerHandler::TogglePortPower(const std::vector<uint8_t> &request,
                                        std::vector<uint8_t> &response) {
  if (request.size() != 1) {
    return ESP_FAIL;
  }
  DeviceController &controller = DeviceController::GetInstance();
  ESP_RETURN_ON_FALSE(controller.is_power_on(), ESP_ERR_NOT_SUPPORTED, TAG,
                      "Device is power off. Cannot toggle port power");
  PortManager &pm = PortManager::GetInstance();
  Port *port = pm.GetPort(request[0]);
  ESP_RETURN_ON_FALSE(port != nullptr, ESP_ERR_NOT_FOUND, TAG,
                      "Invalid port id: %d", request[0]);
  return port->Toggle() ? ESP_OK : ESP_FAIL;
}

esp_err_t PowerHandler::GetPowerStats(const std::vector<uint8_t> &request,
                                      std::vector<uint8_t> &response) {
  if (request.size() <= 0) {
    return ESP_FAIL;
  }

  uint8_t port_id = request[0], fc_protocol = 0, temperature = 0;
  uint16_t voltage = 0, current = 0;

  PortManager &pm = PortManager::GetInstance();
  Port *port = pm.GetPort(port_id);
  ESP_RETURN_ON_FALSE(port != nullptr, ESP_ERR_NOT_FOUND, TAG,
                      "Invalid port number: %d", port_id);
  port->GetData(&fc_protocol, &temperature, &current, &voltage);

  ESP_LOGI(
      TAG,
      "Port: %d, Power stats -> Current: %d, Voltage: %d, Temperature: %d, "
      "Charging Protocol: %d",
      port_id, current, voltage, temperature, fc_protocol);
  response.emplace_back(fc_protocol);
  response.emplace_back(PortStatsData::ScaleAmperage(current));
  response.emplace_back(PortStatsData::ScaleVoltage(voltage));
  response.emplace_back(temperature & 0xff);

  return ESP_OK;
}

esp_err_t PowerHandler::GetPowerHistoricalStats(
    const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  if (request.size() <= 0) {
    ESP_LOGW(TAG, "Port required");
    return ESP_FAIL;
  }

  PortManager &portManager = PortManager::GetInstance();
  uint8_t port_id = request[0];
  Port *port = portManager.GetPort(port_id);
  ESP_RETURN_ON_FALSE(port != nullptr, ESP_ERR_NOT_FOUND, TAG,
                      "Invalid port number: %d", port_id);

  size_t offset = 0;
  if (request.size() == 2) {
    offset = request[1];
  } else if (request.size() == 3) {
    offset = (static_cast<size_t>(request[2]) << 8) | request[1];
  }

  size_t length = port->GetHistoricalStatsSize();
  ESP_LOGD(TAG, "offset: %d, buffer length: %d", offset, length);

  std::vector<PortStatsData> data =
      port->GetHistoricalStats().GetPointsInRange(offset, length);

  EMPLACE_BACK_INT16(response, offset);

  for (const PortStatsData &point : data) {
    response.emplace_back(point.GetAmperage());
    response.emplace_back(point.GetVoltage());
  }

  return ESP_OK;
}

esp_err_t PowerHandler::GetPowerSupplyStatus(
    const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  response.emplace_back(PortManager::GetInstance().GetOpenPortsBitMask());
  return ESP_OK;
}

esp_err_t PowerHandler::GetChargingStatus(const std::vector<uint8_t> &request,
                                          std::vector<uint8_t> &response) {
  uint8_t chargingStatus = PortManager::GetInstance().GetOpenPortsBitMask();
  response.emplace_back(chargingStatus);
  ESP_LOGI(TAG, "Charging status: %s",
           std::bitset<5>(chargingStatus).to_string().c_str());
  return ESP_OK;
}

esp_err_t PowerHandler::SetChargingStrategy(const std::vector<uint8_t> &request,
                                            std::vector<uint8_t> &response) {
  if (request.size() != 1) {
    return ESP_FAIL;
  }
  uint8_t strategy = request[0];
  PowerAllocator &allocator = PowerAllocator::GetInstance();
  switch (strategy) {
    case StrategyType::SLOW_CHARGING:
      allocator.SetStrategy<PowerSlowChargingStrategy>();
      break;
    case StrategyType::USBA_CHARGING:
      allocator.SetStrategy<PowerUSBAChargingStrategy>();
      break;
    default:
      ESP_LOGE(TAG, "Invalid charging strategy: %d", strategy);
      return ESP_ERR_INVALID_ARG;
  }
  return ESP_OK;
}

esp_err_t PowerHandler::SetPortPriority(const std::vector<uint8_t> &request,
                                        std::vector<uint8_t> &response) {
  esp_err_t __attribute__((unused)) ret;
  if (request.size() != NUM_PORTS) {
    ESP_LOGE(TAG, "Invalid request size");
    return ESP_FAIL;
  }
#ifdef CONFIG_MCU_MODEL_SW3566
  std::array<bool, NUM_PORTS> portsAlreadySet = {};
  std::array<uint8_t, NUM_PORTS> priority;
  for (uint8_t port = 0; port < NUM_PORTS; port++) {
    ESP_LOGI(TAG, "port: %d, request: %d", (int)port, (int)request[port]);
    if (portsAlreadySet[port]) {
      return ESP_FAIL;
    }
    portsAlreadySet[port] = true;
    priority[port] = request[port];
  }

  uint8_t oldPriorities[NUM_PORTS] = {};
  size_t length = NUM_PORTS;
  PowerNVSGetOrDefault(oldPriorities, &length, NVSKey::POWER_PORT_PRIORITY);

  for (uint8_t i = 0; i < NUM_PORTS; i++) {
    ESP_RETURN_ON_ERROR(rpc::fpga::set_mcu_priority(i, priority[i]), TAG,
                        "rpc::fpga::set_mcu_priority: %d=%d", i, priority[i]);
  }

  ESP_GOTO_ON_ERROR(PowerNVSSet(priority.data(), NVSKey::POWER_PORT_PRIORITY,
                                priority.size()),
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

esp_err_t PowerHandler::GetPortPriority(const std::vector<uint8_t> &request,
                                        std::vector<uint8_t> &response) {
  uint8_t priorities[NUM_PORTS] = {};
  size_t length = NUM_PORTS;
  PowerNVSGetOrDefault(priorities, &length, NVSKey::POWER_PORT_PRIORITY);

  for (const uint8_t &p : priorities) {
    response.emplace_back(p);
  }

  return ESP_OK;
}

esp_err_t PowerHandler::GetChargingStrategy(const std::vector<uint8_t> &request,
                                            std::vector<uint8_t> &response) {
  uint8_t strategy =
      static_cast<uint8_t>(PowerAllocator::GetInstance().GetStrategyType());
  response.emplace_back(strategy);
  return ESP_OK;
}

esp_err_t PowerHandler::GetPortPDStatus(const std::vector<uint8_t> &request,
                                        std::vector<uint8_t> &response) {
  if (request.size() != 1) {
    ESP_LOGW(TAG, "Invalid request size");
    return ESP_FAIL;
  }
  uint8_t port_index = request[0];
  ClientPDStatus pdstatus;
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

  const uint8_t *data = reinterpret_cast<const uint8_t *>(&pdstatus);
  response.insert(response.end(), data, data + sizeof(pdstatus));
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, response.data(), response.size(),
                           ESP_LOG_DEBUG);
  return ESP_OK;
}

esp_err_t PowerHandler::GetAllPowerStats(const std::vector<uint8_t> &request,
                                         std::vector<uint8_t> &response) {
  PortManager &pm = PortManager::GetInstance();
  for (const Port &port : pm) {
    ClientPDStatus pdstatus = {};
    PortPowerData data = port.GetData();
    port.GetPDStatus(&pdstatus);

    response.emplace_back(data.GetFCProtocol());
    response.emplace_back(PortStatsData::ScaleAmperage(data.GetCurrent()));
    response.emplace_back(PortStatsData::ScaleVoltage(data.GetVoltage()));
    response.emplace_back(data.GetTemperature());

    EMPLACE_BACK_INT16(response, pdstatus.battery_last_full_charge_capacity);
    EMPLACE_BACK_INT16(response, pdstatus.battery_present_capacity);
    ESP_LOGI(TAG,
             "PD stats[%d], current: %d, voltage: %d, temperature: %d, "
             "fc_protocol: %d, Last Full Capacity: %d, Present Capacity: %d",
             port.Id(), data.GetCurrent(), data.GetVoltage(),
             data.GetTemperature(), data.GetFCProtocol(),
             pdstatus.battery_last_full_charge_capacity * 100,
             pdstatus.battery_present_capacity * 100);
  }
  return ESP_OK;
}

esp_err_t PowerHandler::GetStartChargeTimestamp(
    const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  uint32_t ts = PortManager::GetInstance().GetChargingAt();
  ESP_LOGI(TAG, "start charge timestamp: %" PRIu32, ts);
  EMPLACE_BACK_INT32(response, ts);
  return ESP_OK;
}

esp_err_t PowerHandler::TurnOnPort(const std::vector<uint8_t> &request,
                                   std::vector<uint8_t> &response) {
  if (request.size() != 1) {
    ESP_LOGE(TAG, "Invalid request size: %d", request.size());
    return ESP_FAIL;
  }
  DeviceController &controller = DeviceController::GetInstance();
  ESP_RETURN_ON_FALSE(controller.is_power_on(), ESP_ERR_NOT_SUPPORTED, TAG,
                      "Device is power off. Turning on port is forbidden");
  uint8_t port_id = request[0];
  PortManager &pm = PortManager::GetInstance();
  Port *port = pm.GetPort(port_id);
  ESP_RETURN_ON_FALSE(port != nullptr, ESP_ERR_NOT_FOUND, TAG,
                      "Invalid port index: %d", port_id);
  return port->Open() ? ESP_OK : ESP_FAIL;
}

esp_err_t PowerHandler::TurnOffPort(const std::vector<uint8_t> &request,
                                    std::vector<uint8_t> &response) {
  if (request.size() != 1) {
    ESP_LOGE(TAG, "Invalid request size: %d", request.size());
    return ESP_FAIL;
  }
  DeviceController &controller = DeviceController::GetInstance();
  ESP_RETURN_ON_FALSE(controller.is_power_on(), ESP_ERR_NOT_SUPPORTED, TAG,
                      "Device is power off. Turning off port is forbidden");
  uint8_t port_id = request[0];
  PortManager &pm = PortManager::GetInstance();
  Port *port = pm.GetPort(port_id);
  ESP_RETURN_ON_FALSE(port != nullptr, ESP_ERR_NOT_FOUND, TAG,
                      "Invalid port index: %d", port_id);
  return port->Close() ? ESP_OK : ESP_FAIL;
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

esp_err_t PowerHandler::SetStaticAllocator(const std::vector<uint8_t> &request,
                                           std::vector<uint8_t> &response) {
  if (request.size() != sizeof(PowerAllocationTable) + sizeof(uint16_t)) {
    ESP_LOGE(TAG, "Invalid request size: %d", request.size());
    return ESP_ERR_INVALID_SIZE;
  }
  uint16_t identifier = (request[1] << 8) | request[0];
  PowerAllocator &allocator = PowerAllocator::GetInstance();
  PowerAllocationTable table;
  std::memcpy(&table, request.data() + 2, request.size() - 2);
  ESP_RETURN_ON_ERROR(validate_power_allocation_table(table), TAG,
                      "Invalid power allocation table");
  allocator.SetStrategy<PowerStaticChargingStrategy>(table, identifier);
  EMPLACE_BACK_INT16(response, identifier);
  ESP_LOGI(TAG, "Power allocation table set");
  return ESP_OK;
}

esp_err_t PowerHandler::GetStaticAllocator(const std::vector<uint8_t> &request,
                                           std::vector<uint8_t> &response) {
  auto static_strategy = PowerAllocator::GetInstance()
                             .GetStrategyAs<PowerStaticChargingStrategy>();
  if (static_strategy == nullptr) {
    ESP_LOGE(TAG, "Not static strategy type");
    return ESP_ERR_INVALID_STATE;
  }

  uint16_t identifier;
  static_strategy->GetIdentifier(&identifier);
  uint8_t version;
  static_strategy->GetVersion(&version);
  EMPLACE_BACK_INT16(response, identifier);
  response.emplace_back(version);
  return ESP_OK;
}

static int get_port_config_size(uint8_t version) {
  switch (version) {
    case 0:
      return 3;
    default:
      return sizeof(PortConfig);
  }
}

esp_err_t PowerHandler::SetPortConfig(const std::vector<uint8_t> &request,
                                      std::vector<uint8_t> &response) {
  if (request.size() < 1 + 8) {
    ESP_LOGE(TAG, "Invalid SetPortConfigs request size: %d", request.size());
    return ESP_ERR_INVALID_SIZE;
  }
  uint8_t port_mask = request[0];
  uint8_t version = request[1];
  int config_size = get_port_config_size(version);
  if (request.size() != 1 + 8 * config_size) {
    ESP_LOGE(TAG, "Invalid SetPortConfigs request size: %d", request.size());
    return ESP_ERR_INVALID_SIZE;
  }
  PortConfig configs[8];
  for (int i = 0; i < 8; i++) {
    std::memcpy(&configs[i], &request[1 + i * config_size], config_size);
  }
  PortManager &pm = PortManager::GetInstance();
  for (Port &port : pm) {
    if (((port_mask >> port.Id()) & 0x1) == 0) {
      continue;
    }
    ESP_RETURN_ON_ERROR(
        PowerNVSSet(reinterpret_cast<uint8_t *>(&configs[port.Id()]),
                    port_config_keys[port.Id()], sizeof(PortConfig)),
        TAG, "PowerNVSData::SavePortConfig %d", port.Id());
    if (port.Dead()) {
      // If the port is dead, only save the config to NVS, but do not apply it
      continue;
    }
    ESP_RETURN_ON_ERROR(port.SetConfig(configs[port.Id()]), TAG,
                        "Port::SetConfig %d", port.Id());
  }
  return ESP_OK;
}

esp_err_t PowerHandler::GetPortConfig(const std::vector<uint8_t> &request,
                                      std::vector<uint8_t> &response) {
  uint8_t version = 0;
  if (request.size() == 1) {
    version = request[0];
  }
  int config_size = get_port_config_size(version);
  PortConfig configs[8] = {};
  size_t i = 0;
  for (const Port &port : PortManager::GetInstance()) {
    size_t length = sizeof(PortConfig);
    esp_err_t err = PowerNVSGet(reinterpret_cast<uint8_t *>(&configs[i]),
                                &length, port_config_keys[i]);
    ESP_RETURN_ON_FALSE(err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND, err, TAG,
                        "Failed to get port %d config", i);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
      configs[i] = port.GetConfig();
    } else {
      configs[i].features = port.MigrateFeatures(configs[i]);
    }
    configs[i].version = version;
    i++;
  }
  for (i = 0; i < 8; i++) {
    uint8_t *data_ptr = reinterpret_cast<uint8_t *>(&configs[i]);
    response.insert(response.end(), data_ptr, data_ptr + config_size);
  }
  return ESP_OK;
}

struct CompatibilitySettings {
  bool isTfcpEnabled : 1;   //!< whether to enable tfcp
  bool isFcpEnabled : 1;    //!< whether to enable FCP
  bool isUfcsEnabled : 1;   //!< whether to enable ufcs
  bool isHvScpEnabled : 1;  //!< whether to enable high voltage scp
  bool isLvScpEnabled : 1;  //!< whether to enable low voltage scp
  uint8_t unused : 3;
};

esp_err_t PowerHandler::SetPortCompatibilitySettings(
    const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  if (request.size() != sizeof(CompatibilitySettings)) {
    ESP_LOGE(TAG, "Invalid request size: %d", request.size());
    return ESP_ERR_INVALID_SIZE;
  }
  CompatibilitySettings settings;
  memcpy(&settings, &request[0], sizeof(settings));

  PortConfig config;
  PortManager &pm = PortManager::GetInstance();
  for (Port &port : pm) {
    config = port.GetConfig();

    config.features.EnableTfcp = settings.isTfcpEnabled;
    config.features.EnableFcp = settings.isFcpEnabled;
    config.features.EnableUfcs = settings.isUfcsEnabled;
    config.features.EnableHvScp = settings.isHvScpEnabled;
    config.features.EnableLvScp = settings.isLvScpEnabled;

    NVSKey key = port_config_keys[port.Id()];
    ESP_RETURN_ON_ERROR(PowerNVSSet(reinterpret_cast<uint8_t *>(&config), key,
                                    sizeof(PortConfig)),
                        TAG, "PowerNVSData::SavePortConfig %d", port.Id());
    if (port.Dead()) {
      // If the port is dead, only save the config to NVS, but do not apply it
      continue;
    }

    ESP_RETURN_ON_ERROR(port.SetConfig(config), TAG, "Port::SetConfig %d",
                        port.Id());
  }
  return ESP_OK;
}

esp_err_t PowerHandler::GetPortCompatibilitySettings(
    const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
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
    PortConfig config;
    size_t length = sizeof(PortConfig);
    NVSKey key = port_config_keys[port.Id()];
    esp_err_t err =
        PowerNVSGet(reinterpret_cast<uint8_t *>(&config), &length, key);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
      config = port.GetConfig();
    } else {
      ESP_RETURN_ON_ERROR(err, TAG, "Failed to get port %d config", port.Id());
    }
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
  response.insert(response.end(), reinterpret_cast<uint8_t *>(&settings),
                  reinterpret_cast<uint8_t *>(&settings) + sizeof(settings));
  return ESP_OK;
}

esp_err_t PowerHandler::SetTemperatureMode(const std::vector<uint8_t> &request,
                                           std::vector<uint8_t> &response) {
  if (request.size() != 1) {
    ESP_LOGE(TAG, "Invalid request size: %d", request.size());
    return ESP_ERR_INVALID_SIZE;
  }
  ESP_RETURN_ON_FALSE(request[0] == 0 || request[0] == 1, ESP_ERR_INVALID_ARG,
                      TAG, "Invalid request value: %d", request[0]);
  return PowerAllocator::GetInstance().SetTemperatureMode(
      static_cast<TemperatureMode>(request[0]));
}

typedef struct {
  uint8_t open_status;
  uint8_t connected_status;
  esp_timer_handle_t timer_handle;
  int64_t starts_at;
} ExitTemporaryAllocatorTaskArg;

void exit_temporary_allocator_task(void *arg) {
  static bool power_on = true;
  static int connected_change_count = 0;
  esp_err_t ret __attribute__((unused)) = ESP_OK;
  ExitTemporaryAllocatorTaskArg *task_arg =
      reinterpret_cast<ExitTemporaryAllocatorTaskArg *>(arg);
  if (esp_timer_get_time() <= task_arg->starts_at) {
    return;
  }

  DeviceController &controller = DeviceController::GetInstance();
  PowerAllocator &allocator = PowerAllocator::GetInstance();
  PortManager &pm = PortManager::GetInstance();

  uint8_t intensity = 0x80;
  uint8_t mode = DisplayMode::POWER_METER;
  uint8_t open_status = pm.GetOpenPortsBitMask();
  uint8_t connected_status = pm.GetAttachedPortsBitMask();
  if (controller.is_power_on()) {
    if (!power_on) {
      ESP_LOGI(TAG, "Exit temporary allocator on power on");
      goto SLOW_CHARGING;
    }
  } else {
    power_on = false;
    return;
  }
  if (allocator.Type() != PowerAllocatorType::TEMPORARY_ALLOCATOR) {
    ESP_LOGI(TAG, "Exit temporary allocator on strategy change");
    goto DELETE;
  }
  if (open_status != task_arg->open_status) {
    ESP_LOGI(TAG, "Exit temporary allocator on port open status change");
    goto SLOW_CHARGING;
  }
  if (connected_status != task_arg->connected_status) {
    connected_change_count++;
  } else {
    connected_change_count = 0;
  }
  if (connected_change_count >= 2) {
    connected_change_count = 0;
    ESP_LOGI(TAG, "Exit temporary allocator on port connected status change");
    goto SLOW_CHARGING;
  }
  return;

SLOW_CHARGING:
  allocator.SetStrategy<PowerSlowChargingStrategy>();
DELETE:
  PowerAllocatorType type = allocator.Type();
  if (type == PowerAllocatorType::TEMPORARY_ALLOCATOR ||
      type == PowerAllocatorType::FLEXAI_FAST_ALLOCATOR) {
    ESP_ERROR_COMPLAIN(
        DisplayManager::GetInstance().SetDisplayIntensity(intensity),
        "DisplayManager::SetDisplayIntensity");
    ESP_ERROR_COMPLAIN(DisplayManager::GetInstance().SetDisplayMode(mode),
                       "DisplayManager::SetDisplayMode");
  }
  power_on = true;
  esp_timer_stop(task_arg->timer_handle);
  esp_timer_delete(task_arg->timer_handle);
  task_arg->timer_handle = nullptr;
}

esp_err_t PowerHandler::SetTemporaryAllocator(
    const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  static ExitTemporaryAllocatorTaskArg task_arg = {
      .open_status = 0,
      .connected_status = 0,
      .timer_handle = nullptr,
      .starts_at = 0,
  };

  if (request.size() != NUM_PORTS) {
    ESP_LOGE(TAG, "Invalid request size: %d", request.size());
    return ESP_ERR_INVALID_SIZE;
  }

  PortManager &pm = PortManager::GetInstance();
  uint8_t allocation_table[NUM_PORTS] = {};
  uint8_t sum = 0;
  for (const Port &port : pm) {
    uint8_t allocation = request[port.Id()];
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

  if (task_arg.timer_handle != nullptr) {
    esp_timer_stop(task_arg.timer_handle);
    esp_timer_delete(task_arg.timer_handle);
    task_arg.timer_handle = nullptr;
  }

  task_arg.open_status = pm.GetOpenPortsBitMask();
  task_arg.connected_status = pm.GetAttachedPortsBitMask();
  task_arg.starts_at = esp_timer_get_time() + 5 * 1e6;
  PowerAllocator::GetInstance().SetStrategy<PowerTemporaryChargingStrategy>(
      allocation_table);

  esp_timer_create_args_t timer_args = {
      .callback = &exit_temporary_allocator_task,
      .arg = (void *)&task_arg,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "exit_temporary_allocator",
      .skip_unhandled_events = true,
  };
  const uint64_t timer_interval_us = 1 * 1e6;
  esp_timer_create(&timer_args, &task_arg.timer_handle);
  esp_timer_start_periodic(task_arg.timer_handle, timer_interval_us);

  return ESP_OK;
}

esp_err_t PowerHandler::SubscribePDPcapStreaming(
    const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
#ifdef CONFIG_MCU_MODEL_SW3566
  // port id + subscribe
  ESP_RETURN_ON_FALSE(request.size() == sizeof(bool) + sizeof(uint8_t),
                      ESP_ERR_INVALID_SIZE, TAG, "Invalid request size: %d",
                      request.size());
  uint8_t port_id = request[0];
  bool subscribe = bool(request[1]);
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

esp_err_t PowerHandler::SetPortType(const std::vector<uint8_t> &request,
                                    std::vector<uint8_t> &response) {
  if (request.size() != 2) {
    ESP_LOGE(TAG, "Invalid request size: %d", request.size());
    return ESP_ERR_INVALID_SIZE;
  }
  uint8_t port_id = request[0];
  uint8_t port_type = request[1];
#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
  ESP_RETURN_ON_FALSE(port_id > 0, ESP_ERR_INVALID_ARG, TAG,
                      "Invalid port id: %d", port_id);
#endif
  ESP_RETURN_ON_FALSE(port_type < PortType::PORT_TYPE_MAX, ESP_ERR_INVALID_ARG,
                      TAG, "Invalid port type: %d", port_type);
  PortManager &pm = PortManager::GetInstance();
  Port *port = pm.GetPort(port_id);
  ESP_RETURN_ON_ERROR(port->SetPortType(static_cast<PortType>(port_type)), TAG,
                      "Port::SetPortType %d", port_id);
  esp_err_t err = PowerNVSEraseKey(port_config_keys[port_id]);
  ESP_RETURN_ON_FALSE(err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND, err, TAG,
                      "Failed to erase port %d config", port_id);
  ESP_RETURN_ON_ERROR(port->ResetConfig(), TAG, "Port::ResetConfig %d",
                      port_id);
  return ESP_OK;
}
