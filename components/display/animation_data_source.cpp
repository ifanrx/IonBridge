#include "animation_data_source.h"

#include <numeric>

#include "data_types.pb.h"
#include "machine_info.h"
#include "nvs_key.h"
#include "nvs_namespace.h"
#include "port_manager.h"
#include "wifi.h"

using nvs_namespace::user_data::DISPLAY_DATA;

// Static storage for last inserted port ID
static uint8_t s_last_inserted_port_id = 0;

uint8_t AnimationDataSource::GetAttachedPortCount() {
  return PortManager::GetInstance().GetAttachedPortCount();
}

std::vector<PortChargingInfo> AnimationDataSource::GetPortsData() {
  std::vector<PortChargingInfo> result;
  PortManager& port_manager = PortManager::GetInstance();

  for (const Port& port : port_manager) {
    if (port.Attached()) {
      const PortPowerData& data = port.GetData();
      PortChargingInfo info = {};
      info.port_id = port.Id();
      info.power_w = data.GetPower();
      info.voltage_mv = data.GetVoltage();
      info.current_ma = data.GetCurrent();
      info.fc_protocol = data.GetFCProtocol();
      result.push_back(info);
    }
  }

  return result;  // RVO/move optimization makes this efficient
}

uint16_t AnimationDataSource::GetTotalWatts() {
  PortManager& pm = PortManager::GetInstance();
  return std::accumulate(
      pm.begin(), pm.end(), uint16_t{0}, [](uint16_t sum, const Port& port) {
        return port.Attached() ? sum + port.GetData().GetPower() : sum;
      });
}

std::string AnimationDataSource::GetPSN() {
  return MachineInfo::GetInstance().GetPSN();
}

std::array<uint8_t, 6> AnimationDataSource::GetBleMac() {
  return MachineInfo::GetInstance().GetBleMac();
}

std::array<uint8_t, 4> AnimationDataSource::GetPin() {
  return MachineInfo::GetInstance().GetPin();
}

std::string AnimationDataSource::GetProductFamily() const {
  return MachineInfo::GetInstance().GetProductFamily();
}

bool AnimationDataSource::IsWifiConnected() { return wifi_is_connected(); }

std::vector<PowerDataPoint> AnimationDataSource::GetHistoricalPowerData(
    size_t max_points) {
  std::vector<PowerDataPoint> result;
  std::vector<uint16_t> aggregated_power(max_points, 0);

  // Collect historical stats from all ports and aggregate
  PortManager& port_manager = PortManager::GetInstance();
  for (const Port& port : port_manager) {
    if (!port.Activated()) {
      continue;
    }

    size_t length = port.GetHistoricalStatsSize();
    size_t offset = length > max_points ? length - max_points : 0;
    std::vector<PortStatsData> port_data =
        port.GetHistoricalStats().GetPointsInRange(offset, length);

    // Sum power across all ports for each time point
    for (size_t i = 0; i < port_data.size() && i < max_points; i++) {
      uint16_t power =
          port_data[i].GetAmperage() * port_data[i].GetVoltage() / 1e6;
      aggregated_power[i] += power;
    }
  }

  // Convert to PowerDataPoint array (filter out zero points)
  for (size_t i = 0; i < max_points; i++) {
    if (aggregated_power[i] > 0) {
      PowerDataPoint point;
      point.timestamp = i;                          // Use index as timestamp
      point.power_mw = aggregated_power[i] * 1000;  // W to mW
      point.port_id = 0;                            // Combined power
      result.push_back(point);
    }
  }

  return result;
}

uint8_t AnimationDataSource::GetLastInsertedPortId() {
  return s_last_inserted_port_id;
}

void AnimationDataSource::SetLastInsertedPortId(uint8_t port_id) {
  s_last_inserted_port_id = port_id;
}

uint8_t AnimationDataSource::GetIdleAnimationType() {
  uint8_t type = 0;  // Default defined in nvs_default.cpp
  DISPLAY_DATA.GetOrDefault(nvs_keys::IDLE_ANIMATION, type);
  return type;
}

void AnimationDataSource::SetIdleAnimation(uint8_t type) {
  DISPLAY_DATA.Set(nvs_keys::IDLE_ANIMATION, type);
}
