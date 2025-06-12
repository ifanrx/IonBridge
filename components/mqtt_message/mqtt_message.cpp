#include "mqtt_message.h"

#include <cstring>

#include "esp_random.h"
#include "utils.h"

#define EMPLACE_BACK_VERSION(payload, version)                  \
  do {                                                          \
    (payload).emplace_back(static_cast<uint8_t>((version)[0])); \
    (payload).emplace_back(static_cast<uint8_t>((version)[1])); \
    (payload).emplace_back(static_cast<uint8_t>((version)[2])); \
  } while (0)

uint16_t MQTTMessageHeader::message_id_counter = uint16_t(esp_random());

MQTTMessageHeader::MQTTMessageHeader(uint16_t service) : service(service) {
  message_id = ++message_id_counter;
}

std::vector<uint8_t> MQTTMessageHeader::Serialize() const {
  std::vector<uint8_t> data(4);

  // Copy the bytes to the data vector using memcpy
  std::memcpy(&data[0], &service, sizeof(service));
  std::memcpy(&data[2], &message_id, sizeof(message_id));
  return data;
}

std::vector<uint8_t> StreamPortStatus::Serialize() const {
  std::vector<uint8_t> payload = header.Serialize();
  payload.reserve(sizeof(*this));
  payload.emplace_back(status);
  payload.emplace_back(port_status_map);

  // Append padding in Big-Endian format (network byte order)
  EMPLACE_BACK_INT16(payload, padding);

  // Append each PortStatusData
  for (const auto &port : ports) {
    // Create a pointer to the current port's data
    const uint8_t *portData = reinterpret_cast<const uint8_t *>(&port);

    // Append the bytes of PortStatusData to the payload
    payload.insert(payload.end(), portData, portData + sizeof(PortStatusData));
  }
  return payload;
}

std::vector<uint8_t> StreamPortPDStatus::Serialize() const {
  std::vector<uint8_t> payload = header.Serialize();
  payload.reserve(sizeof(*this));

  payload.emplace_back(status);
  payload.emplace_back(port_index);
  const uint8_t *data = reinterpret_cast<const uint8_t *>(&pd_status);
  payload.insert(payload.end(), data, data + sizeof(ClientPDStatus));
  return payload;
}

std::vector<uint8_t> StreamDeviceStatus::Serialize() const {
  std::vector<uint8_t> payload = header.Serialize();
  payload.reserve(sizeof(*this));
  payload.emplace_back(status);
  payload.emplace_back(allocator);
  payload.emplace_back(display_config.intensity);
  payload.emplace_back(display_config.flip);
  payload.emplace_back(display_config.mode);
  EMPLACE_BACK_INT16(payload, display_config.mode_param);
  EMPLACE_BACK_INT32(payload, device_uptime);
  EMPLACE_BACK_INT16(payload, feature_flag.value);
  EMPLACE_BACK_INT16(payload, charging_minutes);
  EMPLACE_BACK_INT16(payload, power_allocator_param);
  payload.emplace_back(temperature_mode);
  return payload;
}

std::vector<uint8_t> DeviceBootInfo::Serialize() const {
  std::vector<uint8_t> payload = header.Serialize();
  payload.reserve(sizeof(*this));
  EMPLACE_BACK_VERSION(payload, esp32);
  EMPLACE_BACK_VERSION(payload, mcu);
  EMPLACE_BACK_VERSION(payload, fpga);
  EMPLACE_BACK_VERSION(payload, zrlib);
  EMPLACE_BACK_VERSION(payload, ESPIdfVersion);
  payload.emplace_back(chipCores);
  payload.insert(payload.end(), bleAddress, bleAddress + sizeof(bleAddress));
  payload.insert(payload.end(), ipv4, ipv4 + sizeof(ipv4));
  payload.insert(payload.end(), ipv6, ipv6 + sizeof(ipv6));

  EMPLACE_BACK_INT64(payload, uptime);

  EMPLACE_BACK_INT16(payload, chipModel);
  EMPLACE_BACK_INT32(payload, chipFeatures);
  EMPLACE_BACK_INT16(payload, chipRevision);
  payload.emplace_back(wifiMaxTxPower);

  payload.emplace_back(activeMCUCount);
  EMPLACE_BACK_INT32(payload, fsTotalSize);
  EMPLACE_BACK_INT32(payload, fsUsedSize);
  EMPLACE_BACK_INT32(payload, resetReason);

  payload.insert(payload.end(), hardwareRev, hardwareRev + sizeof(hardwareRev));
  payload.insert(payload.end(), deviceModel, deviceModel + sizeof(deviceModel));
  payload.insert(payload.end(), productFamily,
                 productFamily + sizeof(productFamily));
  return payload;
}

std::vector<uint8_t> AggregatedPowerHistoricalData::Serialize() const {
  std::vector<uint8_t> payload = header.Serialize();
  payload.reserve(sizeof(*this));
  for (const PortStatsEntry &entry : data) {
    payload.emplace_back(entry.port);
    EMPLACE_BACK_INT16(payload, entry.length);
    for (const PortStatsData &point : entry.data) {
      payload.emplace_back(point.GetAmperage());
      payload.emplace_back(point.GetVoltage());
      payload.emplace_back(point.GetTemperature());
      payload.emplace_back(point.GetVinValue());
    }
  }
  return payload;
}

std::vector<uint8_t> PowerConsumptionData::Serialize() const {
  std::vector<uint8_t> payload = header.Serialize();
  payload.reserve(sizeof(*this));
  payload.emplace_back(port);
  EMPLACE_BACK_INT16(payload, length);
  EMPLACE_BACK_INT16(payload, power_consumption);
  return payload;
}

std::vector<uint8_t> DeviceMemoryInfo::Serialize() const {
  std::vector<uint8_t> payload = header.Serialize();
  payload.reserve(sizeof(*this));
  payload.emplace_back(initialBoot);
  EMPLACE_BACK_INT32(payload, freeHeapSize);
  EMPLACE_BACK_INT32(payload, freeInternalHeapSize);
  EMPLACE_BACK_INT32(payload, minimumFreeHeapSize);
  return payload;
}

std::vector<uint8_t> PowerAllocationData::Serialize() const {
  std::vector<uint8_t> payload = header.Serialize();
  payload.reserve(sizeof(*this));
  EMPLACE_BACK_INT16(payload, power_budget);
  EMPLACE_BACK_INT16(payload, remaining_power);
  EMPLACE_BACK_INT16(payload, adc_value);
  payload.emplace_back(temperature);
  payload.emplace_back(unused);
  for (auto point : port_power_allocations) {
    payload.emplace_back(point.source_cap);
    payload.emplace_back(point.usage);
  }
  return payload;
}

std::vector<uint8_t> PDPCapData::Serialize() const {
  std::vector<uint8_t> payload = header.Serialize();
  payload.reserve(sizeof(*this));
  payload.emplace_back(port);
  payload.insert(payload.end(), pcap_entry.begin(), pcap_entry.end());
  return payload;
}

std::vector<uint8_t> UpgradeInfo::Serialize() const {
  std::vector<uint8_t> payload = header.Serialize();
  payload.reserve(sizeof(*this));
  EMPLACE_BACK_VERSION(payload, version);
  payload.emplace_back(static_cast<uint8_t>(status));
  payload.emplace_back(static_cast<uint8_t>(reason));
  EMPLACE_BACK_INT32(payload, esp_err);
  return payload;
}

std::vector<uint8_t> OTAConfirmInfo::Serialize() const {
  std::vector<uint8_t> payload = header.Serialize();
  payload.reserve(sizeof(*this));
  payload.insert(payload.end(), hash, hash + sizeof(hash));
  payload.insert(payload.end(), esp32_version,
                 esp32_version + sizeof(esp32_version));
  return payload;
}

std::vector<uint8_t> WIFIStatsData::Serialize() const {
  std::vector<uint8_t> payload = header.Serialize();
  payload.reserve(sizeof(*this));
  EMPLACE_BACK_INT16(payload, associationCount);
  EMPLACE_BACK_INT16(payload, disassociationCount);
  payload.emplace_back(rssi);
  payload.emplace_back(channel);
  EMPLACE_BACK_INT16(payload, wifiConnectionTime);
  EMPLACE_BACK_INT16(payload, mqttConnectionTime);
  EMPLACE_BACK_INT16(payload, connectivityFailureCount);
  EMPLACE_BACK_INT16(payload, dnsResolutionFailureCount);
  EMPLACE_BACK_INT16(payload, mqttConnectionCount);
  EMPLACE_BACK_INT32(payload, mqttMessageTxCount);
  EMPLACE_BACK_INT32(payload, mqttMessageRxCount);
  return payload;
}

std::vector<uint8_t> UartMetricsData::Serialize() const {
  std::vector<uint8_t> payload = header.Serialize();
  payload.reserve(sizeof(*this));
  EMPLACE_BACK_INT32(payload, reset_state_count);
  EMPLACE_BACK_INT32(payload, resend_count);
  EMPLACE_BACK_INT32(payload, sent_failed_count);
  EMPLACE_BACK_INT32(payload, sent_count);
  return payload;
}

std::vector<uint8_t> OutOfMemoryAlertInfo::Serialize() const {
  std::vector<uint8_t> payload = header.Serialize();
  payload.reserve(sizeof(*this));
  EMPLACE_BACK_INT32(payload, freeHeapSize);
  EMPLACE_BACK_INT32(payload, freeInternalHeapSize);
  EMPLACE_BACK_INT32(payload, minimumFreeHeapSize);
  EMPLACE_BACK_INT32(payload, thresholdFreeHeapSize);
  return payload;
}

std::vector<uint8_t> OverTemperatureAlertInfo::Serialize() const {
  std::vector<uint8_t> payload = header.Serialize();
  payload.reserve(sizeof(*this));
  payload.emplace_back(temperature);
  payload.emplace_back(thresholdTemperature);
  return payload;
}

std::vector<uint8_t> ChargingAlertInfo::Serialize() const {
  std::vector<uint8_t> payload = header.Serialize();
  payload.reserve(sizeof(*this));
  payload.emplace_back(port);
  EMPLACE_BACK_INT16(payload, pd_rx_soft_reset_count);
  EMPLACE_BACK_INT16(payload, pd_rx_hard_reset_count);
  EMPLACE_BACK_INT16(payload, pd_rx_error_count);
  EMPLACE_BACK_INT16(payload, pd_rx_cable_reset_count);
  payload.emplace_back(unused);

  return payload;
}
