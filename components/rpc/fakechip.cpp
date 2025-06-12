#include <string.h>
#include <sys/param.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <numbers>
#include <numeric>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "ionbridge.h"
#include "rpc.h"
#include "sdkconfig.h"
#include "sw3566_data_types.h"

static const char *TAG = "FakeChip";

#define SW3566_COUNT CONFIG_FAKE_SW3566_COUNT

typedef enum {
  Iout,
  Vout,
  Die,
  Vin,
} ADC;

int32_t port_fc_proto[SW3566_COUNT];
int64_t port_uptime[SW3566_COUNT];
uint32_t port_power[SW3566_COUNT], port_power_cap[SW3566_COUNT];
const uint32_t total_power = 160000;
uint32_t remaining_power = total_power;
uint16_t port_current[SW3566_COUNT], port_voltage[SW3566_COUNT];
bool port_connected[SW3566_COUNT];

esp_err_t read_adc(uint8_t mcu, ADC adc, bool raw, uint16_t *value);
uint8_t generate_nonce();

esp_err_t rpc::mcu::send_command(uint8_t mcu, uint16_t command,
                                 const uint8_t *request, uint8_t request_size,
                                 uint8_t *response, uint8_t *response_size) {
  // Every command requires at least one byte of request data
  if (request_size == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  ESP_LOGI(TAG, "Fake command 0x%04X to SW3566 %d, request size: %d", command,
           mcu, request_size);
  return ESP_OK;
}

esp_err_t rpc::mcu::keep_alive(uint8_t mcu, KeepAliveStatus *status,
                               uint32_t *uptime_ms, uint32_t *reboot_reason) {
  KeepAliveResponse res{
      .nonce = generate_nonce(),
      .status = USER_APPLICATION_ALIVE,
  };
  if (status != nullptr) {
    memcpy(status, &res.status, sizeof(res.status));
  }
  if (uptime_ms != nullptr) {
    *uptime_ms = port_uptime[mcu];
  }
  if (reboot_reason != nullptr) {
    *reboot_reason = 0;
  }
  return ESP_OK;
}

esp_err_t rpc::mcu::boot(uint8_t mcu) { return ESP_OK; }

esp_err_t rpc::mcu::bringup(uint8_t mcu, bool skip_upgrade) {
  ESP_LOGI(TAG, "SW3566 %d is now brought up", mcu);
  port_fc_proto[mcu] = FC_NOT_CHARGING;
  return ESP_OK;
}

esp_err_t rpc::mcu::boot_all(bool *booted, uint8_t count, bool skip_upgrade) {
  if (booted == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  for (uint8_t i = 0; i < count; i++) {
    if (!booted[i]) {
      continue;
    }
    esp_err_t err = bringup(i, skip_upgrade);
    booted[i] = (err == ESP_OK);
    if (!booted[i]) {
      ESP_ERROR_COMPLAIN(booted[i], "Failed to boot SW3566 %d", i);
    }
    port_connect(i);
  }
  return ESP_OK;
}

esp_err_t rpc::mcu::set_port_state(uint8_t mcu, bool shutdown) {
  if (shutdown) {
    port_fc_proto[mcu] = FC_NOT_CHARGING;
    port_connected[mcu] = false;
    return ESP_OK;
  }
  port_fc_proto[mcu] = esp_random() % 7 + 12;
  port_uptime[mcu] = esp_timer_get_time();
  port_connected[mcu] = true;
  return ESP_OK;
}

esp_err_t rpc::mcu::set_subscription(uint8_t mcu, Subscriptions &subs) {
  return ESP_OK;
}

esp_err_t read_adc(uint8_t mcu, ADC adc, bool raw, uint16_t *value) {
  double time_in_radians = (esp_timer_get_time() / 5000000.0);
  double phase_shift = std::numbers::pi * (mcu + 2) / 7.0;

  // Calculate sine value with applied phase shift.
  double val = std::sin(time_in_radians + phase_shift);
  double scaled_val = (val + 1.0) / 2.0;  // Normalize val to range [0, 1]

  // Determine range and scale accordingly.
  uint16_t new_value, max_value;
  uint32_t max_power = (mcu == 0 ? 60 : 140) * 1e3;  // mW
  switch (adc) {
    case Iout:
      max_value = 5000;
      break;
    case Vout:
      max_value = mcu == 0 ? 12000 : 28000;
      break;
    case Die:
      max_value = 50;
      break;
    case Vin:
      max_value = 40000;
      break;
    default:
      return ESP_ERR_INVALID_ARG;
  }

  new_value = static_cast<uint16_t>(scaled_val * max_value);
  new_value = MIN(new_value, max_value);
  if (adc != Vout && adc != Iout) {
    *value = new_value;
    return ESP_OK;
  }

  new_value /= 2;
  remaining_power =
      total_power - std::accumulate(port_power, port_power + SW3566_COUNT, 0);
  remaining_power = MIN(remaining_power + port_power[mcu], total_power);
  if (adc == Vout) {
    port_voltage[mcu] = new_value;
    port_power[mcu] = (new_value * port_current[mcu]) / 1e3;
  } else {
    port_current[mcu] = new_value;
    port_power[mcu] = (port_voltage[mcu] * new_value) / 1e3;
  }

  port_power[mcu] = MIN(port_power[mcu], max_power);
  port_power[mcu] = MIN(port_power[mcu], remaining_power);
  remaining_power -= port_power[mcu];
  ESP_LOGD(TAG,
           "SW3566 %d ADC %d value: %d, power: %" PRIu32
           ", remaining: %" PRIu32,
           mcu, adc, new_value, port_power[mcu], remaining_power);
  port_current[mcu] = MIN(
      static_cast<uint16_t>(port_power[mcu] * 1e3 / port_voltage[mcu]), 5000);
  *value = new_value;
  return ESP_OK;
}

esp_err_t get_port_connected(uint8_t mcu, bool *connected) {
  *connected = port_connected[mcu];
  return ESP_OK;
}

esp_err_t rpc::mcu::get_pd_status(uint8_t mcu, ClientPDStatus *status) {
  uint16_t bat_capacity = static_cast<uint16_t>(esp_random()) & 0xFFF;
  uint16_t battery_design_capacity =
      bat_capacity + (esp_random() % bat_capacity) * 0.2;
  uint16_t battery_last_full_charge_capacity =
      bat_capacity - (esp_random() % bat_capacity) * 0.2;

  ClientPDStatus pdstatus = {
      .battery_vid = 0x19d1,
      .battery_pid = static_cast<uint16_t>(esp_random()),
      .battery_design_capacity = bat_capacity,
      .battery_last_full_charge_capacity = battery_design_capacity,
      .battery_present_capacity = battery_last_full_charge_capacity,
      .cable_vid = 0x05ac,
      .cable_pid = static_cast<uint16_t>(esp_random()),
      .manufacturer_vid = 0x36e9,
      .manufacturer_pid = static_cast<uint16_t>(esp_random()),
  };
  memcpy(status, &pdstatus, sizeof(pdstatus));
  return ESP_OK;
}

esp_err_t rpc::mcu::get_port_details(uint8_t mcu, PortDetails *details) {
  bool connected = false;
  FastChargingProtocol protocol = FC_NOT_CHARGING;
  uint16_t die_temperature;
  uint16_t iout;
  uint16_t vin;
  uint16_t vout;

  read_adc(mcu, Die, true, &die_temperature);
  read_adc(mcu, Iout, true, &iout);
  read_adc(mcu, Vin, true, &vin);
  read_adc(mcu, Vout, true, &vout);

  // Determine fast charging protocol and connection status
  protocol = static_cast<FastChargingProtocol>(port_fc_proto[mcu]);
  get_port_connected(mcu, &connected);

  // Populate PortDetails structure
  PortDetails result = {
      .connected = connected,
      .fc_protocol = protocol,
      .iout_value = iout,
      .vout_value = vout,
      .die_temperature = die_temperature,
      .vin_value = vin,
  };

  // Copy the populated details to the output parameter
  memcpy(details, &result, sizeof(result));

  return ESP_OK;
}

esp_err_t rpc::mcu::set_power_features(uint8_t mcu, PowerFeatures features) {
  return ESP_OK;
}

esp_err_t rpc::mcu::get_power_features(uint8_t mcu, PowerFeatures *features) {
  PowerFeatures f = {};
  memcpy(features, &f, sizeof(PowerFeatures));

  return ESP_OK;
}

esp_err_t rpc::mcu::set_max_power_budget(uint8_t mcu, uint8_t budget,
                                         uint8_t watermark,
                                         bool force_rebroadcast,
                                         uint8_t *set_budget) {
  // Limit budget based on MCU type
  budget = MIN(budget, (mcu == 0) ? 60 : 140);

  // Update port power and current parameters
  port_power_cap[mcu] = budget * 1000;
  port_current[mcu] = esp_random() % 1000 + 500;  // Current in mA
  port_voltage[mcu] =
      (port_power_cap[mcu] / port_current[mcu]) * 1000;  // Voltage in mV

  // Ensure voltage does not exceed the maximum limit for each MCU type
  port_voltage[mcu] =
      MIN(port_voltage[mcu], (mcu == 0) ? 12000 : 28000);  // Voltage in mV

  // Calculate power in milliwatts
  port_power[mcu] = port_voltage[mcu] * port_current[mcu] / 1e3;  // Power in mW

  // Adjust power if it exceeds the remaining available power
  if (port_power[mcu] > remaining_power) {
    port_power[mcu] = remaining_power;
    port_current[mcu] = port_power[mcu] * 1000 / port_voltage[mcu];
  }

  // Deduct the allocated power from the remaining power
  remaining_power -= port_power[mcu];

  // Log the updated power budget details
  ESP_LOGI(TAG,
           "Set max power budget to %d for SW3566 %d, current: %dmA, voltage: "
           "%dmV, remaining: %" PRIu32 "mW",
           budget, mcu, port_current[mcu], port_voltage[mcu], remaining_power);

  // Optionally return the set power budget
  if (set_budget != nullptr) {
    *set_budget = port_power[mcu] / 1e3;  // Convert mW to W
  }

  return ESP_OK;
}

esp_err_t rpc::mcu::get_max_power_budget(uint8_t mcu, uint8_t *budget) {
  if (budget != nullptr) {
    *budget = port_power_cap[mcu] / 1e3;
  }
  return ESP_OK;
}

esp_err_t rpc::mcu::set_system_flags(uint8_t mcu, SystemFlags *flags) {
  if (flags->commit == 0x55) {
    return ESP_OK;
  }
  *flags = {
      .commit = 0,
      .unused0 = true,
      .port_type = 1,
      .cable_compensation = 1,
      .unused = 0,
  };
  if (mcu == 0) {
    flags->port_type = 0;
  }
  return ESP_OK;
}
