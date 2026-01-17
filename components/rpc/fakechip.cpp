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
#include "fake_sw3566_data_types.h"
#include "ionbridge.h"
#include "rpc.h"
#include "sdkconfig.h"
#include "version.h"

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

// Session tracking variables
uint16_t port_session_id[SW3566_COUNT];
uint64_t port_session_charge[SW3566_COUNT];    // in microwatt-seconds (µWs)
uint64_t port_last_update_time[SW3566_COUNT];  // in microseconds

static struct {
  uint16_t manufacturer_vid;
  uint16_t manufacturer_pid;
  uint16_t battery_vid;
  uint16_t battery_pid;
  uint16_t cable_vid;
  uint16_t cable_pid;
} SINK_PD_INFO[SW3566_COUNT] = {
    {0, 0, 0, 0, 0, 0},                      // SW3566 0
    {0x05ac, 0x7315, 0, 0, 0x36e9, 0x3000},  // SW3566 1: MackBook Pro (M4 Pro)
    {0x05ac, 0x7117, 0, 0, 0x36e9, 0x3001},  // SW3566 2: iPad Pro 7th Gen
#if SW3566_COUNT > 3
    {0x05ac, 0x7515, 0, 0, 0x36e9, 0x3002},  // SW3566 3: iPhone 16 Series
#endif
#if SW3566_COUNT > 4
    {0x05ac, 0x7803, 0, 0, 0x36e9, 0x3003},  // SW3566 4: Vision Pro
#endif
};

esp_err_t read_adc(uint8_t mcu, ADC adc, bool raw, uint16_t *value);
uint8_t generate_nonce();
void update_session_charge(uint8_t mcu);
static uint32_t get_max_voltage(uint8_t mcu);
static uint32_t get_max_power_limit(uint8_t mcu);
static uint32_t get_max_iout_value(uint8_t mcu);

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
      .uptimeMS = 0,
      .rebootReason = 0,
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

esp_err_t rpc::mcu::reset_system(uint8_t mcu, bool wait_for_response) {
  ESP_LOGE(TAG, "SW3566 %d is now reset", mcu);
  return ESP_OK;
}

esp_err_t rpc::mcu::bringup(uint8_t mcu, bool, const Version *,
                            bool *upgraded) {
  ESP_LOGI(TAG, "SW3566 %d is now brought up", mcu);
  port_fc_proto[mcu] = FC_NOT_CHARGING;
  if (upgraded != nullptr) {
    *upgraded = false;  // Fake chip never upgrades
  }
  return ESP_OK;
}

esp_err_t rpc::mcu::boot_all(bool *booted, uint8_t count, bool skip_upgrade,
                             bool *any_upgraded) {
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

  if (any_upgraded != nullptr) {
    *any_upgraded = false;  // Fake chip never upgrades
  }
  return ESP_OK;
}

esp_err_t rpc::mcu::set_port_state(uint8_t mcu, bool shutdown) {
  if (shutdown) {
    port_fc_proto[mcu] = FC_NOT_CHARGING;
    port_connected[mcu] = false;
    // Reset session charge when port is detached
    port_session_charge[mcu] = 0;
    return ESP_OK;
  }
  port_fc_proto[mcu] = esp_random() % 6 + FC_PD_Fix5V;
  port_uptime[mcu] = esp_timer_get_time();
  port_connected[mcu] = true;

  // Increment session ID when port is attached
  port_session_id[mcu]++;
  // Reset session charge for new session
  port_session_charge[mcu] = 0;
  // Record start time for charge calculation
  port_last_update_time[mcu] = esp_timer_get_time();

  return ESP_OK;
}

esp_err_t rpc::mcu::set_subscription(uint8_t mcu, const Subscriptions &subs) {
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
  uint32_t max_power = (get_max_power_limit(mcu)) * 1e3;  // mW
  switch (adc) {
    case Iout:
      max_value = get_max_iout_value(mcu);
      break;
    case Vout:
      max_value = get_max_voltage(mcu);
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

void update_session_charge(uint8_t mcu) {
  if (!port_connected[mcu]) {
    return;  // Port not connected, no charge to update
  }

  uint64_t current_time = esp_timer_get_time();  // in microseconds
  uint64_t elapsed_time = current_time - port_last_update_time[mcu];

  // Use a fixed power value of 10W (10000 mW) for simulation
  const uint32_t fixed_power_mw = 10000;

  // Calculate charge increment: power (mW) * time (µs) = µWs
  uint64_t charge_increment = fixed_power_mw * elapsed_time;

  // Add to cumulative session charge
  port_session_charge[mcu] += charge_increment;

  // Update last update time
  port_last_update_time[mcu] = current_time;
}

esp_err_t rpc::mcu::get_pd_status(uint8_t mcu, ClientPDStatus *status) {
  uint16_t bat_capacity = static_cast<uint16_t>(esp_random()) & 0xFFF;
  uint16_t battery_design_capacity =
      bat_capacity + (esp_random() % bat_capacity) * 0.2;
  uint16_t battery_last_full_charge_capacity =
      bat_capacity - (esp_random() % bat_capacity) * 0.2;

  ClientPDStatus pdstatus = {};
  pdstatus.battery_vid = SINK_PD_INFO[mcu].battery_vid;
  pdstatus.battery_pid = SINK_PD_INFO[mcu].battery_pid;
  pdstatus.battery_design_capacity = bat_capacity;
  pdstatus.battery_last_full_charge_capacity = battery_design_capacity;
  pdstatus.battery_present_capacity = battery_last_full_charge_capacity;
  pdstatus.cable_vid = SINK_PD_INFO[mcu].cable_vid;
  pdstatus.cable_pid = SINK_PD_INFO[mcu].cable_pid;
  pdstatus.manufacturer_vid = SINK_PD_INFO[mcu].manufacturer_vid;
  pdstatus.manufacturer_pid = SINK_PD_INFO[mcu].manufacturer_pid;
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

  // Update session charge before reporting
  update_session_charge(mcu);

  read_adc(mcu, Die, true, &die_temperature);
  read_adc(mcu, Iout, true, &iout);
  read_adc(mcu, Vin, true, &vin);
  read_adc(mcu, Vout, true, &vout);

  // Determine fast charging protocol and connection status
  protocol = static_cast<FastChargingProtocol>(port_fc_proto[mcu]);
  get_port_connected(mcu, &connected);

  // Populate PortDetails structure
  PortDetails result = {};
  result.connected = connected;
  result.fc_protocol = protocol;
  result.iout_value = iout;
  result.vout_value = vout;
  result.die_temperature = die_temperature;
  result.vin_value = vin;

  // Add session tracking fields
  result.session_id = port_session_id[mcu];
  result.session_charge = port_session_charge[mcu];

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
  budget = MIN(budget, get_max_power_limit(mcu));

  // Update port power and current parameters
  port_power_cap[mcu] = budget * 1000;
  port_current[mcu] = esp_random() % 1000 + 500;  // Current in mA
  port_voltage[mcu] =
      (port_power_cap[mcu] / port_current[mcu]) * 1000;  // Voltage in mV

  // Ensure voltage does not exceed the maximum limit for each MCU type
  port_voltage[mcu] =
      MIN(port_voltage[mcu], get_max_voltage(mcu));  // Voltage in mV

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
      .enable_flash_cache = true,
      .port_type = 1,
      .enable_cable_compensation = true,
      .cable_compensation = 1,
      .fixed_voltage_offset = 0,
      .unused = 0,
  };
  if (mcu == 0) {
    flags->port_type = 0;
  }
  return ESP_OK;
}

uint32_t get_max_voltage(uint8_t mcu) {
  if (FAKE_SW3566_IS_TYPE_A(mcu)) {
    return 12000;
  }
#ifdef CONFIG_PRODUCT_FAMILY_VALUE
  if (strcmp(CONFIG_PRODUCT_FAMILY_VALUE, "PA768S") == 0) {
    return 20000;
  }
#endif
  return 28000;
}

uint32_t get_max_power_limit(uint8_t mcu) {
  return FAKE_SW3566_IS_TYPE_A(mcu) ? FAKE_SW3566_MIN_POWER   // Type A
                                    : FAKE_SW3566_MAX_POWER;
}

static uint32_t get_max_iout_value(uint8_t mcu) {
  if (FAKE_SW3566_IS_TYPE_A(mcu)) {
    return 3000;
  }
#ifdef CONFIG_PRODUCT_FAMILY_VALUE
  if (strcmp(CONFIG_PRODUCT_FAMILY_VALUE, "PA768S") == 0) {
    return 3000;
  }
#endif
  return 5000;
}
