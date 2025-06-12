#include "nvs_default.h"

#include <cstdint>
#include <string>

#include "sdkconfig.h"

#define DISPLAY_DEFAULT_MODE CONFIG_DISPLAY_MODE
#define DISPLAY_DEFAULT_INTENSITY CONFIG_DISPLAY_INTENSITY

#ifdef CONFIG_DEFAULT_SYSLOG_REPORTING_ENABLED
#define SYSLOG_REPORT_STATE_DEFAULT true
#else
#define SYSLOG_REPORT_STATE_DEFAULT false
#endif

#define ADC_THRESHOLD_LOW CONFIG_ADC_THRESHOLD_LOW
#define ADC_THRESHOLD_HIGH CONFIG_ADC_THRESHOLD_HIGH
#define ACTION_DEADZONE CONFIG_ACTION_DEADZONE

#define NVS_KEY_INT(key) static_cast<int>(NVSKey::key)

static const std::string nvs_key_names[NVS_KEY_INT(NVS_KEY_COUNT)] = {
    [NVS_KEY_INT(DEVICE_SERIAL_NUMBER)] = "serial_number",
    [NVS_KEY_INT(DEVICE_PASSWORD)] = "password",
    [NVS_KEY_INT(DEVICE_NAME)] = "name",
    [NVS_KEY_INT(DEVICE_HARDWARE_REV)] = "hardware_rev",
    [NVS_KEY_INT(DEVICE_PRODUCT_FAMILY)] = "product_family",
    [NVS_KEY_INT(DEVICE_PRODUCT_COLOR)] = "color",
    [NVS_KEY_INT(DEVICE_AES_KEY)] = "aes_key",
    [NVS_KEY_INT(DEVICE_MODEL)] = "model",
    [NVS_KEY_INT(DEVICE_WIFI_MAC)] = "wifi_mac",
    [NVS_KEY_INT(DEVICE_BLUETOOTH_MAC)] = "bluetooth_mac",

    [NVS_KEY_INT(CA_CERT)] = "ca_cert",

    [NVS_KEY_INT(MQTT_CA_CERT)] = "mqtt_ca",
    [NVS_KEY_INT(MQTT_CERT)] = "mqtt_cert",
    [NVS_KEY_INT(MQTT_KEY)] = "mqtt_key",
    [NVS_KEY_INT(MQTT_BROKER)] = "mqtt_broker",
    [NVS_KEY_INT(MQTT_CUSTOM_BROKER)] = "mqtt_c_broker",

    [NVS_KEY_INT(DEVICE_TOKEN)] = "DEVICE_TOKEN",

    [NVS_KEY_INT(SYSLOG_REPORT_STATE)] = "report_state",

#ifdef CONFIG_ENABLE_RFTEST
    [NVS_KEY_INT(TEST_MODE_A)] = "test_mode_a",
#endif

    [NVS_KEY_INT(FPGA_CONFIG)] = "config",
    [NVS_KEY_INT(FPGA_POWER_CONTROL)] = "power_control",

    [NVS_KEY_INT(LICENSE)] = "license",

    [NVS_KEY_INT(DISPLAY_MODE)] = "mode",
    [NVS_KEY_INT(DISPLAY_INTENSITY)] = "intensity",
    [NVS_KEY_INT(DISPLAY_FLIP)] = "flip",
    [NVS_KEY_INT(DISPLAY_IDLE_ANIMATION)] = "idle_anim",

    [NVS_KEY_INT(POWER_PORT_PRIORITY)] = "port_priority",
    [NVS_KEY_INT(POWER_ALLOCATOR_ENABLED)] = "alloc_enabled",
    [NVS_KEY_INT(POWER_PORT0_CONFIG)] = "port_config_0",
    [NVS_KEY_INT(POWER_PORT1_CONFIG)] = "port_config_1",
    [NVS_KEY_INT(POWER_PORT2_CONFIG)] = "port_config_2",
    [NVS_KEY_INT(POWER_PORT3_CONFIG)] = "port_config_3",
    [NVS_KEY_INT(POWER_PORT4_CONFIG)] = "port_config_4",
    [NVS_KEY_INT(POWER_PORT5_CONFIG)] = "port_config_5",
    [NVS_KEY_INT(POWER_PORT6_CONFIG)] = "port_config_6",
    [NVS_KEY_INT(POWER_PORT7_CONFIG)] = "port_config_7",
    [NVS_KEY_INT(POWER_CONFIG)] = "power_config",

    [NVS_KEY_INT(WIFI_NUMBER)] = "number",
    [NVS_KEY_INT(WIFI_SSID_0)] = "SSID-0",
    [NVS_KEY_INT(WIFI_PASSWD_0)] = "PASSWD-0",
    [NVS_KEY_INT(WIFI_SSID_1)] = "SSID-1",
    [NVS_KEY_INT(WIFI_PASSWD_1)] = "PASSWD-1",
    [NVS_KEY_INT(WIFI_HASH)] = "hash",
    [NVS_KEY_INT(WIFI_CREDENTIAL_1)] = "credential_01",
    [NVS_KEY_INT(WIFI_CREDENTIAL_2)] = "credential_02",
    [NVS_KEY_INT(WIFI_CREDENTIAL_3)] = "credential_03",
    [NVS_KEY_INT(WIFI_CREDENTIAL_4)] = "credential_04",
    [NVS_KEY_INT(WIFI_CREDENTIAL_5)] = "credential_05",
    [NVS_KEY_INT(WIFI_CREDENTIAL_6)] = "credential_06",
    [NVS_KEY_INT(WIFI_CREDENTIAL_7)] = "credential_07",
    [NVS_KEY_INT(WIFI_CREDENTIAL_8)] = "credential_08",
    [NVS_KEY_INT(WIFI_CREDENTIAL_9)] = "credential_09",
    [NVS_KEY_INT(WIFI_CREDENTIAL_10)] = "credential_10",
    [NVS_KEY_INT(WIFI_CREDENTIAL_11)] = "credential_11",
    [NVS_KEY_INT(WIFI_CREDENTIAL_12)] = "credential_12",
    [NVS_KEY_INT(WIFI_CREDENTIAL_13)] = "credential_13",
    [NVS_KEY_INT(WIFI_CREDENTIAL_14)] = "credential_14",
    [NVS_KEY_INT(WIFI_CREDENTIAL_15)] = "credential_15",
    [NVS_KEY_INT(WIFI_CREDENTIAL_16)] = "credential_16",
    [NVS_KEY_INT(WIFI_CREDENTIAL_17)] = "credential_17",
    [NVS_KEY_INT(WIFI_CREDENTIAL_18)] = "credential_18",
    [NVS_KEY_INT(WIFI_CREDENTIAL_19)] = "credential_19",
    [NVS_KEY_INT(WIFI_CREDENTIAL_20)] = "credential_20",
    [NVS_KEY_INT(WIFI_CREDENTIAL_21)] = "credential_21",
    [NVS_KEY_INT(WIFI_CREDENTIAL_22)] = "credential_22",
    [NVS_KEY_INT(WIFI_CREDENTIAL_23)] = "credential_23",
    [NVS_KEY_INT(WIFI_CREDENTIAL_24)] = "credential_24",
    [NVS_KEY_INT(WIFI_CREDENTIAL_25)] = "credential_25",
    [NVS_KEY_INT(WIFI_CREDENTIAL_26)] = "credential_26",
    [NVS_KEY_INT(WIFI_CREDENTIAL_27)] = "credential_27",
    [NVS_KEY_INT(WIFI_CREDENTIAL_28)] = "credential_28",
    [NVS_KEY_INT(WIFI_CREDENTIAL_29)] = "credential_29",
    [NVS_KEY_INT(WIFI_CREDENTIAL_30)] = "credential_30",
    [NVS_KEY_INT(WIFI_CREDENTIAL_31)] = "credential_31",
    [NVS_KEY_INT(WIFI_CREDENTIAL_32)] = "credential_32",
    [NVS_KEY_INT(WIFI_CREDENTIAL_33)] = "credential_33",
    [NVS_KEY_INT(WIFI_CREDENTIAL_34)] = "credential_34",
    [NVS_KEY_INT(WIFI_CREDENTIAL_35)] = "credential_35",
    [NVS_KEY_INT(WIFI_CREDENTIAL_36)] = "credential_36",
    [NVS_KEY_INT(WIFI_CREDENTIAL_37)] = "credential_37",
    [NVS_KEY_INT(WIFI_CREDENTIAL_38)] = "credential_38",
    [NVS_KEY_INT(WIFI_CREDENTIAL_39)] = "credential_39",
    [NVS_KEY_INT(WIFI_CREDENTIAL_40)] = "credential_40",
    [NVS_KEY_INT(WIFI_CREDENTIAL_41)] = "credential_41",
    [NVS_KEY_INT(WIFI_CREDENTIAL_42)] = "credential_42",
    [NVS_KEY_INT(WIFI_CREDENTIAL_43)] = "credential_43",
    [NVS_KEY_INT(WIFI_CREDENTIAL_44)] = "credential_44",
    [NVS_KEY_INT(WIFI_CREDENTIAL_45)] = "credential_45",
    [NVS_KEY_INT(WIFI_CREDENTIAL_46)] = "credential_46",
    [NVS_KEY_INT(WIFI_CREDENTIAL_47)] = "credential_47",
    [NVS_KEY_INT(WIFI_CREDENTIAL_48)] = "credential_48",
    [NVS_KEY_INT(WIFI_CREDENTIAL_49)] = "credential_49",
    [NVS_KEY_INT(WIFI_CREDENTIAL_50)] = "credential_50",
    [NVS_KEY_INT(WIFI_CREDENTIAL_51)] = "credential_51",
    [NVS_KEY_INT(WIFI_CREDENTIAL_52)] = "credential_52",
    [NVS_KEY_INT(WIFI_CREDENTIAL_53)] = "credential_53",
    [NVS_KEY_INT(WIFI_CREDENTIAL_54)] = "credential_54",
    [NVS_KEY_INT(WIFI_CREDENTIAL_55)] = "credential_55",
    [NVS_KEY_INT(WIFI_CREDENTIAL_56)] = "credential_56",
    [NVS_KEY_INT(WIFI_CREDENTIAL_57)] = "credential_57",
    [NVS_KEY_INT(WIFI_CREDENTIAL_58)] = "credential_58",
    [NVS_KEY_INT(WIFI_CREDENTIAL_59)] = "credential_59",
    [NVS_KEY_INT(WIFI_CREDENTIAL_60)] = "credential_60",
    [NVS_KEY_INT(WIFI_CREDENTIAL_61)] = "credential_61",
    [NVS_KEY_INT(WIFI_CREDENTIAL_62)] = "credential_62",
    [NVS_KEY_INT(WIFI_CREDENTIAL_63)] = "credential_63",
    [NVS_KEY_INT(WIFI_CREDENTIAL_64)] = "credential_64",
    [NVS_KEY_INT(OTA_CONFIRM_RESULT)] = "confirm_res",
};

static uint8_t PORT_DEFAULT_PRIORITIES[5] = {5, 2, 3, 4, 1};
static uint8_t FPGA_DEFAULT_CONFIG[3] = {ADC_THRESHOLD_LOW, ADC_THRESHOLD_HIGH,
                                         ACTION_DEADZONE};
// Define the default values for each key
const std::unordered_map<NVSKey, ConfigValue> DEFAULT_CONFIG = {
    {NVSKey::SYSLOG_REPORT_STATE, {SYSLOG_REPORT_STATE_DEFAULT}},
    {NVSKey::FPGA_CONFIG, {FPGA_DEFAULT_CONFIG}},
    {NVSKey::FPGA_POWER_CONTROL, {true}},
    {NVSKey::DISPLAY_MODE, {static_cast<uint8_t>(DISPLAY_DEFAULT_MODE)}},
    {NVSKey::DISPLAY_INTENSITY,
     {static_cast<uint8_t>(DISPLAY_DEFAULT_INTENSITY)}},
    {NVSKey::DISPLAY_FLIP, {static_cast<uint8_t>(0)}},
    {NVSKey::POWER_ALLOCATOR_ENABLED, {true}},
    {NVSKey::POWER_PORT_PRIORITY,
     {static_cast<uint8_t*>(PORT_DEFAULT_PRIORITIES)}},
};

std::string nvs_key_to_string(NVSKey key) {
  int index = static_cast<int>(key);
  if (index < 0 || index >= static_cast<int>(NVSKey::NVS_KEY_COUNT)) {
    return "UNKNOWN_KEY";
  }
  return std::string(nvs_key_names[index]);
}
