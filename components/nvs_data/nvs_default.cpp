#include "nvs_default.h"

#include <cstdint>
#include <unordered_map>

#include "nvs_key.h"
#include "sdkconfig.h"

#define DISPLAY_DEFAULT_MODE CONFIG_DISPLAY_MODE
#define DISPLAY_DEFAULT_INTENSITY CONFIG_DISPLAY_INTENSITY

#ifdef CONFIG_DEFAULT_SYSLOG_REPORTING_ENABLED
#define SYSLOG_REPORT_STATE_DEFAULT true
#else
#define SYSLOG_REPORT_STATE_DEFAULT false
#endif

#ifdef CONFIG_ENABLE_FPGA_POWER_CTRL
#define FPGA_POWER_CONTROL_DEFAULT true
#else
#define FPGA_POWER_CONTROL_DEFAULT false
#endif

using namespace nvs_wrapper;

static constexpr bool SYSLOG_REPORT_STATE_VAL = SYSLOG_REPORT_STATE_DEFAULT;
static constexpr bool FPGA_POWER_CONTROL_VAL = FPGA_POWER_CONTROL_DEFAULT;
static constexpr uint8_t DISPLAY_DEFAULT_MODE_VAL = DISPLAY_DEFAULT_MODE;
static constexpr uint8_t DISPLAY_DEFAULT_INTENSITY_VAL =
    DISPLAY_DEFAULT_INTENSITY;
static constexpr uint8_t DISPLAY_DEFAULT_FLIP_VAL = 0;
static constexpr uint8_t DISPLAY_DEFAULT_ROTATION_VAL = 0;
static constexpr uint8_t IDLE_ANIMATION_DEFAULT_VAL = 1;
static constexpr bool POWER_ALLOCATOR_ENABLED_VAL = true;
static uint8_t PORT_DEFAULT_PRIORITIES[5] = {5, 2, 3, 4, 1};

// Define the default values for each key
const std::unordered_map<Key, DefaultValue> DefaultValueStore::DEFAULT_VALUES =
    {
        {nvs_keys::SYSLOG_REPORT_STATE, DefaultValue(SYSLOG_REPORT_STATE_VAL)},
        {nvs_keys::FPGA_POWER_CONTROL, DefaultValue(FPGA_POWER_CONTROL_VAL)},
        {nvs_keys::DISPLAY_MODE, DefaultValue(DISPLAY_DEFAULT_MODE_VAL)},
        {nvs_keys::DISPLAY_INTENSITY,
         DefaultValue(DISPLAY_DEFAULT_INTENSITY_VAL)},
        {nvs_keys::DISPLAY_FLIP, DefaultValue(DISPLAY_DEFAULT_FLIP_VAL)},
        {nvs_keys::DISPLAY_ROTATION,
         DefaultValue(DISPLAY_DEFAULT_ROTATION_VAL)},
        {nvs_keys::IDLE_ANIMATION, DefaultValue(IDLE_ANIMATION_DEFAULT_VAL)},
        {nvs_keys::POWER_ALLOCATOR_ENABLED,
         DefaultValue(POWER_ALLOCATOR_ENABLED_VAL)},
        {nvs_keys::POWER_PORT_PRIORITY, DefaultValue(PORT_DEFAULT_PRIORITIES)},
};
