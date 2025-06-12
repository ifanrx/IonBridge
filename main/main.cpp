#include <inttypes.h>
#include <stdint.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <set>
#include <string>

#include "FreeRTOSConfig.h"
#include "acdc.h"
#include "app.h"
#include "ble.h"
#include "controller.h"
#include "display_animation.h"
#include "display_manager.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_log_level.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/task.h"      // IWYU pragma: keep
#include "handler.h"
#include "ionbridge.h"
#include "machine_info.h"
#include "mdns.h"
#include "mqtt_app.h"
#include "nvs.h"
#include "nvs_default.h"
#include "nvs_namespace.h"
#include "nvs_partition.h"
#include "port.h"
#include "port_manager.h"
#include "power_allocator.h"
#include "power_config.h"
#include "sdkconfig.h"
#include "storage.h"
#include "strategy.h"
#include "syslog.h"
#include "telemetry_task.h"
#include "utils.h"
#include "wifi.h"

#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif

#ifdef CONFIG_MCU_MODEL_SW3566
#include "fpga.h"
#endif

#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
#include "rpc.h"
#endif

#if defined(CONFIG_INTERFACE_TYPE_UART) && \
    !defined(CONFIG_MCU_MODEL_FAKE_SW3566)
#include "uart.h"
#endif

#ifdef CONFIG_ENABLE_FPGA_ADC_MONITOR
#define ENABLE_FPGA_ADC_MONITOR
#endif

#ifdef CONFIG_ENABLE_AUTO_STREAM_DATA
#define ENABLE_AUTO_STREAM_DATA
#endif

#define WATCHDOG_TIMEOUT CONFIG_WATCHDOG_TIMEOUT

#ifdef CONFIG_MCU_MODEL_SW3566
#define SW3566_FIRMWARE_NUMERIC_VERSION CONFIG_SW3566_FIRMWARE_NUMERIC_VERSION
#define FPGA_FIRMWARE_NUMERIC_VERSION CONFIG_FPGA_FIRMWARE_NUMERIC_VERSION

#if defined(CONFIG_SKIP_3566_UPGRADE) || defined(CONFIG_DEVICE_HUMMING_BOARD)
#define SKIP_3566_UPGRADE true
#else
#define SKIP_3566_UPGRADE false
#endif
#endif

#ifdef CONFIG_MCU_MODEL_SW3526_SW3566
#endif

#ifdef CONFIG_ENABLE_POWER_ALLOCATOR
#define ENABLE_POWER_ALLOCATOR
#endif

using namespace std;

extern "C" void app_main(void);

static const char *TAG = "IonBridge";

void feed_watchdog_task(void *arg);

#ifndef CONFIG_ENABLE_FACTORY_MODE
static MachineInfo &machine_info = MachineInfo::GetInstance();

static bool comm_interface_initialized = false;
#if !defined(CONFIG_MCU_MODEL_FAKE_SW3566)
static void comm_interface_init();
#endif
static bool chip_ready[NUM_PORTS];
static bool boot_chips();

static void start_mdns_service();

const int startup_task_max_run = 5;
const int startup_task_period_ms = 5 * 1000;
esp_timer_handle_t startup_task_timer;

#ifdef ENABLE_FPGA_ADC_MONITOR
void fpga_adc_monitor() {
  static uint8_t last_adc_value = 0;
  uint8_t adc_value = 0;
  esp_err_t ret = rpc::fpga::read_adc_value(&adc_value);
  if (ret == ESP_OK && adc_value != last_adc_value) {
    ESP_LOGI(TAG, "FPGA ADC value: 0x%02x", adc_value);
    last_adc_value = adc_value;
    return;
  }
  ESP_ERROR_COMPLAIN(ret, "rpc::fpga::read_adc_value");
}
#endif

void start_allocator() {
  init_power_config();
  AllocatorBuilder *builder = new AllocatorBuilder();
  builder->SetPortCount(NUM_PORTS);
  builder->SetTimerPeriod(100);    // collecting power data every 100ms
  builder->SetCooldownPeriod(10);  // sufficient time before reapplying
  builder->SetApplyPeriod(1000);   // applying power strategy every 1000ms
  builder->SetPortActiveState(chip_ready, NUM_PORTS);
  builder->SetStrategy<PowerSlowChargingStrategy>();
  PowerAllocator &allocator = builder->Build();
  delete builder;

  ESP_RETURN_VOID_ON_ERROR(allocator.Start(), TAG,
                           "Failed to start power allocator");

  bool enabled = true;
  PowerNVSGetOrDefault(&enabled, NVSKey::POWER_ALLOCATOR_ENABLED);
  if (!enabled) {
    ESP_LOGI(TAG, "Power allocation is disabled");
    allocator.DisablePowerAllocation();
  }

  DeviceController &controller = DeviceController::GetInstance();
  TelemetryTask &task = TelemetryTask::GetInstance();
  task.Start();

#ifndef CONFIG_DEVICE_HUMMING_BOARD
  PortManager &pm = PortManager::GetInstance();
  bool normally_booted =
      comm_interface_initialized && pm.GetActivePortCount() > 0;
  controller.set_normally_booted(normally_booted);
#else
  controller.set_normally_booted(comm_interface_initialized);
#endif
}

void set_port_configs() {
  NVSKey keys[] = {
      NVSKey::POWER_PORT0_CONFIG, NVSKey::POWER_PORT1_CONFIG,
      NVSKey::POWER_PORT2_CONFIG,
#if CONFIG_MCU_MODEL_SW3566
      NVSKey::POWER_PORT3_CONFIG, NVSKey::POWER_PORT4_CONFIG,
#endif
  };
  size_t keys_count = sizeof(keys) / sizeof(keys[0]);
  PortManager &pm = PortManager::GetInstance();
  for (Port &port : pm.GetAlivePorts()) {
    size_t i = port.Id();
    if (i >= keys_count) {
      continue;
    }

    PortConfig config = {};
    size_t size = sizeof(PortConfig);
    esp_err_t err =
        PowerNVSGet(reinterpret_cast<uint8_t *>(&config), &size, keys[i]);
    if (err == ESP_OK) {
      port.SetConfig(config);
      continue;
    }
    if (err == ESP_ERR_NVS_NOT_FOUND) {
      port.ApplyConfig();
      continue;
    }
    ESP_ERROR_COMPLAIN(err, "PowerNVSData::GetPortConfig: %d", i);
  }
}

#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
bool configure_fpga() {
#define DISPLAY_DEFAULT_MODE CONFIG_DISPLAY_MODE
#define DISPLAY_DEFAULT_INTENSITY CONFIG_DISPLAY_INTENSITY

  uint8_t displayMode = DISPLAY_DEFAULT_MODE;
  uint8_t displayIntensity = DISPLAY_DEFAULT_INTENSITY;
  uint8_t portPriorities[NUM_PORTS] = {0};
  uint8_t adc_threshold_low, adc_threshold_high, action_deadzone;
  size_t length;

  uint8_t adc_data[3];
  length = sizeof(adc_data);
  FPGANVSGetOrDefault(adc_data, &length, NVSKey::FPGA_CONFIG);
  adc_threshold_low = adc_data[0];
  adc_threshold_high = adc_data[1];
  action_deadzone = adc_data[2];
  ESP_LOGI(TAG, "ADC threshold: low=0x%x, high=0x%x, deadzone=0x%x",
           adc_threshold_low, adc_threshold_high, action_deadzone);
  // 设置拉断的 threshold
  ESP_RETURN_FALSE_ON_ERROR(
      rpc::fpga::set_adc_threshold(adc_threshold_low, adc_threshold_high),
      "rpc::fpga::set_adc_threshold: low=0x%x, high=0x%x", adc_threshold_low,
      adc_threshold_high);

  // 设置 action deadzone
  ESP_RETURN_FALSE_ON_ERROR(rpc::fpga::set_action_deadzone(action_deadzone),
                            "rpc::fpga::set_action_deadzone: deadzone=0x%x",
                            action_deadzone);

  // 加载端口优先级
  length = NUM_PORTS;
  PowerNVSGetOrDefault(portPriorities, &length, NVSKey::POWER_PORT_PRIORITY);
  for (uint8_t i = 0; i < length; i++) {
    ESP_RETURN_FALSE_ON_ERROR(rpc::fpga::set_mcu_priority(i, portPriorities[i]),
                              "set_mcu_priority: port %d=%d", i,
                              portPriorities[i]);
  }

#ifdef CONFIG_ENABLE_FPGA_POWER_CTRL
  bool enabled = true;
  FPGANVSGetOrDefault(&enabled, NVSKey::FPGA_POWER_CONTROL);
  if (enabled) {
    // 启用端口优先级功能
    ESP_RETURN_FALSE_ON_ERROR(rpc::fpga::toggle_power_control(),
                              "toggle_power_control");
  } else {
    ESP_LOGI(TAG, "FPGA power control disabled");
  }
#endif

  DisplayManager &display_manager = DisplayManager::GetInstance();
  ESP_RETURN_FALSE_ON_ERROR(display_manager.SetDisplayMode(displayMode),
                            "SetDisplayMode 0x%x", displayMode);

  ESP_RETURN_FALSE_ON_ERROR(
      display_manager.SetDisplayIntensity(displayIntensity),
      "SetDisplayIntensity 0x%x", displayIntensity);
  uint8_t displayFlip = 0x00;
  DisplayNVSGetOrDefault(&displayFlip, NVSKey::DISPLAY_FLIP);
  ESP_RETURN_FALSE_ON_ERROR(display_manager.SetDisplayFlipMode(displayFlip),
                            "SetDisplayFlipMode 0x%x", displayFlip);
  return true;
}
#endif

esp_err_t set_mac_address() {
  // Currently we use 6-bytes MAC address
  uint8_t mac[6];
  size_t length = sizeof(mac);

  ESP_RETURN_ON_ERROR(DeviceNVSGet(mac, &length, NVSKey::DEVICE_WIFI_MAC), TAG,
                      "get_device_wifi_mac");
  mac[0] &= 0xFE;  // Ensure Unicast bit is set
  ESP_RETURN_ON_ERROR(esp_iface_mac_addr_set(mac, ESP_MAC_WIFI_STA), TAG,
                      "esp_iface_mac_addr_set ESP_MAC_WIFI_STA");

  ESP_RETURN_ON_ERROR(DeviceNVSGet(mac, &length, NVSKey::DEVICE_BLUETOOTH_MAC),
                      TAG, "get_device_bluetooth_mac");
  ESP_RETURN_ON_ERROR(esp_iface_mac_addr_set(mac, ESP_MAC_BT), TAG,
                      "esp_iface_mac_addr_set ESP_MAC_BT");

  return ESP_OK;
}

void heap_caps_alloc_failed_hook(size_t requested_size, uint32_t caps,
                                 const char *function_name) {
  // Log warning message with allocation details
  ESP_LOGW(
      TAG,
      "Heap allocation failed - caller: %s, requested: %d bytes, caps: %" PRIu32
      ", free: %" PRIu32 " bytes",
      function_name, (int)requested_size, caps, esp_get_free_heap_size());
  // Only available in serial console
  heap_caps_print_heap_info(caps);

  MQTTClient *mqtt = MQTTClient::GetInstance();
  if (!mqtt) {
    // MQTT is unavailable, return
    return;
  }
  mqtt->SetLowMemoryMark();
  TelemetryTask::GetInstance().ReportOutOfMemory(caps);
}

#ifdef CONFIG_ENABLE_RFTEST
#warning "Ensure `sdkconfig.rftest` is used before compiling with ENABLE_RFTEST"
void cert_test() {
  uint32_t mode_args[6] = {};
  esp_err_t err = TestModeNVSData::GetAndEraseTestModeAArgs(mode_args);
  if (err != ESP_OK) {
    return;
  }
  ESP_LOGW(TAG, "Starting certification test");
  esp_phy_wifi_rate_t rate = static_cast<esp_phy_wifi_rate_t>(mode_args[1]);
  int8_t backoff = static_cast<int8_t>(mode_args[2]);
  ESP_LOGI(TAG,
           "Test mode A parameters - mode: %" PRIu32
           ", rate: %d, backoff: %d, "
           "param3: %" PRIu32 ", param4: %" PRIu32 ", param5: %" PRIu32,
           mode_args[0], rate, (int)backoff, mode_args[3], mode_args[4],
           mode_args[5]);
  esp_wifi_power_domain_on();
  esp_phy_rftest_config(1);
  esp_phy_rftest_init();
  esp_phy_tx_contin_en(true);
  esp_phy_test_start_stop(3);
  esp_phy_wifi_tx(mode_args[0], rate, backoff, mode_args[3], mode_args[4],
                  mode_args[5]);
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
#endif

static void report_startup_status_task(void *) {
  static int run_count = 0;
  run_count++;
  PowerAllocator &allocator = PowerAllocator::GetInstance();
  PortManager &pm = PortManager::GetInstance();
  if (!comm_interface_initialized) {
#if CONFIG_MCU_MODEL_SW3566
    ESP_LOGW(TAG, "STARTUP_FAILED_FPGA");
#elif !defined(CONFIG_MCU_MODEL_FAKE_SW3566)
    ESP_LOGW(TAG, "STARTUP_FAILED_I2C");
#endif
  } else if (!allocator.IsConfigured()) {
#ifndef CONFIG_MCU_MODEL_FAKE_SW3566
    ESP_LOGW(TAG, "STARTUP_FAILED_ALLOCATOR");
#endif
  } else {
    uint8_t active_ports = pm.GetActivePortCount();
    if (active_ports == NUM_PORTS) {
      ESP_LOGW(TAG, "STARTUP_SUCCESS");
    } else {
#if CONFIG_MCU_MODEL_SW3566
      for (int i = 0; i < NUM_PORTS; i++) {
        if (!chip_ready[i]) {
          ESP_LOGW(TAG, "SW3566_%d_BOOT_FAILED", i);
        }
      }
      ESP_LOGW(TAG, "STARTUP_FAILED_SW3566: active_ports=%d", active_ports);
#endif
    }
  }

  if (run_count >= startup_task_max_run) {
    ESP_ERROR_COMPLAIN(esp_timer_stop(startup_task_timer),
                       "Failed to stop startup status timer");
    ESP_ERROR_COMPLAIN(esp_timer_delete(startup_task_timer),
                       "Failed to delete startup status timer");
  }
}

static void report_startup_status() {
  const esp_timer_create_args_t startup_task_timer_args = {
      .callback = report_startup_status_task,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "report_startup_status",
      .skip_unhandled_events = true,
  };
  ESP_ERROR_COMPLAIN(
      esp_timer_create(&startup_task_timer_args, &startup_task_timer),
      "Failed to create startup status timer");
  report_startup_status_task(NULL);  // Run once immediately without waiting
  ESP_ERROR_COMPLAIN(esp_timer_start_periodic(startup_task_timer,
                                              startup_task_period_ms * 1e3),
                     "Failed to start periodic startup status timer");
}

#if CONFIG_MCU_MODEL_FAKE_SW3566
static void update_fake_mode() {
  char model[8];
  size_t length = sizeof(model);
  DeviceNVSGet(model, &length, NVSKey::DEVICE_MODEL);
  if (strncmp(model, "fake", 8) == 0) {
    return;
  }
  char fake_model[] = "fake";
  ESP_LOGW(TAG, "Setting model to '%s'", fake_model);
  NVSNamespace::SSet(fake_model, NVSKey::DEVICE_MODEL, strlen(fake_model),
                     PROTECTED_DATA_NVS_PARTITION, DEVICE_DATA_NVS_NAMESPACE);
}
#endif

static void start_boot_animation() {
  ESP_LOGI(TAG, "Creating boot animation");
  DisplayManager &display_manager = DisplayManager::GetInstance();
  display_manager.SetAnimation(AnimationType::BOOT_APP, false);
}

static void handle_system_reboot() {
  ESP_LOGW(TAG, "should_reboot requested, rebooting");
  for (Port &port : PortManager::GetInstance()) {
    port.Shutdown();
  }
  esp_restart();
}

static void register_syslog_handler() {
#ifdef CONFIG_ENABLE_GLOBAL_LOGGING_COLLECTOR
  SyslogClient &syslog = SyslogClient::GetInstance();
  ESP_ERROR_COMPLAIN(syslog.Initialize(), "Failed to initialize syslog");
  syslog.Register(wifi_is_connected());
#endif
}

void app_main() {
#ifdef CONFIG_PM_ENABLE
  static const esp_pm_config_t pm_config = {
      .max_freq_mhz = 160,
      .min_freq_mhz = 80,
  };
#endif

  esp_err_t ret __attribute__((unused));
#if !CONFIG_ESP_TASK_WDT_INIT
  // If the TWDT was not initialized automatically on startup,
  // manually initialize it now
  esp_task_wdt_config_t twdt_config = {
      .timeout_ms = WATCHDOG_TIMEOUT,
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // Bitmask of all cores
      .trigger_panic = true,
  };
  ESP_ERROR_COMPLAIN(esp_task_wdt_init(&twdt_config), "esp_task_wdt_init");
  ESP_LOGI(TAG, "TWDT initialized");
  xTaskCreate(feed_watchdog_task, "wdg", 1.5 * 1024, NULL,
              CONFIG_WATCHDOG_TASK_PRIORITY, nullptr);
#endif  // CONFIG_ESP_TASK_WDT_INIT

  esp_log_level_set("wifi", ESP_LOG_WARN);
  esp_log_level_set("mqtt_client", ESP_LOG_WARN);
  esp_log_level_set("mqtt5_client", ESP_LOG_WARN);
  heap_caps_register_failed_alloc_callback(heap_caps_alloc_failed_hook);

  reset_acdc();

  // Initialize NVS partitions
  ESP_GOTO_ON_ERROR(NVSPartition::Init(), AFTER_NVS_INIT, TAG,
                    "Failed to initialize NVS partition");
  set_mac_address();
  machine_info.LoadFromStorage();
  machine_info.PrintInfo();

AFTER_NVS_INIT:
  ESP_ERROR_COMPLAIN(Storage::Init(), "Failed to initialize storage");

  DisplayManager &display_manager = DisplayManager::GetInstance();
  ESP_GOTO_ON_ERROR(display_manager.Init(), AFTER_DISPLAY_INIT, TAG,
                    "Failed to initialize display manager");
  display_manager.StartTask();

AFTER_DISPLAY_INIT:

#ifdef CONFIG_ENABLE_RFTEST
  cert_test();
#endif

#ifdef CONFIG_PM_ENABLE
  ESP_ERROR_COMPLAIN(esp_pm_configure(&pm_config),
                     "Failed to configure power management");
#endif

  ESP_ERROR_COMPLAIN(start_ble_adv_task(),
                     "Failed to start BLE advertising task");

#ifdef CONFIG_ENABLE_GLOBAL_LOGGING_COLLECTOR
  SyslogClient::GetInstance().Register();
#endif

  PortManager::GetInstance().StartTask();
#if CONFIG_MCU_MODEL_FAKE_SW3566
  update_fake_mode();
  ESP_LOGI(TAG, "Using fake SW3566 chip - all data is simulated");
  memset(chip_ready, true, sizeof(chip_ready));
  comm_interface_initialized = true;
#else
  comm_interface_init();
#endif  // CONFIG_MCU_MODEL_FAKE_SW3566

  DeviceController &controller = DeviceController::GetInstance();
  if (!comm_interface_initialized) {
    // Rollback due to communication interface initialization failure
    ESP_LOGW(
        TAG,
        "Communication interface initialization failed - performing rollback");
    controller.try_rollback();
  } else {
    start_boot_animation();
  }

  /* Start Wi-Fi task after initializing communication interface.
   * This ensures we can display animations during Wi-Fi connection.
   * If communication interface initialization fails, the system will roll back.
   * Otherwise, the Wi-Fi task will run without animation support.
   */
  if (wifi_initialize() == ESP_OK) {
    start_mdns_service();
    WiFiController::StartTask();
    if (!MQTTClient::Initialize()) {
      ESP_LOGE(TAG,
               "Failed to initialize MQTT client - remote control "
               "functionality will be unavailable");
    }
  } else {
    ESP_LOGE(TAG, "Failed to initialize Wi-Fi - system will be unavailable");
  }

  App &app = App::GetInstance();
  Handler::RegisterAllServices(app);

  // Boot chips concurrently with Wi-Fi task to save time
  if (comm_interface_initialized) {
    // Boot chips
    if (!boot_chips()) {
      ESP_LOGE(TAG, "All chips failed to boot - performing rollback");
      controller.try_rollback();
    } else {
      controller.try_confirm();
    }

    // If rollback occurs, system will reboot
#ifdef ENABLE_POWER_ALLOCATOR
    start_allocator();
#endif  // ENABLE_POWER_ALLOCATOR
    set_port_configs();
  }

  report_startup_status();

#ifdef ENABLE_FPGA_ADC_MONITOR
  int64_t last_time = esp_timer_get_time(), first_time = last_time, now = 0;
#endif

  while (true) {
    DELAY_MS(1 * 1000);

    register_syslog_handler();

    if (controller.should_reboot()) {
      handle_system_reboot();
    }

#ifdef ENABLE_AUTO_STREAM_DATA
    TelemetryTask::GetInstance().SubscribeTelemetryStream();
#endif

#ifdef ENABLE_FPGA_ADC_MONITOR
    now = esp_timer_get_time();
    if (now <= first_time + 30 * 1000 * 1000) {
      continue;
    }
    if (now <= last_time + 5 * 1000 * 1000) {
      continue;
    }
    last_time = now;
    if (!controller.is_upgrading()) {
      fpga_adc_monitor();
    }
#endif  // ENABLE_FPGA_ADC_MONITOR
  }

#if !CONFIG_ESP_TASK_WDT_INIT
  // If we manually initialized the TWDT, deinitialize it now
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_task_wdt_deinit());
  ESP_LOGI(TAG, "TWDT deinitialized");
#endif  // !CONFIG_ESP_TASK_WDT_INIT
}

#if !defined(CONFIG_MCU_MODEL_FAKE_SW3566)
void comm_interface_init() {
  comm_interface_initialized = false;
#if CONFIG_INTERFACE_TYPE_UART
  ESP_LOGI(TAG, "Initializing communication interface: UART");
  ESP_RETURN_VOID_ON_ERROR(uart_init(), TAG, "Failed to initialize UART");
  fpga_init();
  uint8_t value;
  ESP_RETURN_VOID_ON_ERROR(rpc::fpga::read_adc_value(&value), TAG,
                           "FPGA is not operational");
  comm_interface_initialized = configure_fpga();
#endif
}
#endif

/*
 * Boot all MCU chips, return true if any chip was successfully booted
 */
bool boot_chips() {
  memset(chip_ready,
#ifdef CONFIG_ALL_PORTS_ENABLED
         true,
#else
         false,
#endif
         sizeof(chip_ready));

#ifndef CONFIG_ALL_PORTS_ENABLED
#ifdef CONFIG_PORT_0_ENABLED
  chip_ready[0] = true;
#endif
#ifdef CONFIG_PORT_1_ENABLED
  chip_ready[1] = true;
#endif
#ifdef CONFIG_PORT_2_ENABLED
  chip_ready[2] = true;
#endif
#ifdef CONFIG_PORT_3_ENABLED
  chip_ready[3] = true;
#endif
#ifdef CONFIG_PORT_4_ENABLED
  chip_ready[4] = true;
#endif
#endif

#if CONFIG_MCU_MODEL_SW3566
  for (uint8_t i = 0; i < NUM_PORTS; i++) {
    esp_err_t err = rpc::fpga::toggle_mcu_gpio(i);
    chip_ready[i] = err == ESP_OK;
    if (chip_ready[i]) {
      ESP_LOGI(TAG, "Chip %d GPIO toggled - will reboot shortly", i);
    } else {
      ESP_LOGE(TAG, "Failed to toggle GPIO for chip %d", i);
    }
  }
  ESP_LOGI(TAG, "Waiting for chips to return to bootloader");
  DELAY_MS(3000);  // 3-second delay for chips to stabilize
  ESP_LOGI(TAG, "Booting all chips");
  rpc::mcu::boot_all(chip_ready, NUM_PORTS, SKIP_3566_UPGRADE);
#endif
  return std::any_of(chip_ready, chip_ready + NUM_PORTS,
                     [](bool b) { return b; });
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  ESP_LOGE(TAG, "Stack overflow detected in task: %s", pcTaskName);

  // Allow time for syslog message to be reported
  DELAY_MS(100);

  const std::set<std::string> unimportant_tasks = {
      "animation",
      "ble_adv",
  };

  if (unimportant_tasks.find(pcTaskName) != unimportant_tasks.end()) {
    ESP_LOGE(TAG, "Terminating non-critical task: %s", pcTaskName);
    vTaskDelete(xTask);
  } else {
    esp_restart();
  }
}

void start_mdns_service() {
  // Initialize mDNS service
  esp_err_t err = mdns_init();
  if (err) {
    ESP_LOGE(TAG, "mDNS initialization failed: 0x%x", err);
    return;
  }

  const std::string &hostname = MachineInfo::GetInstance().GetMDNSHostname();
  // Set hostname
  mdns_hostname_set(hostname.c_str());
  // Set default instance
  mdns_instance_name_set(hostname.c_str());
  mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
}

#else

void app_main() {
  ESP_LOGI(TAG, "Entering factory mode");
  esp_task_wdt_config_t twdt_config = {
      .timeout_ms = WATCHDOG_TIMEOUT,
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // Bitmask of all cores
      .trigger_panic = true,
  };
  esp_err_t ret = esp_task_wdt_init(&twdt_config);
  ESP_ERROR_CHECK(ret);
  ESP_LOGI(TAG, "TWDT initialized");
  xTaskCreate(feed_watchdog_task, "wdg", 1.5 * 1024, NULL,
              CONFIG_WATCHDOG_TASK_PRIORITY, nullptr);

#ifdef CONFIG_INTERFACE_TYPE_UART
  ESP_RETURN_VOID_ON_ERROR(Storage::Init(), TAG, "Storage::Init failed");
  ESP_RETURN_VOID_ON_ERROR(uart_init(), TAG, "uart_init() failed");
  fpga_init();
  uint8_t value;
  ESP_RETURN_VOID_ON_ERROR(rpc::fpga::read_adc_value(&value), TAG,
                           "FPGA is not operational");

#if CONFIG_MCU_MODEL_SW3566
  for (uint8_t i = 0; i < NUM_PORTS; i++) {
    esp_err_t err = rpc::fpga::toggle_mcu_gpio(i);
    if (err == ESP_OK) {
      ESP_LOGI(TAG, "Chip %d GPIO toggled, will reboot shortly.", i);
    } else {
      ESP_LOGE(TAG, "Failed to toggle GPIO for chip %d", i);
    }
  }
  ESP_LOGI(TAG, "Waiting for chips to return to bootloader");
  DELAY_MS(3000);  // 3-second delay for chips to settle

#endif
#endif

  while (true) {
    DELAY_MS(1000);
  }
}
#endif

void feed_watchdog_task(void *arg) {
  ESP_ERROR_CHECK(esp_task_wdt_add(nullptr));     // Subscribe this task to TWDT
  ESP_ERROR_CHECK(esp_task_wdt_status(nullptr));  // Check if it is subscribed
  ESP_LOGI(TAG, "TWDT is subscribed");
  while (true) {
    if (esp_get_free_heap_size() < 6 * 1024) {
      // If the free heap size is less than 6KB, restart the system to free up
      // memory
      ESP_LOGW(TAG,
               "Free heap size is below 6KB: %" PRIu32 " - restarting system",
               esp_get_free_heap_size());
      esp_restart();
    }
    esp_task_wdt_reset();  // Feed watchdog timer
    DELAY_MS(50);
  }
  vTaskDelete(NULL);
}
