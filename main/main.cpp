#include <inttypes.h>
#include <stdint.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <set>
#include <string>

#include "acdc.h"
#include "ble.h"
#include "config_sync_manager.h"
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
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/task.h"      // IWYU pragma: keep
#include "handler.h"
#include "ionbridge.h"
#include "license.h"
#include "machine_info.h"
#include "mdns.h"
#include "mqtt_app.h"
#include "nvs.h"
#include "nvs_key.h"
#include "nvs_namespace.h"
#include "nvs_partition.h"
#include "port.h"
#include "port_manager.h"
#include "power_allocator.h"
#include "power_config.h"
#include "rftest.h"
#include "sdkconfig.h"
#include "storage.h"
#include "strategy.h"
#include "syslog.h"
#include "telemetry_task.h"
#include "utils.h"
#include "version.h"
#include "wdt.h"
#include "wifi.h"

#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif

#ifdef CONFIG_MCU_MODEL_SW3566
#include "esp_log_buffer.h"
#include "fpga.h"
#endif

#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
#ifdef CONFIG_ENABLE_FPGA_POWER_CTRL
#include "fpga_data_types.h"
#endif
#include "rpc.h"
#endif

#if defined(CONFIG_INTERFACE_TYPE_UART) && \
    !defined(CONFIG_MCU_MODEL_FAKE_SW3566)
#include "uart.h"
#endif

// Display driver includes
#if defined(CONFIG_DISPLAY_DRIVER_FPGA)
#include "fpga_display_driver.h"
#include "fpga_display_renderer.h"
#elif defined(CONFIG_DISPLAY_DRIVER_GPU)
#include "gpu_display_driver.h"
#include "gpu_display_renderer.h"
#elif defined(CONFIG_DISPLAY_DRIVER_LVGL)
#include "lvgl_display_driver.h"
#include "lvgl_display_renderer.h"
#else
#include "stub_display_driver.h"
#include "stub_display_renderer.h"
#endif

#ifdef CONFIG_ENABLE_FACTORY_MODE
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "factory.h"
#endif

#ifdef CONFIG_MCU_MODEL_SW3566
#if defined(CONFIG_SKIP_3566_UPGRADE) || defined(CONFIG_DEVICE_HUMMING_BOARD)
#define SKIP_3566_UPGRADE true
#else
#define SKIP_3566_UPGRADE false
#endif
#endif

using namespace std;
using namespace nvs_namespace::user_data;
using nvs_namespace::protected_data::DEVICE_DATA;

extern "C" void app_main(void);

static const char *TAG = "IonBridge";

static bool chip_ready[NUM_PORTS];

static void enter_main_loop();

static void set_log_level();
static void register_syslog_handler();
static void initialize_pm_config();

static bool initialize_comm_interface();
static void reset_chips();
static bool boot_chips();
static void cleanup_old_firmwares();
static void start_allocator();
static void set_port_configs();

static void initialize_nvs();
static void set_mac_address();

static void initialize_display();
static void start_boot_animation();

static void initialize_wifi_and_mqtt();
static void start_mdns_service();
static void check_switch_app_flag();

static void heap_caps_alloc_failed_hook(size_t requested_size, uint32_t caps,
                                        const char *function_name);

// Timer context structure
typedef struct {
  bool comm_interface_initialized;
  int run_count;
  esp_timer_handle_t timer_handle;  // Self-reference for cleanup
} startup_context_t;
const int startup_task_max_run = 5;
const int startup_task_period_ms = 5 * 1000;
static void report_startup_status(bool comm_interface_initialized);
static void report_startup_status_task(void *);

#if defined(CONFIG_MCU_MODEL_SW3566)
static bool configure_fpga();
#endif

#if CONFIG_MCU_MODEL_FAKE_SW3566
static void update_fake_mode();
#endif

void app_main() {
  init_watchdog();
  enter_main_loop();
  deinit_watchdog();  // should never be reached, but good practice
}

#ifndef CONFIG_ENABLE_FACTORY_MODE
void enter_main_loop() {
  set_log_level();
  heap_caps_register_failed_alloc_callback(heap_caps_alloc_failed_hook);

  initialize_nvs();
  ESP_ERROR_COMPLAIN(Storage::Init(), "Failed to initialize storage");
  cert_test();
  initialize_pm_config();

  ESP_ERROR_COMPLAIN(start_ble_adv_task(),
                     "Failed to start BLE advertising task");

  PortManager &pm = PortManager::GetInstance();
  pm.Start();
  bool comm_interface_initialized = initialize_comm_interface();

#ifndef CONFIG_FPGA_DUAL_FIRMWARE_WORKAROUND
  // Initialize display immediately if not using dual firmware workaround
  initialize_display();
#endif

  check_switch_app_flag();

  DeviceController &controller = DeviceController::GetInstance();
  if (!comm_interface_initialized) {
    // Rollback due to communication interface initialization failure
    ESP_LOGW(
        TAG,
        "Communication interface initialization failed - performing rollback");
    controller.try_rollback();
  } else {
    reset_acdc();
    reset_chips();
    start_boot_animation();
  }

  initialize_wifi_and_mqtt();
  register_syslog_handler();
  Handler::RegisterAllServices();

  // Boot chips concurrently with Wi-Fi task to save time
  if (comm_interface_initialized) {
    // If rollback occurs, system will reboot
    start_allocator();
    set_port_configs();

#ifdef CONFIG_ACDC_MODULE
    acdc_self_test();
#endif

#ifdef CONFIG_FPGA_DUAL_FIRMWARE_WORKAROUND
    PowerAllocator::GetInstance().DisablePowerAllocation();
    // Switch to GPU firmware variant after SW3566 chips are initialized
    fpga_init("gpu");
    // Initialize display AFTER GPU firmware is loaded
    initialize_display();
#endif
  }

  report_startup_status(comm_interface_initialized);
  LicenseController::GetInstance().CheckLicenseExpiredTime();

  while (true) {
    DELAY_MS(200);

#if defined(CONFIG_ACDC_CURRENT_VOLTAGE_CONTROL)
    auto &pm_ref = PortManager::GetInstance();
    auto req = pm_ref.GetVoltageRequirements();
    voltage_control(pm_ref.GetTotalOperatingPower(), req.max_output_voltage_mv,
                    req.has_pps_avs);
#endif

    if (controller.should_reboot()) {
      ESP_LOGW(TAG, "should_reboot requested, rebooting");
      pm.Stop();
      esp_restart();
    }

#ifdef CONFIG_ENABLE_AUTO_STREAM_DATA
    TelemetryTask::GetInstance().SubscribeTelemetryStream();
#endif
  }
}
#endif

bool initialize_comm_interface() {
#ifdef CONFIG_MCU_MODEL_FAKE_SW3566
  update_fake_mode();
  ESP_LOGI(TAG, "Using fake SW3566 chip - all data is simulated");
  return true;
#elif defined(CONFIG_INTERFACE_TYPE_UART)
  ESP_LOGI(TAG, "Initializing communication interface: UART");
  ESP_RETURN_FALSE_ON_ERROR(uart_init(), "Failed to initialize UART");
  fpga_init();
  uint8_t value;
  ESP_RETURN_FALSE_ON_ERROR(rpc::fpga::read_adc_value(&value),
                            "FPGA is not operational");
  return configure_fpga();
#else
  return false;
#endif
}

void reset_chips() {
#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
  for (uint8_t i = 0; i < NUM_PORTS; i++) {
    ESP_LOGD(TAG, "Toggling GPIO for chip %d", i);
    esp_err_t err = rpc::fpga::toggle_mcu_gpio(i);
    chip_ready[i] = err == ESP_OK;
    if (err == ESP_OK) {
      ESP_LOGI(TAG, "Chip %d GPIO toggled, will reboot shortly.", i);
    } else {
      ESP_LOGE(TAG, "Failed to toggle GPIO for chip %d", i);
    }
  }
  ESP_LOGI(TAG, "Waiting for chips to return to bootloader");
  DELAY_MS(1000);  // 1-second delay for chips to settle
#elif defined(CONFIG_INTERFACE_TYPE_I2C)
  // Not needed for I2C interface
  memset(chip_ready, true, sizeof(chip_ready));
#endif
}

/*
 * Boot all MCU chips, return true if any chip was successfully booted
 */
bool boot_chips() {
#if CONFIG_MCU_MODEL_SW3566
  ESP_LOGI(TAG, "Booting all chips");
  esp_err_t boot_err =
      rpc::mcu::boot_all(chip_ready, NUM_PORTS, SKIP_3566_UPGRADE);

  // Permanent failure - don't retry
  if (boot_err == ESP_ERR_NOT_FOUND) {
    ESP_LOGE(TAG, "Firmware file error, skipping retry");
    return false;
  }

#ifndef CONFIG_PORT_SKIP_BOOT_RETRY
  // Reset failed chips via GPIO toggle
  bool any_failed = false;
  for (uint8_t i = 0; i < NUM_PORTS; i++) {
    if (!chip_ready[i]) {
      rpc::fpga::toggle_mcu_gpio(i);
      any_failed = true;
    }
  }

  // Retry after stabilization delay
  if (any_failed) {
    DELAY_MS(3000);
    for (uint8_t i = 0; i < NUM_PORTS; i++) {
      if (chip_ready[i]) continue;
      ESP_LOGI(TAG, "Retrying chip %d", i);
      chip_ready[i] = (rpc::mcu::bringup(i, SKIP_3566_UPGRADE) == ESP_OK);
      if (!chip_ready[i]) {
        ESP_LOGE(TAG, "Chip %d failed after retry", i);
      }
    }
  }
#endif  // CONFIG_PORT_SKIP_BOOT_RETRY
#endif  // CONFIG_MCU_MODEL_SW3566
  return std::any_of(chip_ready, chip_ready + NUM_PORTS,
                     [](bool b) { return b; });
}

void cleanup_old_firmwares() {
#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
  MachineInfo &info = MachineInfo::GetInstance();
  Storage::RemoveOldVersionFile(
      Version(info.GetMCUVersion()).ToString().c_str(), "SW3566");
  Storage::RemoveOldVersionFile(
      Version(info.GetFPGAVersion()).ToString().c_str(), "FPGA");
  ESP_LOGI(TAG, "Cleaned up old firmwares");
#endif
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

void start_allocator() {
  bool enabled = false, normally_booted = true;

  DeviceController &controller = DeviceController::GetInstance();
  // Boot chips
  if (!boot_chips()) {
    ESP_LOGE(TAG, "All chips failed to boot - performing rollback");
    controller.try_rollback();
  } else {
    controller.try_confirm();
    cleanup_old_firmwares();
  }

  init_power_config();
  AllocatorBuilder *builder = new AllocatorBuilder();
  builder->SetPortCount(NUM_PORTS);
  builder->SetTimerPeriod(100);   // collecting power data every 100ms
  builder->SetApplyPeriod(1000);  // applying power strategy every 1000ms
  builder->SetPortActiveState(chip_ready, NUM_PORTS);
  builder->SetStrategy<PowerSlowChargingStrategy>();
  PowerAllocator &allocator = builder->Build();
  delete builder;

  ESP_RETURN_VOID_ON_ERROR(allocator.Start(), TAG,
                           "Failed to start power allocator");

#ifdef CONFIG_ENABLE_POWER_ALLOCATOR
  POWER.GetOrDefault(nvs_keys::POWER_ALLOCATOR_ENABLED, enabled);
#endif  // CONFIG_ENABLE_POWER_ALLOCATOR
  if (!enabled) {
    ESP_LOGI(TAG, "Power allocation is disabled");
    allocator.DisablePowerAllocation();
  }

  TelemetryTask &task = TelemetryTask::GetInstance();
  task.Start();

  // Initialize ConfigSyncManager after PowerAllocator is started
  ConfigSyncManager &sync_manager = ConfigSyncManager::GetInstance();
  ESP_ERROR_COMPLAIN(sync_manager.Initialize(),
                     "Failed to initialize ConfigSyncManager");

#ifndef CONFIG_DEVICE_HUMMING_BOARD
  PortManager &pm = PortManager::GetInstance();
  normally_booted = pm.GetActivePortCount() > 0;
#endif
  controller.set_normally_booted(normally_booted);
}

void set_port_configs() {
  PortManager &pm = PortManager::GetInstance();
  for (Port &port : pm.GetAlivePorts()) {
    size_t i = port.Id();
    if (i >= nvs_keys::PORT_CFG_SIZE) {
      continue;
    }

    PortConfig config = {};
    size_t size = sizeof(PortConfig);
    esp_err_t err = POWER.Get(nvs_keys::PORT_CFG_KEYS[i],
                              reinterpret_cast<uint8_t *>(&config), size);
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

#if defined(CONFIG_MCU_MODEL_SW3566)
bool configure_fpga() {
  // Load port priorities from NVS
  uint8_t portPriorities[NUM_PORTS] = {};
  size_t length = NUM_PORTS;
  POWER.GetOrDefault(nvs_keys::POWER_PORT_PRIORITY, portPriorities);
  ESP_LOG_BUFFER_HEXDUMP(TAG, portPriorities, length, ESP_LOG_DEBUG);
  for (uint8_t i = 0; i < length; i++) {
    ESP_RETURN_FALSE_ON_ERROR(rpc::fpga::set_mcu_priority(i, portPriorities[i]),
                              "set_mcu_priority: port %d=%d", i,
                              portPriorities[i]);
  }

#if defined(CONFIG_ENABLE_FPGA_POWER_CTRL) && !defined(CONFIG_ACDC_MODULE)
  ESP_RETURN_FALSE_ON_ERROR(FPGAADCConfig::ApplyDefaultConfig(),
                            "FPGAADCConfig::ApplyDefaultConfig");
#endif

  return true;
}
#endif

static esp_err_t get_mac_address(const nvs_namespace::Key &key, uint8_t *mac) {
  memset(mac, 0xFF, 6);
  ESP_RETURN_ON_ERROR(DEVICE_DATA.Get(key, mac, 6), TAG, "get_mac_address");

  // Detect corruption: first 3 bytes aren't 6C:80:AB, last 3 are 0xFF
  if ((mac[0] != 0x6C || mac[1] != 0x80 || mac[2] != 0xAB) && mac[3] == 0xFF &&
      mac[4] == 0xFF && mac[5] == 0xFF) {
    ESP_LOGW(TAG, "Corruption MAC address format detected, correcting");
    // Move first 3 bytes to end, fill front with vendor OUI
    memmove(&mac[3], &mac[0], 3);
    mac[0] = 0x6C;
    mac[1] = 0x80;
    mac[2] = 0xAB;
    DEVICE_DATA.Set(key, mac, 6);
  }
  return ESP_OK;
}

void set_mac_address() {
  uint8_t mac[6];

  // WiFi MAC
  ESP_RETURN_VOID_ON_ERROR(get_mac_address(nvs_keys::DEVICE_WIFI_MAC, mac), TAG,
                           "get_wifi_mac");
  mac[0] &= 0xFE;  // Ensure Unicast bit
  ESP_RETURN_VOID_ON_ERROR(esp_iface_mac_addr_set(mac, ESP_MAC_WIFI_STA), TAG,
                           "esp_iface_mac_addr_set ESP_MAC_WIFI_STA");

  // BLE MAC
  ESP_RETURN_VOID_ON_ERROR(get_mac_address(nvs_keys::DEVICE_BLUETOOTH_MAC, mac),
                           TAG, "get_ble_mac");
  ESP_RETURN_VOID_ON_ERROR(esp_iface_mac_addr_set(mac, ESP_MAC_BT), TAG,
                           "esp_iface_mac_addr_set ESP_MAC_BT");
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
  if (mqtt) {
    mqtt->SetLowMemoryMark();
    TelemetryTask::GetInstance().ReportOutOfMemory(caps);
  }
}

void report_startup_status_task(void *arg) {
  startup_context_t *ctx = (startup_context_t *)arg;
  ctx->run_count++;

  PowerAllocator &allocator = PowerAllocator::GetInstance();
  PortManager &pm = PortManager::GetInstance();
  if (!ctx->comm_interface_initialized) {
#if CONFIG_MCU_MODEL_SW3566
    ESP_LOGW(TAG, "STARTUP_FAILED_FPGA");
#endif
  } else if (!allocator.IsStarted() || !allocator.IsConfigured()) {
#ifndef CONFIG_MCU_MODEL_FAKE_SW3566
    ESP_LOGW(TAG, "STARTUP_FAILED_ALLOCATOR");
#endif
  } else {
    uint8_t active_ports = pm.GetActivePortCount();
    if (active_ports == NUM_PORTS) {
      ESP_LOGW(TAG, "STARTUP_SUCCESS");
    } else {
#if CONFIG_MCU_MODEL_SW3566
      for (const Port &port : pm) {
        if (!port.Activated()) {
          ESP_LOGW(TAG, "SW3566_%d_BOOT_FAILED", port.Id());
        }
      }
      ESP_LOGW(TAG, "STARTUP_FAILED_SW3566: active_ports=%d", active_ports);
#endif
    }
  }

  // Check if we should stop the timer
  if (ctx->run_count < startup_task_max_run) {
    return;
  }

  esp_timer_handle_t timer_to_delete = ctx->timer_handle;
  esp_err_t err = esp_timer_stop(timer_to_delete);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to stop startup status timer: %s",
             esp_err_to_name(err));
  }
  err = esp_timer_delete(timer_to_delete);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to delete startup status timer: %s",
             esp_err_to_name(err));
  }
  free(ctx);  // Free the context memory
}

void report_startup_status(bool comm_interface_initialized) {
  // Allocate context on heap to avoid dangling pointer
  startup_context_t *ctx =
      (startup_context_t *)calloc(1, sizeof(startup_context_t));
  if (!ctx) {
    ESP_LOGE(TAG, "Failed to allocate memory for startup context");
    return;
  }

  ctx->comm_interface_initialized = comm_interface_initialized;
  ctx->run_count = 0;
  const esp_timer_create_args_t startup_task_timer_args = {
      .callback = report_startup_status_task,
      .arg = ctx,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "report_startup_status",
      .skip_unhandled_events = true,
  };

  esp_err_t err =
      esp_timer_create(&startup_task_timer_args, &ctx->timer_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create startup status timer: %s",
             esp_err_to_name(err));
    free(ctx);
    return;
  }

  report_startup_status_task(ctx);  // Run once immediately without waiting

  err =
      esp_timer_start_periodic(ctx->timer_handle, startup_task_period_ms * 1e3);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start periodic startup status timer: %s",
             esp_err_to_name(err));
    esp_timer_delete(ctx->timer_handle);
    free(ctx);
    return;
  }
}

void initialize_nvs() {
  // Initialize NVS partitions
  ESP_RETURN_VOID_ON_ERROR(nvs_wrapper::Initialize(), TAG,
                           "Failed to initialize NVS partition");
  set_mac_address();

  MachineInfo &machine_info = MachineInfo::GetInstance();
  machine_info.LoadFromStorage();
  machine_info.PrintInfo();
  sync_system_time(machine_info.GetSystemTime());
}

void initialize_display() {
  // Static driver and renderer instances (compile-time selected)
  // Must be static because DisplayManager stores pointers to them
#if defined(CONFIG_DISPLAY_DRIVER_FPGA)
  static FPGADisplayDriver g_display_driver;
  static FPGARenderer g_display_renderer(&g_display_driver);
#elif defined(CONFIG_DISPLAY_DRIVER_GPU)
  static GPUDisplayDriver g_display_driver;
  static GPURenderer g_display_renderer(&g_display_driver);
#elif defined(CONFIG_DISPLAY_DRIVER_LVGL)
  static LVGLDisplayDriver g_display_driver;
  static LVGLRenderer g_display_renderer(&g_display_driver);
#else
  static StubDisplayDriver g_display_driver;
  static StubRenderer g_display_renderer(&g_display_driver);
#endif

  DisplayManager &display_manager = DisplayManager::GetInstance();

  // Check if animation resources exist (GPU only)
  bool skip_animations = false;
#if defined(CONFIG_DISPLAY_DRIVER_GPU)
  if (!Storage::Exists(CONFIG_ANIMATION_IMAGE_DIR)) {
    ESP_LOGW(TAG, "Animation resources not found, skipping animation task");
    skip_animations = true;
  }
#endif

  // Initialize driver and renderer
  ESP_RETURN_VOID_ON_ERROR(
      display_manager.Init(&g_display_driver, &g_display_renderer,
                           skip_animations),
      TAG, "Failed to initialize display");

  // Apply saved display settings from NVS
  display_manager.SetRotation(display_manager.GetRotation());
  display_manager.SetIntensity(display_manager.GetIntensity());
}

void start_boot_animation() {
  ESP_LOGI(TAG, "Creating boot animation");
  DisplayManager::GetInstance().SetNextAnimation(AnimationType::BOOT_APP,
                                                 false);
}

void check_switch_app_flag() {
  std::string switch_app_flag;
  esp_err_t err = nvs_namespace::protected_data::DEVICE_DATA.Get(
      nvs_keys::DEVICE_SWITCH_APP, switch_app_flag);

  if (err == ESP_ERR_NVS_NOT_FOUND) {
    return;
  }

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get device switch app flag from NVS: %s",
             esp_err_to_name(err));
    return;
  }

  if (!switch_app_flag.empty()) {
    ESP_LOGI(TAG, "Device switch app flag is set, switching to production app");
    err = nvs_namespace::protected_data::DEVICE_DATA.EraseKey(
        nvs_keys::DEVICE_SWITCH_APP);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to erase switch app flag in NVS: %s",
               esp_err_to_name(err));
      return;
    }

    DeviceController &controller = DeviceController::GetInstance();
    controller.try_confirm();
  }
}

#if CONFIG_MCU_MODEL_FAKE_SW3566
void update_fake_mode() {
  char model[8];
  DEVICE_DATA.Get(nvs_keys::DEVICE_MODEL, model);
  if (strncmp(model, "fake", 8) == 0) {
    return;
  }
  char fake_model[] = "fake";
  ESP_LOGW(TAG, "Setting model to '%s'", fake_model);
  DEVICE_DATA.Set(nvs_keys::DEVICE_MODEL, fake_model, strlen(fake_model));
}
#endif

void register_syslog_handler() {
  SyslogClient &syslog = SyslogClient::GetInstance();
  syslog.Register();
}

void set_log_level() {
  esp_log_level_set("wifi", ESP_LOG_WARN);
  esp_log_level_set("mqtt_client", ESP_LOG_WARN);
  esp_log_level_set("mqtt5_client", ESP_LOG_WARN);
#if defined(CONFIG_DISPLAY_DRIVER_LVGL)
  esp_log_level_set("bsp_generic", ESP_LOG_WARN);
#endif
}

void initialize_wifi_and_mqtt() {
  /* Start Wi-Fi task after initializing communication interface.
   * This ensures we can display animations during Wi-Fi connection.
   * If communication interface initialization fails, the system will roll back.
   * Otherwise, the Wi-Fi task will run without animation support.
   */
  ESP_RETURN_VOID_ON_ERROR(
      wifi_initialize(), TAG,
      "Failed to initialize Wi-Fi - system will be unavailable");
  start_mdns_service();
  WiFiController::StartTask();
  if (!MQTTClient::Initialize()) {
    ESP_LOGE(TAG,
             "Failed to initialize MQTT client - remote control "
             "functionality will be unavailable");
  }
}

void initialize_pm_config() {
#ifdef CONFIG_PM_ENABLE
  static const esp_pm_config_t pm_config = {
      .max_freq_mhz = 160,
      .min_freq_mhz = 80,
  };

  ESP_ERROR_COMPLAIN(esp_pm_configure(&pm_config),
                     "Failed to configure power management");
#endif
}
