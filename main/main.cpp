#include <inttypes.h>
#include <stdint.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <set>
#include <string>

#include "animation.h"
#include "app.h"
#include "ble.h"
#include "controller.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "esp_pm.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/task.h"      // IWYU pragma: keep
#include "handler.h"
#include "ionbridge.h"
#include "machine_info.h"
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
#include "version.h"
#include "wifi.h"

#if defined(CONFIG_MCU_MODEL_SW3566) || defined(CONFIG_MCU_MODEL_FAKE_SW3566)
#include "fpga.h"
#include "rpc.h"
#endif

#ifdef CONFIG_ENABLE_RFTEST
#include "esp_phy_cert_test.h"
#endif

#include "mqtt_app.h"
#define MQTT_MAX_RECONNECT_ATTEMPTS CONFIG_MQTT_MAX_RECONNECT_ATTEMPTS
#define MQTT_APP_STABLE_CONNECTION_DURATION 10 * 60 * 1e6
#ifdef CONFIG_ENABLE_RESTART_MQTT_CLIENT
#define ENABLE_RESTART_MQTT_CLIENT
#endif

#ifdef CONFIG_ENABLE_TEMP_MONITOR
#define ENABLE_TEMP_MONITOR
#endif

#ifdef CONFIG_INTERFACE_TYPE_UART
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

static PowerAllocator *pAllocator;
static DeviceController *controller;
static AnimationController *animationController;
static MachineInfo &machine_info = MachineInfo::GetInstance();

static bool comm_interface_initialized = false;
static void comm_interface_init();
static bool chip_ready[NUM_PORTS];
static bool boot_chips();

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
  pAllocator = builder->Build();

  ESP_RETURN_VOID_ON_ERROR(pAllocator->Start(), TAG,
                           "Failed to start power allocator");

  bool enabled = true;
  PowerNVSGetOrDefault(&enabled, NVSKey::POWER_ALLOCATOR_ENABLED);
  if (!enabled) {
    ESP_LOGI(TAG, "Power allocation is disabled");
    pAllocator->DisablePowerAllocation();
  }

  TelemetryTask::Initialize(*pAllocator, *controller);
  TelemetryTask *task = TelemetryTask::GetInstance();
  if (task != nullptr) {
    task->Start();
  }

  bool normally_booted = comm_interface_initialized &&
                         pAllocator->GetPortManager().GetActivePortCount() > 0;
  controller->set_normally_booted(normally_booted);
}

void set_port_configs() {
  NVSKey keys[] = {
      NVSKey::POWER_PORT0_CONFIG, NVSKey::POWER_PORT1_CONFIG,
      NVSKey::POWER_PORT2_CONFIG,
#if CONFIG_MCU_MODEL_SW3566
      NVSKey::POWER_PORT3_CONFIG, NVSKey::POWER_PORT4_CONFIG,
#endif
  };
  PortManager &pm = PortManager::GetInstance();
  for (const Port &port : pm.GetAlivePorts()) {
    size_t i = port.Id();
    if (i >= sizeof(keys)) {
      continue;
    }

    PortConfig config = {};
    size_t size = sizeof(PortConfig);
    esp_err_t err =
        PowerNVSGet(reinterpret_cast<uint8_t *>(&config), &size, keys[i]);
    if (err == ESP_OK) {
      handle_port_config_compatibility(i, config);
      pm.SetPortConfig(port, config);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
      pm.SetPortConfig(port,
                       i == 0 ? default_port_a_config : default_port_config);
    } else {
      ESP_ERROR_COMPLAIN(err, "PowerNVSData::GetPortConfig: %d", i);
    }
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

  ESP_RETURN_FALSE_ON_ERROR(rpc::display::set_display_mode(displayMode),
                            "set_display_mode 0x%x", displayMode);

  ESP_RETURN_FALSE_ON_ERROR(
      rpc::display::set_display_intensity(displayIntensity),
      "set_display_intensity 0x%x", displayIntensity);
  uint8_t displayFlip = 0x00;
  DisplayNVSGetOrDefault(&displayFlip, NVSKey::DISPLAY_FLIP);
  ESP_RETURN_FALSE_ON_ERROR(rpc::display::set_display_flip_mode(displayFlip),
                            "set_display_flip_mode 0x%x", displayFlip);
  return true;
}
#endif

void report_firmware_versions() {
  Version current_mcu_version = Version(machine_info.GetMCUVersion()),
          current_fpga_version = Version(machine_info.GetFPGAVersion()),
          current_esp32_version = Version(machine_info.GetESP32Version()),
          current_zrlib_version = Version(machine_info.GetZRLIBVersion());
  ESP_LOGI(TAG, "App version: %s", current_esp32_version.toString().c_str());
#ifdef CONFIG_MCU_MODEL_SW3566
  ESP_LOGI(TAG, "MCU version: %s, numeric version: %d",
           current_mcu_version.toString().c_str(),
           SW3566_FIRMWARE_NUMERIC_VERSION);
  ESP_LOGI(TAG, "FPGA version: %s, numeric version: %d",
           current_fpga_version.toString().c_str(),
           FPGA_FIRMWARE_NUMERIC_VERSION);
#else
  ESP_LOGI(TAG, "MCU version: %s", current_mcu_version.toString().c_str());
  ESP_LOGI(TAG, "FPGA version: %s", current_fpga_version.toString().c_str());
#endif
  ESP_LOGI(TAG, "ZRLib version: %s", current_zrlib_version.toString().c_str());
}

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
  static bool memory_exceeded = false;
  // Log warning message with allocation details
  ESP_LOGW(TAG,
           "Heap alloc failed: caller: %s, requested: %d, caps: %" PRIu32
           ", free: %" PRIu32,
           function_name, (int)requested_size, caps, esp_get_free_heap_size());
  MQTTClient::GetInstance()->SetLowMemoryMark();
  // Only available in serial console
  heap_caps_print_heap_info(caps);
  multi_heap_info_t info;
  heap_caps_get_info(&info, caps);
  uint32_t freeHeapSize = info.total_free_bytes;
  if (freeHeapSize <= CONFIG_MEMORY_ALERT_THRESHOLD_BYTES && !memory_exceeded) {
    OutOfMemoryAlertInfo data;
    data.header = {
        .service = TelemetryServiceCommand::OUT_OF_MEMORY_ALERT,
        .message_id = TelemetryTask::GetInstance()->GetMessageId(),
    };
    data.freeHeapSize = freeHeapSize;
    data.freeInternalHeapSize = esp_get_free_internal_heap_size();
    data.minimumFreeHeapSize = info.minimum_free_bytes;
    data.thresholdFreeHeapSize = CONFIG_MEMORY_ALERT_THRESHOLD_BYTES;
    ESP_LOGW(TAG,
             "OOM: threshold: %d, free heap size: %" PRIu32
             ", free internal heap size: %" PRIu32
             ", minimum free heap size: %" PRIu32,
             CONFIG_MEMORY_ALERT_THRESHOLD_BYTES, freeHeapSize,
             data.freeInternalHeapSize, data.minimumFreeHeapSize);

    ESP_ERROR_COMPLAIN(MQTTClient::GetInstance()->Publish(data.serialize(), 1),
                       "Publish OutOfMemoryAlertInfo");
  }
  memory_exceeded = freeHeapSize <= CONFIG_MEMORY_ALERT_THRESHOLD_BYTES;
}

#ifdef CONFIG_ENABLE_RFTEST
#warning "Ensure `sdkconfig.rftest` is used before compiling with ENABLE_RFTEST"
void cert_test() {
  uint32_t mode_args[6] = {};
  esp_err_t err = TestModeNVSData::GetAndEraseTestModeAArgs(mode_args);
  if (err != ESP_OK) {
    return;
  }
  ESP_LOGW(TAG, "Start cert test");
  esp_phy_wifi_rate_t rate = static_cast<esp_phy_wifi_rate_t>(mode_args[1]);
  int8_t backoff = static_cast<int8_t>(mode_args[2]);
  ESP_LOGI(TAG,
           "SetTestModeA args: %" PRIu32 " %d %d %" PRIu32 " %" PRIu32
           " %" PRIu32,
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

#ifdef ENABLE_RESTART_MQTT_CLIENT
void restart_mqtt_task(void *arg) {
  ESP_LOGI(TAG, "Restarting MQTT client");
  MQTTClient::GetInstance()->Stop();
  MQTTClient::GetInstance()->Start();
  vTaskDelete(NULL);
}
#endif

void feed_watchdog_task(void *arg) {
  ESP_ERROR_CHECK(esp_task_wdt_add(nullptr));     // Subscribe this task to TWDT
  ESP_ERROR_CHECK(esp_task_wdt_status(nullptr));  // Check if it is subscribed
  ESP_LOGI(TAG, "TWDT is subscribed");
#ifdef ENABLE_RESTART_MQTT_CLIENT
  static int64_t restart_mqtt_at = 0;
  static size_t largest_block_size = 0;
#endif
  while (true) {
    if (esp_get_free_heap_size() < 6 * 1024) {
      // If the free heap size is less than 6KB, restart the system to free up
      ESP_LOGW(TAG, "Free heap size is less than 6KB: %" PRIu32 ", restarting",
               esp_get_free_heap_size());
      esp_restart();
    }
    esp_task_wdt_reset();  // feed watchdog
#ifdef ENABLE_RESTART_MQTT_CLIENT
    largest_block_size =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (largest_block_size <= 16 * 1024 &&
        (esp_timer_get_time() - restart_mqtt_at > 60 * 1e6)) {
      if (MQTTClient::GetInstance()->Started() &&
          !MQTTClient::GetInstance()->Connected()) {
        // If the MQTT client is started but not connected, restart it
        restart_mqtt_at = esp_timer_get_time();
        xTaskCreate(restart_mqtt_task, "restart_mqtt", 2 * 1024, NULL, 1, NULL);
      }
    }
#endif
    DELAY_MS(50);
  }
  vTaskDelete(NULL);
}

static void report_startup_status_task(void *) {
  static int run_count = 0;
  run_count++;
  if (comm_interface_initialized && pAllocator != nullptr) {
    uint8_t active_ports = pAllocator->GetPortManager().GetActivePortCount();
    if (active_ports == NUM_PORTS) {
      ESP_LOGW(TAG, "STARTUP_SUCCESS");
    } else {
#if CONFIG_MCU_MODEL_SW3566
      for (int i = 0; i < NUM_PORTS; i++) {
        if (!chip_ready[i]) {
          ESP_LOGW(TAG, "SW3566_%d_BOOT_FAILED", i);
        }
      }
      ESP_LOGW(TAG, "STARTUP_FAILED_SW3566: alive=%d", active_ports);
#endif
    }
  } else if (!comm_interface_initialized) {
#if CONFIG_MCU_MODEL_SW3566
    ESP_LOGW(TAG, "STARTUP_FAILED_FPGA");
#endif
  } else {
#ifndef CONFIG_MCU_MODEL_FAKE_SW3566
    ESP_LOGW(TAG, "STARTUP_FAILED_ALLOCATOR");
#endif
  }

  if (run_count >= startup_task_max_run) {
    ESP_ERROR_COMPLAIN(esp_timer_stop(startup_task_timer),
                       "esp_timer_stop startup_task_timer");
    ESP_ERROR_COMPLAIN(esp_timer_delete(startup_task_timer),
                       "esp_timer_delete startup_task_timer");
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
      "esp_timer_create startup_task_timer");
  report_startup_status_task(NULL);  // manually run once without waiting for 5s
  ESP_ERROR_COMPLAIN(esp_timer_start_periodic(startup_task_timer,
                                              startup_task_period_ms * 1e3),
                     "esp_timer_start_periodic startup_task_timer");
}

#if CONFIG_MCU_MODEL_FAKE_SW3566
static void update_fake_mode() {
  char model[8];
  size_t length = sizeof(model);
  DeviceNVSGet(model, &length, NVSKey::DEVICE_MODEL);
  if (strcmp(model, "fake") == 0) {
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
  animationController = &AnimationController::GetInstance();
  animationController->StartTask();
  animationController->StartAnimation(AnimationId::BOOT_APP, false);
}

void app_main() {
  static const esp_pm_config_t pm_config = {
      .max_freq_mhz = 160,
      .min_freq_mhz = 80,
  };
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
  xTaskCreate(feed_watchdog_task, "wdg", 1.5 * 1024, NULL, 17, nullptr);
#endif  // CONFIG_ESP_TASK_WDT_INIT

  heap_caps_register_failed_alloc_callback(heap_caps_alloc_failed_hook);

  // Initialize NVS partitions
  ESP_GOTO_ON_ERROR(NVSPartition::Init(), AFTER_NVS_INIT, TAG,
                    "NVSPartition::Init");
  set_mac_address();
  machine_info.LoadFromStorage();
  machine_info.PrintInfo();
  report_firmware_versions();
  controller = DeviceController::GetInstance();

AFTER_NVS_INIT:
  ESP_ERROR_COMPLAIN(Storage::Init(), "Storage::Init");

#ifdef CONFIG_ENABLE_RFTEST
  cert_test();
#endif

  ESP_ERROR_COMPLAIN(esp_pm_configure(&pm_config), "esp_pm_configure");
  ESP_ERROR_COMPLAIN(start_ble_adv_task(), "start_ble_adv_task");

#ifdef CONFIG_ENABLE_GLOBAL_LOGGING_COLLECTOR
  SyslogClient::GetInstance().Register();
#endif

  PortManager::GetInstance().StartTask();
#if CONFIG_MCU_MODEL_FAKE_SW3566
  update_fake_mode();
  ESP_LOGI(TAG, "Using fake SW3566 chip, all data are fabricated.");
  memset(chip_ready, true, sizeof(chip_ready));
  comm_interface_initialized = true;
#else
  comm_interface_init();
#endif  // CONFIG_MCU_MODEL_FAKE_SW3566

  if (!comm_interface_initialized) {
    // Rollback
    ESP_LOGW(TAG, "Failed to initialize communication interface; rolling back");
    if (controller->is_in_ota()) {
      ESP_LOGW(TAG, "In OTA mode; rolling back");
      esp_ota_mark_app_invalid_rollback_and_reboot();
    } else {
      ESP_LOGW(TAG, "Not in OTA mode; not rolling back");
    }
  } else {
    start_boot_animation();
  }

  /* Start Wi-Fi task after initializing communication interface.
   * Because we need to play animation during Wi-Fi connection.
   * Hopefully, the system would be rollback if communication interface failed
   * to initialize.
   * If not, Wi-Fi task would be running without any animation.
   */
  ESP_ERROR_COMPLAIN(wifi_initialize(), "wifi_initialize");
  WiFiController::StartTask();

  // Boot chips concurrently with Wi-Fi task. This will save some time.
  if (comm_interface_initialized) {
    // Boot chips
    if (!boot_chips()) {
      ESP_LOGE(TAG, "All chips are dead, rolling back.");
      if (controller->is_in_ota()) {
        ESP_LOGW(TAG, "In OTA mode; rolling back");
        esp_ota_mark_app_invalid_rollback_and_reboot();
      } else {
        ESP_LOGW(TAG, "Not in OTA mode; not rolling back");
      }
    } else {
      if (controller->is_in_ota()) {
        ret = controller->confirm();
        if (ret != ESP_OK) {
          ESP_LOGE(TAG, "Failed to confirm OTA, rolling back");
          controller->rollback_ota();
        }
      }
#ifdef ENABLE_POWER_ALLOCATOR
      start_allocator();
#endif  // ENABLE_POWER_ALLOCATOR
      set_port_configs();
    }
  }

  report_startup_status();

  // 初始化 App 服务
  App &app = App::GetInstance();
  app.Init(*controller, *pAllocator);
  Handler::RegisterAllServices(app);

  xTaskCreate(handle_ble_messages_task, "ble", CONFIG_BLE_TASK_STACK_SIZE,
              controller, 1, nullptr);

#ifdef ENABLE_FPGA_ADC_MONITOR
  int64_t last_time = esp_timer_get_time(), first_time = last_time, now = 0;
#endif

  uint8_t mqtt_reconnect_count = 0;
  bool mqtt_destroyed = false;
  while (true) {
    DELAY_MS(1 * 1000);

    if (controller->should_reboot()) {
      ESP_LOGW(TAG, "should_reboot requested, rebooting");
#if CONFIG_MCU_MODEL_SW3566
      for (uint8_t i = 0; i < NUM_PORTS; i++) {
        ESP_ERROR_COMPLAIN(rpc::fpga::toggle_mcu_gpio(i),
                           "Failed to toggle mcu %d gpio", i);
      }
#endif
      esp_restart();
    }

#ifdef CONFIG_ENABLE_MQTT
    MQTTClient *mqtt = MQTTClient::GetInstance();
    if (mqtt_destroyed && wifi_controller.GetState() == WiFiStateType::IDLE) {
      mqtt->Start();
      mqtt_destroyed = false;
      continue;
    }

    if (!mqtt->NeedDestroy()) {
      continue;
    }

    if (mqtt->GetConnectionDuration() > MQTT_APP_STABLE_CONNECTION_DURATION) {
      mqtt_reconnect_count = 0;
    } else if (mqtt_reconnect_count >= MQTT_MAX_RECONNECT_ATTEMPTS) {
      wifi_controller.Notify(WiFiEventType::MQTT_DISCONNECTED);
      mqtt_reconnect_count = 0;
      continue;
    }

    mqtt_reconnect_count++;
    ESP_LOGI(TAG, "MQTT reconnect attempts: %d", mqtt_reconnect_count);
    MQTTClient::DestroyInstance();
    mqtt_destroyed = true;
#endif

#ifdef ENABLE_AUTO_STREAM_DATA
    TelemetryTask::GetInstance()->SubscribeTelemetryStream();
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
    if (controller && !controller->is_upgrading()) {
      fpga_adc_monitor();
    }
#endif  // ENABLE_FPGA_ADC_MONITOR
  }

#if !CONFIG_ESP_TASK_WDT_INIT
  // If we manually initialized the TWDT, deinitialize it now
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_task_wdt_deinit());
  ESP_LOGI(TAG, "TWDT deinitialized");
#endif  // !CONFIG_ESP_TASK_WDT_INIT

  // Clean up before exit
  if (pAllocator != nullptr) {
    delete pAllocator;
  }
  if (controller != nullptr) {
    delete controller;
  }
}

void comm_interface_init() {
  comm_interface_initialized = false;
#if CONFIG_INTERFACE_TYPE_UART
  ESP_RETURN_VOID_ON_ERROR(uart_init(), TAG, "uart_init() failed");
  fpga_init();
  uint8_t value;
  ESP_RETURN_VOID_ON_ERROR(rpc::fpga::read_adc_value(&value), TAG,
                           "FPGA is not operational");
  comm_interface_initialized = configure_fpga();
#endif
}

/*
 * boot all MCU chips, return true if any chip was booted
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
      ESP_LOGI(TAG, "Chip %d GPIO toggled, will reboot shortly.", i);
    } else {
      ESP_LOGE(TAG, "Failed to toggle GPIO for chip %d", i);
    }
  }
  ESP_LOGI(TAG, "Waiting for chips to return to bootloader");
  DELAY_MS(3000);  // 3-second delay for chips to settle
  ESP_LOGI(TAG, "Booting all chips");
  rpc::mcu::boot_all(chip_ready, NUM_PORTS, SKIP_3566_UPGRADE);
#endif
  return std::any_of(chip_ready, chip_ready + NUM_PORTS,
                     [](bool b) { return b; });
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  ESP_LOGE(TAG, "Stack overflow detected in task %s", pcTaskName);

  // So that the syslog message can be reported.
  DELAY_MS(100);

  const std::set<std::string> unimportant_tasks = {
      "animation",
      "ble_adv",
  };

  if (unimportant_tasks.find(pcTaskName) != unimportant_tasks.end()) {
    ESP_LOGE(TAG, "Killing offending task %s", pcTaskName);
    vTaskDelete(xTask);
  } else {
    esp_restart();
  }
}
