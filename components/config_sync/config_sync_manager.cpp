#include "config_sync_manager.h"

// Standard library includes
#include <atomic>
#include <cinttypes>
#include <cstdint>
#include <cstring>

// Protobuf includes
#include "admin.pb.h"
#include "data_types.pb.h"
#include "pb_utils.h"
#include "port.h"
#include "telemetry.pb.h"

// ESP-IDF includes
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

// Project includes
#include "data_types.h"
#include "display_manager.h"
#include "display_types.h"
#include "mqtt_app.h"
#include "nvs_key.h"
#include "nvs_namespace.h"
#include "port_manager.h"
#include "power_allocator.h"
#include "power_config.h"
#include "service.h"
#include "strategy.h"
#include "utils.h"

static const char* TAG = "ConfigSyncManager";

// ============================================================================
// Helper Functions
// ============================================================================

static const char* StateToString(ConfigSyncState state) {
  switch (state) {
    case ConfigSyncState::UNINITIALIZED:
      return "UNINITIALIZED";
    case ConfigSyncState::DEFAULT_RUNNING:
      return "DEFAULT_RUNNING";
    case ConfigSyncState::LOADING:
      return "LOADING";
    case ConfigSyncState::READY:
      return "READY";
    case ConfigSyncState::PENDING_SYNC:
      return "PENDING_SYNC";
    case ConfigSyncState::SYNCING:
      return "SYNCING";
    case ConfigSyncState::APPLYING_CLOUD_CONFIG:
      return "APPLYING_CLOUD_CONFIG";
    case ConfigSyncState::ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}

// ============================================================================
// Constructor & Destructor
// ============================================================================

ConfigSyncManager::ConfigSyncManager()
    : metadata_(),  // Initialized by struct constructor
      state_(ConfigSyncState::UNINITIALIZED),
      sync_timer_(nullptr),
      sync_timer_running_(false),
      retry_count_(0),
      retry_delay_ms_(INITIAL_RETRY_DELAY_MS) {}

ConfigSyncManager::~ConfigSyncManager() { Stop(); }

// ============================================================================
// Lifecycle Methods
// ============================================================================

esp_err_t ConfigSyncManager::Initialize() {
  // Check if already initialized
  ConfigSyncState current = state_.load(std::memory_order_acquire);
  if (current != ConfigSyncState::UNINITIALIZED) {
    ESP_LOGW(TAG, "ConfigSyncManager already initialized (state: %s)",
             StateToString(current));
    return ESP_OK;
  }

  // Create sync timer
  const esp_timer_create_args_t timer_args = {
      .callback = &ConfigSyncManager::OnSyncTimerExpired,
      .arg = this,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "config_sync_timer",
      .skip_unhandled_events = false,
  };

  esp_err_t ret = esp_timer_create(&timer_args, &sync_timer_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create sync timer: %s", esp_err_to_name(ret));
    return ret;
  }

  // Metadata already initialized by constructor
  // Set initial state
  SetState(ConfigSyncState::DEFAULT_RUNNING);

  ESP_LOGI(TAG, "ConfigSyncManager initialized");
  return ESP_OK;
}

void ConfigSyncManager::Stop() {
  // Stop timer first
  if (sync_timer_) {
    esp_timer_stop(sync_timer_);
    esp_timer_delete(sync_timer_);
    sync_timer_ = nullptr;
  }

  // Update atomic flags
  sync_timer_running_.store(false, std::memory_order_release);

  // Force state to uninitialized (bypass state transition validation for
  // cleanup)
  state_.store(ConfigSyncState::UNINITIALIZED, std::memory_order_release);
}

// ============================================================================
// MQTT Connection Handling
// ============================================================================

void ConfigSyncManager::CheckPendingSync() {
  ESP_LOGD(TAG, "CheckPendingSync started (non-blocking)");

  // Safety check: if not initialized, log and return
  if (sync_timer_ == nullptr) {
    ESP_LOGW(TAG,
             "CheckPendingSync called but ConfigSyncManager not initialized");
    return;
  }

  // Lock-free read of current state
  ConfigSyncState current_state = state_.load(std::memory_order_acquire);

  switch (current_state) {
    case ConfigSyncState::DEFAULT_RUNNING:
      // First MQTT connection - schedule config request from cloud
      ESP_LOGI(TAG, "First MQTT connection, scheduling config request");
      // Transition to LOADING state
      if (SetState(ConfigSyncState::LOADING)) {
        // Schedule with delay to ensure MQTT subscription is complete
        // This prevents race condition where config response arrives before
        // subscription
        const uint64_t FIRST_CONFIG_REQUEST_DELAY_US = 500000;  // 500ms
        esp_timer_start_once(sync_timer_, FIRST_CONFIG_REQUEST_DELAY_US);
      }
      break;

    case ConfigSyncState::LOADING:
      // MQTT reconnected while waiting for config - retry the request
      ESP_LOGI(TAG, "MQTT reconnected during LOADING, retrying config request");
      // Check if timer is already running
      if (!sync_timer_running_.load(std::memory_order_acquire)) {
        // Schedule config request retry
        const uint64_t RETRY_CONFIG_REQUEST_DELAY_US = 500000;  // 500ms
        esp_timer_start_once(sync_timer_, RETRY_CONFIG_REQUEST_DELAY_US);
      }
      break;

    case ConfigSyncState::READY:
    case ConfigSyncState::PENDING_SYNC:
      // MQTT reconnected - check for pending syncs
      HandleReconnection();
      break;

    default:
      ESP_LOGW(TAG, "CheckPendingSync called in invalid state: %s",
               StateToString(current_state));
      break;
  }

  ESP_LOGD(TAG, "CheckPendingSync completed (non-blocking)");
}

void ConfigSyncManager::HandleReconnection() {
  // Lock-free reads of metadata
  uint64_t last_update =
      metadata_.last_update_timestamp.load(std::memory_order_acquire);
  uint64_t last_sync =
      metadata_.last_sync_timestamp.load(std::memory_order_acquire);
  bool has_unsynced_changes = last_update > last_sync;

  // Check if timer is already running
  bool timer_running = sync_timer_running_.load(std::memory_order_acquire);

  if (timer_running) {
    ESP_LOGD(TAG, "MQTT reconnected, scheduling immediate sync (non-blocking)");
    // Stop any existing timer and schedule immediate execution
    if (sync_timer_) {
      esp_timer_stop(sync_timer_);
      // Schedule with 0 delay for immediate but non-blocking execution
      esp_timer_start_once(sync_timer_, 0);
    }
  } else if (has_unsynced_changes) {
    ESP_LOGI(TAG, "MQTT reconnected, found unsynced changes");
    NotifyConfigChange();
  }
}

// ============================================================================
// Configuration Change Handling
// ============================================================================

void ConfigSyncManager::NotifyConfigChange() {
  // Safety check: if not initialized, log and return
  if (sync_timer_ == nullptr) {
    ESP_LOGW(TAG,
             "NotifyConfigChange called but ConfigSyncManager not initialized");
    return;
  }

  // Skip config sync if allocator is in temporary mode
  PowerAllocator& allocator = PowerAllocator::GetInstance();
  if (allocator.Type() == PowerAllocatorType::TEMPORARY_ALLOCATOR) {
    ESP_LOGD(TAG, "Ignoring config change during temporary allocator mode");
    return;
  }

  // Check current state first before updating timestamp
  ConfigSyncState current = state_.load(std::memory_order_acquire);

  // If we're applying cloud config, ignore this notification
  // This prevents feedback loop when Port::Open()/Close() is called during
  // cloud sync
  if (current == ConfigSyncState::APPLYING_CLOUD_CONFIG) {
    ESP_LOGD(TAG, "Ignoring config change during cloud config application");
    return;
  }

  // Lock-free update of timestamp
  uint64_t update_timestamp = get_epoch_timestamp();
  metadata_.last_update_timestamp.store(update_timestamp,
                                        std::memory_order_release);

  // Only proceed if in a valid state for config changes
  switch (current) {
    case ConfigSyncState::READY:
    case ConfigSyncState::PENDING_SYNC:
      break;  // Valid states, continue

      // Invalid state for config change
      // During startup, DEFAULT_RUNNING and LOADING states are expected until
      // cloud config loads
    case ConfigSyncState::DEFAULT_RUNNING:
    case ConfigSyncState::LOADING:
      ESP_LOGD(TAG,
               "Config change during startup (state: %s) - will sync after "
               "cloud config loads",
               StateToString(current));
      return;
    case ConfigSyncState::SYNCING:
      ESP_LOGD(TAG,
               "Config change during sync - will be handled in next cycle");
      return;
    default:
      ESP_LOGW(TAG, "Ignoring config change in invalid state %s",
               StateToString(current));
      return;
  }

  // Try to acquire timer - if already running, the existing timer will handle
  // the updated timestamp
  bool expected = false;
  if (!sync_timer_running_.compare_exchange_strong(expected, true,
                                                   std::memory_order_acq_rel)) {
    // Timer already running, timestamp was updated above
    return;
  }

  // We acquired the timer flag - now we must either schedule the timer
  // or release the flag

  // Transition to PENDING_SYNC if needed
  if (current == ConfigSyncState::READY) {
    SetState(ConfigSyncState::PENDING_SYNC);
  }

  // Schedule sync with debounce
  // If this fails, we must release the timer flag
  if (sync_timer_ == nullptr) {
    ESP_LOGE(TAG, "Timer not initialized");
    sync_timer_running_.store(false, std::memory_order_release);
    return;
  }

  ScheduleSync(update_timestamp);
}

// ============================================================================
// Cloud Configuration Management
// ============================================================================

esp_err_t ConfigSyncManager::HandleSetMachineConfig(
    const pb_SetMachineConfigReq& request) {
  // Safety check: if not initialized, return error
  if (state_.load(std::memory_order_acquire) ==
      ConfigSyncState::UNINITIALIZED) {
    ESP_LOGE(
        TAG,
        "HandleSetMachineConfig called but ConfigSyncManager not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t ret = ESP_OK;

  switch (request.operation) {
    case pb_ConfigOperation_CONFIG_UPDATE:
      ret = HandleConfigUpdate(request.config);
      break;

    case pb_ConfigOperation_CONFIG_REQUEST_UPLOAD:
      ret = HandleConfigRequestUpload();
      break;

    default:
      ESP_LOGE(TAG, "Unknown config operation: %d", request.operation);
      ret = ESP_ERR_INVALID_ARG;
      break;
  }

  return ret;
}

esp_err_t ConfigSyncManager::HandleConfigUpdate(
    const pb_MachineConfig& config) {
  ESP_LOGI(TAG, "Received CONFIG_UPDATE with timestamp %" PRIu32,
           (uint32_t)config.timestamp);

  // Save current state before transitioning
  ConfigSyncState current = state_.load(std::memory_order_acquire);

  // Transition to APPLYING_CLOUD_CONFIG state during application
  // This prevents NotifyConfigChange() from triggering sync back to cloud
  if (current == ConfigSyncState::READY ||
      current == ConfigSyncState::LOADING) {
    SetState(ConfigSyncState::APPLYING_CLOUD_CONFIG);
  }

  esp_err_t ret = ApplyCloudConfig(config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to apply machine config: %s", esp_err_to_name(ret));
    // Restore previous state on error
    SetState(current);
    return ret;
  }

  ESP_LOGI(TAG, "Successfully applied machine config");

  // Transition to READY after successful application
  SetState(ConfigSyncState::READY);

  // Special logging for initial config
  if (current == ConfigSyncState::LOADING) {
    ESP_LOGI(TAG, "Initial config received, transitioning to READY");
  }

  return ESP_OK;
}

esp_err_t ConfigSyncManager::HandleConfigRequestUpload() {
  ESP_LOGI(TAG, "Received CONFIG_REQUEST_UPLOAD from cloud");

  // Try to transition to READY (from LOADING or stay in READY)
  if (!SetState(ConfigSyncState::READY)) {
    ESP_LOGW(TAG, "Cannot handle CONFIG_REQUEST_UPLOAD in state %s",
             StateToString(state_.load(std::memory_order_acquire)));
    return ESP_OK;
  }

  ESP_LOGD(TAG, "Triggering immediate config upload");
  // Don't update timestamp - we're uploading existing config

  // Try to acquire timer flag
  bool expected = false;
  if (!sync_timer_running_.compare_exchange_strong(expected, true,
                                                   std::memory_order_acq_rel)) {
    ESP_LOGW(TAG, "Sync timer already running, skipping upload request");
    return ESP_OK;
  }

  uint64_t current_time = get_epoch_timestamp();
  ScheduleSync(current_time);
  SetState(ConfigSyncState::PENDING_SYNC);

  return ESP_OK;
}

esp_err_t ConfigSyncManager::ApplyCloudConfig(const pb_MachineConfig& config) {
  // Lock-free timestamp check
  uint64_t current_sync_timestamp =
      metadata_.last_sync_timestamp.load(std::memory_order_acquire);

  // Check if this is actually a newer config
  if (config.timestamp <= current_sync_timestamp) {
    ESP_LOGD(TAG,
             "Ignoring already-synced config: cloud=%" PRIu32 " sync=%" PRIu32,
             (uint32_t)config.timestamp, (uint32_t)current_sync_timestamp);
    return ESP_OK;
  }

  ESP_LOGD(TAG, "Applying cloud config with timestamp %" PRIu32,
           (uint32_t)config.timestamp);

  // Apply port configs if present
  for (size_t i = 0; i < config.ports_count; i++) {
    esp_err_t ret = ApplyPortConfig(config.ports[i]);
    if (ret != ESP_OK && ret != ESP_ERR_NOT_FOUND) {
      ESP_LOGE(TAG, "Failed to apply port config: %s", esp_err_to_name(ret));
      return ret;
    }
  }

  // Apply charging strategy
  esp_err_t ret = ApplyChargingStrategy(config.charging);
  if (ret != ESP_OK) {
    return ret;
  }

  // Validate and apply display configuration
  if (static_cast<uint8_t>(config.display.mode) >= DisplayMode::MAX) {
    ESP_LOGE(TAG, "Invalid display mode: %d (max: %d)", config.display.mode,
             DisplayMode::MAX - 1);
    return ESP_ERR_INVALID_ARG;
  }
  if (static_cast<uint8_t>(config.display.flip) >= DisplayFlipMode::MAX) {
    ESP_LOGE(TAG, "Invalid display flip: %d (max: %d)", config.display.flip,
             DisplayFlipMode::MAX - 1);
    return ESP_ERR_INVALID_ARG;
  }
  if (config.display.rotation >= DisplayRotationMax) {
    ESP_LOGE(TAG, "Invalid display rotation: %d (max: %d)",
             config.display.rotation, DisplayRotationMax - 1);
    return ESP_ERR_INVALID_ARG;
  }
  DisplayManager& dm = DisplayManager::GetInstance();
  ESP_RETURN_ON_ERROR(dm.SetMode(config.display.mode), TAG,
                      "Failed to set display mode");
  ESP_RETURN_ON_ERROR(dm.SetIntensity(config.display.intensity), TAG,
                      "Failed to set display intensity");

  // Handle rotation: cloud config may send either flip (deprecated) or rotation
  // Prefer rotation field if set, otherwise translate deprecated flip to
  // rotation
  uint8_t rotation_degrees = static_cast<uint8_t>(config.display.rotation) * 90;
  if (rotation_degrees == 0 && config.display.flip != 0) {
    // Deprecated flip field: translate to rotation (0=normal->0°,
    // 1=flipped->180°)
    rotation_degrees = (config.display.flip == 1) ? 180 : 0;
  }

  esp_err_t rotation_err = dm.SetRotation(rotation_degrees);
  if (rotation_err != ESP_OK && rotation_err != ESP_ERR_NOT_SUPPORTED) {
    ESP_LOGE(TAG, "Failed to set display rotation");
    return rotation_err;
  }

  // Validate and apply temperature mode
  if (static_cast<uint8_t>(config.temperature_mode) >= TemperatureMode::MAX) {
    ESP_LOGE(TAG, "Invalid temperature mode: %d (max: %d)",
             config.temperature_mode, TemperatureMode::MAX - 1);
    return ESP_ERR_INVALID_ARG;
  }

  PowerAllocator& allocator = PowerAllocator::GetInstance();
  allocator.SetTemperatureMode(
      static_cast<TemperatureMode>(config.temperature_mode));

  // Update both timestamps to the cloud config timestamp
  // This ensures we don't try to sync this config back to the cloud
  metadata_.last_sync_timestamp.store(config.timestamp,
                                      std::memory_order_release);
  metadata_.last_update_timestamp.store(config.timestamp,
                                        std::memory_order_release);

  return ESP_OK;
}

esp_err_t ConfigSyncManager::ApplyPortConfig(
    const pb_MachinePortConfig& port_config) {
  uint8_t port_id = port_config.port_id;
  ESP_LOGD(TAG, "Applying config for port %d: enabled=%d", port_id,
           port_config.config.enabled);

  PortManager& pm = PortManager::GetInstance();
  Port* port = pm.GetPort(port_id);
  if (port == nullptr) {
    ESP_LOGW(TAG, "Port %d not found, skipping config", port_id);
    return ESP_ERR_NOT_FOUND;
  }

  // Check if port is DEAD first
  if (port->Dead()) {
    ESP_LOGW(TAG, "Port %d is DEAD, skipping all config operations", port_id);
    // Return success to avoid endless retry loop
    return ESP_OK;
  }

  // Port on/off state is local-only and not applied from cloud config
  // Direct remote control via ControlPorts RPC still works
  // (Lines 515-534 removed: enabled state application)

  // Apply the features
  PowerFeatures features;
  PowerFeaturesFromPB(features, port_config.config.features);

  esp_err_t ret = port->UpdatePowerFeatures(features);
  if (ret != ESP_OK) {
    // Check port state again after operation
    if (port->Dead()) {
      ESP_LOGW(TAG,
               "Port %d became DEAD during feature update, ignoring error: %s",
               port_id, esp_err_to_name(ret));
      return ESP_OK;
    }
    ESP_LOGE(TAG, "Failed to set features for port %d: %s", port_id,
             esp_err_to_name(ret));
    return ret;
  }

  // Set custom PDO voltage if specified
  if (port_config.config.custom_pdo_voltage_mv > 0) {
    ret = port->SetCustomPdoVoltage(port_config.config.custom_pdo_voltage_mv);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Failed to set custom PDO voltage for port %d: %s", port_id,
               esp_err_to_name(ret));
      // This is not critical, continue
    }
  }

  return ESP_OK;
}

esp_err_t ConfigSyncManager::ApplyChargingStrategy(
    const pb_ChargingStrategyData& strategy_data) {
  PowerAllocator& allocator = PowerAllocator::GetInstance();

  // Local-only strategies - device manages these, cloud doesn't apply
  if (strategy_data.strategy_type == StrategyType::TEMPORARY_CHARGING ||
      strategy_data.strategy_type ==
          StrategyType::ULTRA_FAST_SINGLE_PORT_CHARGING ||
      strategy_data.strategy_type == StrategyType::HIGH_PERFORMANCE_CHARGING) {
    return ESP_OK;
  }

  switch (strategy_data.strategy_type) {
    case StrategyType::LEGACY_CHARGING:
    case StrategyType::SLOW_CHARGING:
      allocator.SetStrategy<PowerSlowChargingStrategy>();
      break;

    case StrategyType::STATIC_CHARGING:
      if (strategy_data.has_identifier && strategy_data.has_static_table) {
        PowerAllocationTable table;
        std::memcpy(&table, &strategy_data.static_table,
                    sizeof(PowerAllocationTable));
        allocator.SetStrategy<PowerStaticChargingStrategy>(
            table, strategy_data.identifier);
      }
      break;

    case StrategyType::USBA_CHARGING:
      allocator.SetStrategy<PowerUSBAChargingStrategy>();
      break;

    default:
      ESP_LOGW(TAG, "Unknown strategy type: %" PRIu32,
               strategy_data.strategy_type);
      return ESP_ERR_INVALID_ARG;
  }

  return ESP_OK;
}

// ============================================================================
// Configuration Building & Sending
// ============================================================================

esp_err_t ConfigSyncManager::BuildConfigReport(uint64_t sync_timestamp,
                                               pb_ConfigReport& report) {
  report.timestamp = sync_timestamp;

  pb_MachineConfig& config = report.config;
  config.timestamp = sync_timestamp;

  // Build port configs
  PortManager& pm = PortManager::GetInstance();
  config.ports_count = 0;

  for (const Port& port : pm) {
    if (config.ports_count >= 8) {  // Max 8 ports as defined in .options
      ESP_LOGW(TAG, "Too many ports, truncating to 8");
      break;
    }

    pb_MachinePortConfig& port_config = config.ports[config.ports_count];
    port_config.port_id = port.Id();

    // Port on/off state is local-only, not synced to cloud
    // Set to true as default (ignored during config apply)
    port_config.config.enabled = true;

    // Convert PowerFeatures to protobuf format
    PowerFeatures features;
    port.GetPowerFeatures(&features);
    PowerFeaturesToPB(port_config.config.features, features);

    // Get custom PDO voltage
    port_config.config.custom_pdo_voltage_mv = port.GetCustomPdoVoltage();

    config.ports_count++;
  }

  // Build charging strategy data
  esp_err_t ret = BuildChargingStrategyData(config.charging);
  if (ret != ESP_OK) {
    return ret;
  }

  // Build temperature mode data with validation
  PowerAllocator& allocator = PowerAllocator::GetInstance();
  TemperatureMode temp_mode = allocator.GetTemperatureMode();
  if (temp_mode >= TemperatureMode::MAX) {
    ESP_LOGW(TAG, "Invalid temperature mode %d from allocator, using default",
             temp_mode);
    temp_mode = TemperatureMode::POWER_PRIORITY;
  }
  config.temperature_mode = static_cast<pb_TemperatureMode>(temp_mode);

  // Build display configuration data with validation
  DisplayManager& dm = DisplayManager::GetInstance();
  uint8_t display_mode = dm.GetMode();
  if (display_mode >= DisplayMode::MAX) {
    ESP_LOGW(TAG, "Invalid display mode %d from manager, using OFF",
             display_mode);
    display_mode = DisplayMode::OFF;
  }
  config.display.mode = static_cast<pb_DisplayMode>(display_mode);

  config.display.intensity = dm.GetIntensity();

  // Get rotation and set both rotation and flip (legacy) fields for
  // compatibility
  uint8_t degrees = dm.GetRotation();
  pb_DisplayRotation rotation_enum =
      static_cast<pb_DisplayRotation>(degrees / 90);
  if (rotation_enum >= DisplayRotationMax) {
    ESP_LOGW(TAG, "Invalid display rotation %d degrees from manager, using 0",
             degrees);
    rotation_enum = pb_DisplayRotation_ROTATION_0;
  }
  config.display.rotation = rotation_enum;

  // Derive flip from rotation for backward compatibility (flip is deprecated)
  uint8_t flip_mode = (degrees == 180) ? 1 : 0;
  config.display.flip = static_cast<pb_DisplayFlip>(flip_mode);

  return ESP_OK;
}

esp_err_t ConfigSyncManager::BuildChargingStrategyData(
    pb_ChargingStrategyData& strategy_data) {
  PowerAllocator& allocator = PowerAllocator::GetInstance();
  StrategyType strategy_type = allocator.GetStrategyType();

  // Local-only strategies - device manages these, don't sync to cloud
  if (strategy_type == StrategyType::TEMPORARY_CHARGING ||
      strategy_type == StrategyType::ULTRA_FAST_SINGLE_PORT_CHARGING ||
      strategy_type == StrategyType::HIGH_PERFORMANCE_CHARGING) {
    return ESP_OK;
  }

  strategy_data.strategy_type = static_cast<uint32_t>(strategy_type);

  // Handle STATIC_CHARGING special case
  if (strategy_type == StrategyType::STATIC_CHARGING) {
    auto static_strategy =
        allocator.GetStrategyAs<PowerStaticChargingStrategy>();
    if (static_strategy != nullptr) {
      strategy_data.has_identifier = true;
      strategy_data.identifier = static_strategy->GetIdentifier();

      strategy_data.has_static_table = true;
      const PowerAllocationTable& table = static_strategy->GetTable();
      std::memcpy(&strategy_data.static_table, &table,
                  sizeof(PowerAllocationTable));
    }
  }

  return ESP_OK;
}

esp_err_t ConfigSyncManager::SendToCloud(const pb_ConfigReport& report) {
  pb_TelemetryData telemetry;
  telemetry.which_data = pb_TelemetryData_config_report_tag;
  telemetry.data.config_report = report;

  // Send via MQTT
  MQTTClient* mqtt = MQTTClient::GetInstance();
  if (mqtt && mqtt->Connected()) {
    return mqtt->Publish(TelemetryCommand::CONFIG_REPORT, telemetry);
  } else {
    ESP_LOGW(TAG, "MQTT not connected, cannot send config report");
    return ESP_ERR_INVALID_STATE;
  }
}

esp_err_t ConfigSyncManager::RequestConfigFromCloud() {
  // Send a ConfigRequest telemetry message to request config from cloud
  pb_TelemetryData telemetry;
  telemetry.which_data = pb_TelemetryData_config_request_tag;

  // Set reason for the request
  const char* reason = "Initial boot config request";
  strncpy(telemetry.data.config_request.reason, reason,
          sizeof(telemetry.data.config_request.reason) - 1);
  telemetry.data.config_request
      .reason[sizeof(telemetry.data.config_request.reason) - 1] = '\0';

  // Send via MQTT
  MQTTClient* mqtt = MQTTClient::GetInstance();
  if (mqtt && mqtt->Connected()) {
    ESP_LOGI(TAG, "Requesting config from cloud");
    return mqtt->Publish(TelemetryCommand::CONFIG_REQUEST, telemetry);
  } else {
    ESP_LOGW(TAG, "MQTT not connected, cannot request config");
    return ESP_ERR_INVALID_STATE;
  }
}

// ============================================================================
// Sync Timer Management
// ============================================================================

void ConfigSyncManager::ScheduleSync(uint64_t timestamp) {
  if (sync_timer_ == nullptr) {
    return;
  }

  // Store the timestamp for the timer callback (lock-free)
  metadata_.scheduled_sync_timestamp.store(timestamp,
                                           std::memory_order_release);

  esp_timer_stop(sync_timer_);
  esp_timer_start_once(sync_timer_, SYNC_DEBOUNCE_MS * 1000);
}

void ConfigSyncManager::OnSyncTimerExpired(void* arg) {
  ConfigSyncManager* manager = static_cast<ConfigSyncManager*>(arg);

  // Get the timestamp that was stored when timer was scheduled (lock-free)
  uint64_t sync_timestamp = manager->metadata_.scheduled_sync_timestamp.load(
      std::memory_order_acquire);

  manager->HandleSyncTimer(sync_timestamp);
}

void ConfigSyncManager::HandleSyncTimer(uint64_t sync_timestamp) {
  // Check MQTT connection first
  MQTTClient* mqtt = MQTTClient::GetInstance();
  if (!(mqtt && mqtt->Connected())) {
    ESP_LOGD(TAG, "MQTT not connected, keeping sync pending");
    // Don't clear timer state - keep sync pending until MQTT reconnects
    // The sync will be retried when CheckPendingSync() is called after MQTT
    // reconnects
    return;
  }

  // If we're in LOADING state, it means we need to request config from cloud
  ConfigSyncState current_state = state_.load(std::memory_order_acquire);
  if (current_state == ConfigSyncState::LOADING) {
    ESP_LOGD(TAG, "Timer executing config request from cloud");
    esp_err_t ret = RequestConfigFromCloud();

    if (ret != ESP_OK) {
      // Revert to DEFAULT_RUNNING state if request failed
      SetState(ConfigSyncState::DEFAULT_RUNNING);
      ESP_LOGW(TAG, "Failed to send config request to cloud: %s",
               esp_err_to_name(ret));
    }
    // If successful, stay in LOADING state until cloud responds
    // Clear timer flag since this operation is complete
    sync_timer_running_.store(false, std::memory_order_release);
    return;
  }

  // Normal sync operation: send config to cloud
  // Transition to SYNCING state
  SetState(ConfigSyncState::SYNCING);

  // Build and send config
  pb_ConfigReport report = pb_ConfigReport_init_zero;
  esp_err_t ret = BuildConfigReport(sync_timestamp, report);
  if (ret == ESP_OK) {
    ret = SendToCloud(report);
  }

  // Handle result (lock-free)
  if (ret == ESP_OK) {
    HandleSyncSuccess(sync_timestamp);
  } else {
    HandleSyncFailure(ret);
  }
}

void ConfigSyncManager::HandleSyncSuccess(uint64_t sync_timestamp) {
  // Check if this is the first sync (migration point)
  uint64_t previous_sync =
      metadata_.last_sync_timestamp.load(std::memory_order_acquire);
  bool is_first_sync = (previous_sync == 0);

  // Update sync timestamp (lock-free)
  metadata_.last_sync_timestamp.store(sync_timestamp,
                                      std::memory_order_release);

  if (is_first_sync) {
    ESP_LOGI(TAG,
             "First successful cloud sync completed - migration ready at "
             "timestamp %" PRIu32,
             (uint32_t)sync_timestamp);
  }

  // Reset retry counters on success
  retry_count_.store(0, std::memory_order_release);
  retry_delay_ms_.store(INITIAL_RETRY_DELAY_MS, std::memory_order_release);

  // Check if more changes happened during sync (lock-free)
  uint64_t last_update =
      metadata_.last_update_timestamp.load(std::memory_order_acquire);
  bool need_resync = (last_update > sync_timestamp);

  // Mark timer as not running
  sync_timer_running_.store(false, std::memory_order_release);

  if (need_resync) {
    // More changes happened, schedule another sync
    // Try to acquire timer again
    bool expected = false;
    if (sync_timer_running_.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel)) {
      ScheduleSync(last_update);
      SetState(ConfigSyncState::PENDING_SYNC);
    }
  } else {
    SetState(ConfigSyncState::READY);
  }
}

void ConfigSyncManager::HandleSyncFailure(esp_err_t error) {
  ESP_LOGE(TAG, "Failed to sync config: %s", esp_err_to_name(error));

  // If MQTT disconnected, don't retry - wait for reconnection
  if (error == ESP_ERR_INVALID_STATE) {
    // Mark timer as not running (lock-free)
    sync_timer_running_.store(false, std::memory_order_release);
    SetState(ConfigSyncState::PENDING_SYNC);
    ESP_LOGI(TAG, "MQTT disconnected during sync, will retry on reconnect");
    return;
  }

  // Increment retry count
  uint32_t current_retry =
      retry_count_.fetch_add(1, std::memory_order_acq_rel) + 1;

  if (current_retry > MAX_RETRY_COUNT) {
    // Max retries exceeded, go to ERROR state
    ESP_LOGE(TAG,
             "Max retry count exceeded (%" PRIu32 "), entering ERROR state",
             MAX_RETRY_COUNT);
    sync_timer_running_.store(false, std::memory_order_release);
    SetState(ConfigSyncState::ERROR);
    return;
  }

  // Calculate next retry delay with exponential backoff
  uint32_t current_delay = retry_delay_ms_.load(std::memory_order_acquire);
  uint32_t next_delay = current_delay * 2;
  if (next_delay > MAX_RETRY_DELAY_MS) {
    next_delay = MAX_RETRY_DELAY_MS;
  }
  retry_delay_ms_.store(next_delay, std::memory_order_release);

  ESP_LOGI(TAG,
           "Retrying sync in %" PRIu32 " ms (attempt %" PRIu32 "/%" PRIu32 ")",
           current_delay, current_retry, MAX_RETRY_COUNT);

  // Keep state as PENDING_SYNC and schedule retry
  SetState(ConfigSyncState::PENDING_SYNC);

  // Schedule retry with backoff delay
  // Note: sync_timer_running_ is still true at this point
  if (sync_timer_) {
    esp_timer_stop(sync_timer_);
    esp_timer_start_once(sync_timer_,
                         current_delay * 1000);  // Convert to microseconds
  } else {
    // Timer not available, release flag
    sync_timer_running_.store(false, std::memory_order_release);
    SetState(ConfigSyncState::ERROR);
  }
}

// ============================================================================
// State Management
// ============================================================================

bool ConfigSyncManager::TryTransitionState(ConfigSyncState expected,
                                           ConfigSyncState new_state) {
  // Lock-free CAS operation
  if (state_.compare_exchange_strong(expected, new_state,
                                     std::memory_order_acq_rel,
                                     std::memory_order_acquire)) {
    ESP_LOGD(TAG, "State change: %s -> %s", StateToString(expected),
             StateToString(new_state));
    return true;
  }
  return false;
}

bool ConfigSyncManager::SetState(ConfigSyncState new_state) {
  // Lock-free state transition with validation
  while (true) {
    ConfigSyncState current = state_.load(std::memory_order_acquire);

    if (current == new_state) {
      return true;  // Already in desired state
    }

    // Validate state transitions
    bool valid_transition = false;

    switch (current) {
      case ConfigSyncState::UNINITIALIZED:
        valid_transition = (new_state == ConfigSyncState::DEFAULT_RUNNING);
        break;

      case ConfigSyncState::DEFAULT_RUNNING:
        valid_transition = (new_state == ConfigSyncState::LOADING ||
                            new_state == ConfigSyncState::UNINITIALIZED);
        break;

      case ConfigSyncState::LOADING:
        valid_transition =
            (new_state == ConfigSyncState::READY ||
             new_state == ConfigSyncState::APPLYING_CLOUD_CONFIG ||
             new_state == ConfigSyncState::DEFAULT_RUNNING);
        break;

      case ConfigSyncState::READY:
        valid_transition =
            (new_state == ConfigSyncState::PENDING_SYNC ||
             new_state == ConfigSyncState::SYNCING ||
             new_state == ConfigSyncState::APPLYING_CLOUD_CONFIG ||
             new_state == ConfigSyncState::UNINITIALIZED);
        break;

      case ConfigSyncState::PENDING_SYNC:
        valid_transition = (new_state == ConfigSyncState::SYNCING ||
                            new_state == ConfigSyncState::READY ||
                            new_state == ConfigSyncState::UNINITIALIZED);
        break;

      case ConfigSyncState::SYNCING:
        valid_transition = (new_state == ConfigSyncState::READY ||
                            new_state == ConfigSyncState::PENDING_SYNC ||
                            new_state == ConfigSyncState::ERROR ||
                            new_state == ConfigSyncState::UNINITIALIZED);
        break;

      case ConfigSyncState::APPLYING_CLOUD_CONFIG:
        // Can transition back to READY after applying config, or to previous
        // state on error
        valid_transition = (new_state == ConfigSyncState::READY ||
                            new_state == ConfigSyncState::LOADING ||
                            new_state == ConfigSyncState::UNINITIALIZED);
        break;

      case ConfigSyncState::ERROR:
        valid_transition = (new_state == ConfigSyncState::PENDING_SYNC ||
                            new_state == ConfigSyncState::UNINITIALIZED);
        break;
    }

    if (!valid_transition) {
      ESP_LOGE(TAG, "Invalid state transition: %s -> %s",
               StateToString(current), StateToString(new_state));
      return false;
    }

    // Try the transition
    if (TryTransitionState(current, new_state)) {
      return true;
    }
    // If CAS failed, loop and retry with fresh state
  }
}

// ============================================================================
// NVS Cleanup Implementation (KISS)
// ============================================================================

bool ConfigSyncManager::CanCleanupNVS() const {
  // Simple: Can cleanup if we've ever synced to cloud
  return metadata_.last_sync_timestamp.load(std::memory_order_acquire) > 0;
}

esp_err_t ConfigSyncManager::CleanupNVS(bool force) {
  // Simple safety check
  if (!force && !CanCleanupNVS()) {
    ESP_LOGW(TAG, "Cannot cleanup NVS - no cloud sync yet");
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGI(TAG, "Cleaning all NVS configs...");

  // Just clean everything - no selective cleanup
  // Clean port configs
  for (const auto& port_key : nvs_keys::PORT_CFG_KEYS) {
    nvs_namespace::user_data::POWER.EraseKey(port_key);
  }

  // Clean power configs (also cleans port priorities)
  cleanup_power_config_nvs();

  // Clean display configs
  nvs_namespace::user_data::DISPLAY_DATA.EraseKey(nvs_keys::DISPLAY_MODE);
  nvs_namespace::user_data::DISPLAY_DATA.EraseKey(nvs_keys::DISPLAY_INTENSITY);
  nvs_namespace::user_data::DISPLAY_DATA.EraseKey(nvs_keys::DISPLAY_ROTATION);

  // Note: No temperature mode cleanup needed - PowerAllocator stores mode in
  // memory only, cloud config reapplies it on each sync

  ESP_LOGI(TAG, "NVS cleanup complete");
  return ESP_OK;
}
