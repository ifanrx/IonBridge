#ifndef CONFIG_SYNC_MANAGER_H_
#define CONFIG_SYNC_MANAGER_H_

#include <atomic>
#include <cstdint>

#include "admin.pb.h"
#include "data_types.pb.h"
#include "esp_err.h"
#include "esp_timer.h"

enum class ConfigSyncState {
  UNINITIALIZED,
  DEFAULT_RUNNING,
  LOADING,
  READY,
  PENDING_SYNC,
  SYNCING,
  APPLYING_CLOUD_CONFIG,  // Cloud config being applied to local system
  ERROR
};

// Lock-free metadata structure using atomic operations
struct ConfigSyncMetadata {
  std::atomic<uint64_t> last_sync_timestamp;
  std::atomic<uint64_t> last_update_timestamp;
  std::atomic<uint64_t> scheduled_sync_timestamp;

  ConfigSyncMetadata()
      : last_sync_timestamp(0),
        last_update_timestamp(0),
        scheduled_sync_timestamp(0) {}
};

class ConfigSyncManager {
 public:
  static ConfigSyncManager& GetInstance() {
    static ConfigSyncManager instance;
    return instance;
  }

  // Delete copy constructor and assignment operator
  ConfigSyncManager(const ConfigSyncManager&) = delete;
  ConfigSyncManager& operator=(const ConfigSyncManager&) = delete;

  // Initialize the sync manager
  esp_err_t Initialize();

  // Check if the sync manager is initialized
  bool IsInitialized() const {
    return state_.load(std::memory_order_acquire) !=
           ConfigSyncState::UNINITIALIZED;
  }

  // Stop the sync manager
  void Stop();

  // Called by components when config changes
  void NotifyConfigChange();

  // Handle SetMachineConfig request from cloud
  esp_err_t HandleSetMachineConfig(const pb_SetMachineConfigReq& request);

  // Check and trigger pending sync (called when MQTT reconnects)
  void CheckPendingSync();

  // NVS Cleanup - simple interface
  // Returns true if device has synced to cloud and can safely cleanup NVS
  bool CanCleanupNVS() const;

  // Clean up all NVS configs after cloud migration
  esp_err_t CleanupNVS(bool force = false);

 private:
  ConfigSyncManager();
  ~ConfigSyncManager();

  // Apply config from cloud
  esp_err_t ApplyCloudConfig(const pb_MachineConfig& config);

  // Build config report with specific timestamp
  esp_err_t BuildConfigReport(uint64_t sync_timestamp, pb_ConfigReport& report);

  // Send config to cloud
  esp_err_t SendToCloud(const pb_ConfigReport& report);

  // Request config from cloud
  esp_err_t RequestConfigFromCloud();

  // Schedule sync with debounce
  void ScheduleSync(uint64_t timestamp);

  // Timer callback
  static void OnSyncTimerExpired(void* arg);
  void HandleSyncTimer(uint64_t sync_timestamp);

  // Lock-free state management
  bool SetState(
      ConfigSyncState new_state);  // Returns true if transition was successful
  bool TryTransitionState(ConfigSyncState expected,
                          ConfigSyncState new_state);  // Lock-free CAS

  // Helper methods for CheckPendingSync
  void HandleReconnection();

  // Helper methods for HandleSetMachineConfig
  esp_err_t HandleConfigUpdate(const pb_MachineConfig& config);
  esp_err_t HandleConfigRequestUpload();

  // Helper methods for applying configurations
  esp_err_t ApplyPortConfig(const pb_MachinePortConfig& port_config);
  esp_err_t ApplyChargingStrategy(const pb_ChargingStrategyData& strategy_data);

  // Helper methods for building configurations
  esp_err_t BuildChargingStrategyData(pb_ChargingStrategyData& strategy_data);

  // Helper methods for sync result handling
  void HandleSyncSuccess(uint64_t sync_timestamp);
  void HandleSyncFailure(esp_err_t error);

  // Metadata (lock-free)
  ConfigSyncMetadata metadata_;

  // State (lock-free atomic)
  std::atomic<ConfigSyncState> state_;

  // Sync timer
  esp_timer_handle_t sync_timer_;
  std::atomic<bool> sync_timer_running_;

  // Constants
  static constexpr uint32_t SYNC_DEBOUNCE_MS = 2000;        // 2 seconds
  static constexpr uint32_t INITIAL_RETRY_DELAY_MS = 5000;  // 5 seconds
  static constexpr uint32_t MAX_RETRY_DELAY_MS = 300000;    // 5 minutes
  static constexpr uint32_t MAX_RETRY_COUNT = 5;

  // Error recovery state
  std::atomic<uint32_t> retry_count_;
  std::atomic<uint32_t> retry_delay_ms_;

#ifdef IONBRIDGE_CONFIG_SYNC_HOST_TEST
 public:
  // Test-only methods
  void ResetRetryState() {
    retry_count_.store(0, std::memory_order_release);
    retry_delay_ms_.store(INITIAL_RETRY_DELAY_MS, std::memory_order_release);
  }

  void ResetTimestamps() {
    metadata_.last_sync_timestamp.store(0, std::memory_order_release);
    metadata_.last_update_timestamp.store(0, std::memory_order_release);
    metadata_.scheduled_sync_timestamp.store(0, std::memory_order_release);
  }

  ConfigSyncState GetState() const {
    return state_.load(std::memory_order_acquire);
  }

  void ClearTimerState() {
    // Stop timer if running
    if (sync_timer_) {
      esp_timer_stop(sync_timer_);
    }
    sync_timer_running_.store(false, std::memory_order_release);
  }
#endif
};

#endif  // CONFIG_SYNC_MANAGER_H_
