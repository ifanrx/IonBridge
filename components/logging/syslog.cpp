#include "syslog.h"

#include <sys/types.h>
#include <sys/unistd.h>

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "cc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "logging.h"
#include "lwip/sockets.h"
#include "machine_info.h"
#include "nvs_key.h"
#include "nvs_namespace.h"
#include "portmacro.h"
#include "sdkconfig.h"
#include "utils.h"

using nvs_namespace::user_data::SYSLOG;

#define SYSLOG_SEMAPHORE_TIMEOUT_MS CONFIG_SYSLOG_SEMAPHORE_TIMEOUT_MS
#define PRI_VALUE 156
#define SYSLOG_FALLBACK_IP CONFIG_SYSLOG_HOST_FALLBACK_IP

constexpr char kSyslogHost[] = CONFIG_SYSLOG_HOST;
constexpr uint16_t kSyslogPort = CONFIG_SYSLOG_PORT;
constexpr size_t kSyslogBufferSize = CONFIG_SYSLOG_MESSAGE_BUFFER_SIZE,
                 kLogBufferSize = CONFIG_SYSLOG_COLLECTOR_BUFFER_SIZE;
static char kSyslogBuffer[kSyslogBufferSize], kLogBuffer[kLogBufferSize];

#define INTERNAL_LOG(format, ...)  \
  do {                             \
    printf("SYSLOG: ");            \
    printf(format, ##__VA_ARGS__); \
    printf("\n");                  \
  } while (0);

SyslogClient::SyslogClient()
    : sockfd_(-1),
      available_(false),
      remote_logging_enabled_(false),
      remote_logging_active_(false),
      mutex_(nullptr),
      original_vprintf_(nullptr) {
  std::memset(&dest_addr_struct_, 0, sizeof(dest_addr_struct_));
  semphr_timeout_ = pdMS_TO_TICKS(SYSLOG_SEMAPHORE_TIMEOUT_MS);

  for (int i = 0; mutex_ == nullptr && i < 3;
       i++, vTaskDelay(pdMS_TO_TICKS(100))) {
    mutex_ = xSemaphoreCreateMutex();
  }

  available_ = mutex_ != nullptr;
  if (!available_) {
    INTERNAL_LOG("Failed to create syslog mutex after 3 retries");
    return;
  }

  INTERNAL_LOG("Syslog mutex created successfully");
  // Load remote logging state from NVS
  esp_err_t err = SYSLOG.GetOrDefault(nvs_keys::SYSLOG_REPORT_STATE,
                                      remote_logging_enabled_);
  if (err != ESP_OK) {
    INTERNAL_LOG("Failed to get syslog report state from NVS, error: %d", err);
    remote_logging_enabled_ = true;  // Default to enabled
  }
}

SyslogClient::~SyslogClient() {
  EnableRemoteLogging(false);  // Cleanup network resources

  if (mutex_ != nullptr) {
    vSemaphoreDelete(mutex_);
    mutex_ = nullptr;
  }

  // Restore the original vprintf handler if it was set
  if (original_vprintf_) {
    esp_log_set_vprintf(original_vprintf_);
    original_vprintf_ = nullptr;
  }
}

esp_err_t SyslogClient::EnableRemoteLogging(bool enable) {
  if (!available_) {
    INTERNAL_LOG("Syslog client not available");
    return ESP_ERR_INVALID_STATE;
  }

  remote_logging_enabled_ = enable;
  if (!remote_logging_enabled_) {
    StopRemoteLogging();
  } else if (IsNetworkAvailable()) {
    StartRemoteLogging();
  }
  return SYSLOG.Set(nvs_keys::SYSLOG_REPORT_STATE, enable);
}

esp_err_t SyslogClient::StartRemoteLogging() {
  if (!available_) {
    INTERNAL_LOG("Syslog client not available");
    return ESP_ERR_INVALID_STATE;
  }

  if (remote_logging_active_) {
    return ESP_OK;
  }

  if (!remote_logging_enabled_) {
    return ESP_ERR_INVALID_STATE;
  }

  if (xSemaphoreTake(mutex_, semphr_timeout_) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }

  bool success = InitializeNetwork();
  if (success) {
    remote_logging_active_ = true;
    INTERNAL_LOG("Remote logging started");
  }

  xSemaphoreGive(mutex_);
  return success ? ESP_OK : ESP_FAIL;
}

esp_err_t SyslogClient::StopRemoteLogging() {
  if (!available_) {
    INTERNAL_LOG("Syslog client not available");
    return ESP_ERR_INVALID_STATE;
  }

  if (!remote_logging_active_) {
    return ESP_OK;  // Already stopped
  }

  if (xSemaphoreTake(mutex_, semphr_timeout_) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }

  CleanupNetwork();
  remote_logging_active_ = false;
  INTERNAL_LOG("Remote logging stopped");

  xSemaphoreGive(mutex_);
  return ESP_OK;
}

bool SyslogClient::InitializeNetwork() {
  if (!available_) {
    INTERNAL_LOG("Syslog client not available");
    return false;
  }

  // Resolve syslog server hostname
  in_addr_t ip_addr;
  if (!resolve_hostname(kSyslogHost, &ip_addr, true)) {
    INTERNAL_LOG("Failed to resolve syslog host: %s", kSyslogHost);
    INTERNAL_LOG("Using fallback IP: %s", SYSLOG_FALLBACK_IP);
    if (!resolve_hostname(SYSLOG_FALLBACK_IP, &ip_addr, true)) {
      return false;
    }
  }

  // Setup destination address
  std::memset(&dest_addr_struct_, 0, sizeof(dest_addr_struct_));
  dest_addr_struct_.sin_family = AF_INET;
  dest_addr_struct_.sin_port = htons(kSyslogPort);
  dest_addr_struct_.sin_addr.s_addr = ip_addr;

  // Create UDP socket
  sockfd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sockfd_ < 0) {
    perror("Failed to create UDP socket");
    return false;
  }

  return true;
}

void SyslogClient::CleanupNetwork() {
  if (sockfd_ >= 0) {
    close(sockfd_);
    sockfd_ = -1;
  }
  std::memset(&dest_addr_struct_, 0, sizeof(dest_addr_struct_));
}

void SyslogClient::Register() {
  if (!available_) {
    INTERNAL_LOG("Syslog client not available");
    return;
  }

  // Register syslog as the ESP log handler
  if (original_vprintf_ == nullptr) {
    original_vprintf_ = esp_log_set_vprintf(SyslogClient::LogToSyslog);
  }
}

int SyslogClient::ConstructMessage(char* buffer, size_t buffer_size,
                                   const char* format, va_list args) const {
  if (buffer == nullptr || buffer_size == 0 || format == nullptr ||
      strlen(format) < 3) {
    return -1;
  }

  // Start constructing the syslog message
  // Example: <PRI>1  - - SERIAL_NUMBER VERSION MODEL-HW_REV MSG
  MachineInfo& machine_info = MachineInfo::GetInstance();
  int written = snprintf(
      buffer, buffer_size, "<%d>1  - - %s v%s %s/%s ", PRI_VALUE,
      machine_info.GetPSN().c_str(),
      machine_info.FormatVersion(machine_info.GetESP32Version()).c_str(),
      machine_info.GetProductFamily().c_str(), machine_info.GetHwRev().c_str());
  if (written < 0 || static_cast<size_t>(written) >= buffer_size) {
    INTERNAL_LOG("snprintf error or buffer too small: %d %d", written,
                 (int)buffer_size);
    return -1;
  }

  size_t remaining = buffer_size - written;
  char* msg_start = buffer + written;

  // Append the formatted log message
  int msg_written = vsnprintf(msg_start, remaining, format, args);
  if (msg_written < 0 || static_cast<size_t>(msg_written) >= remaining) {
    // vsnprintf error or buffer too small
    INTERNAL_LOG("vsnprintf error or buffer too small: %d %d", msg_written,
                 (int)remaining);
    return -1;
  }

  written += msg_written;
  return written;
}

void SyslogClient::SendToRemote(const char* format, va_list args) {
  if (!(available_ && remote_logging_active_ && IsNetworkAvailable())) {
    return;
  }

  if (xSemaphoreTake(mutex_, semphr_timeout_) != pdTRUE) {
    return;
  }

  // Construct the syslog message
  int message_length =
      ConstructMessage(kSyslogBuffer, kSyslogBufferSize, format, args);

  if (message_length > 0) {
    sendto(sockfd_, kSyslogBuffer, message_length, MSG_DONTWAIT,
           reinterpret_cast<struct sockaddr*>(&dest_addr_struct_),
           sizeof(dest_addr_struct_));
  }
  xSemaphoreGive(mutex_);
}

int SyslogClient::LogToSyslog(const char* format, va_list args) {
  SyslogClient& inst = SyslogClient::GetInstance();

  if (!(inst.remote_logging_enabled_ && inst.remote_logging_active_)) {
    return inst.LogToConsole(format, args);
  }

  if (inst.IsNetworkAvailable()) {
    inst.SendToRemote(format, args);
  } else {
    INTERNAL_LOG("Network not available, logging to console instead");
  }
  return inst.LogToConsole(format, args);
}

int SyslogClient::LogToConsole(const char* format, va_list args) {
  int written = 0;
  int size = vsnprintf(kLogBuffer, kLogBufferSize, format, args);
  LogCollector::GetInstance().Push(kLogBuffer, static_cast<size_t>(size));
  SyslogClient& inst = SyslogClient::GetInstance();
  if (inst.original_vprintf_) {
    written = inst.original_vprintf_(format, args);
  } else {
    written = fwrite(kLogBuffer, 1, size, stdout);
  }
  return written;
}

bool SyslogClient::IsNetworkAvailable() {
  // Cache result to avoid frequent network checks
  static TickType_t last_check = 0;
  static bool last_result = false;

  TickType_t now = xTaskGetTickCount();
  if (now - last_check < pdMS_TO_TICKS(1000)) {  // Check every 1 seconds
    return last_result;
  }

  last_check = now;
  int test_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (test_sock < 0) {
    last_result = false;
    return false;
  }
  close(test_sock);
  last_result = true;
  return last_result;
}
