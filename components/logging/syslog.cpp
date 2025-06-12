#include "syslog.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "logging.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "machine_info.h"
#include "nvs_default.h"
#include "nvs_namespace.h"
#include "sdkconfig.h"

#ifdef CONFIG_ENABLE_REPORT_SYSLOG
#define ENABLE_REPORT_SYSLOG
#define SYSLOG_SEMAPHORE_TIMEOUT_MS CONFIG_SYSLOG_SEMAPHORE_TIMEOUT_MS
#define PRI_VALUE 156
#define SYSLOG_FALLBACK_IP CONFIG_SYSLOG_HOST_FALLBACK_IP

constexpr char kSyslogHost[] = CONFIG_SYSLOG_HOST;
constexpr uint16_t kSyslogPort = CONFIG_SYSLOG_PORT;
constexpr size_t kSyslogBufferSize = 256;
static char kSyslogBuffer[kSyslogBufferSize];
constexpr size_t kLogBufferSize = 256;
static char kLogBuffer[kLogBufferSize];
#endif

#ifdef CONFIG_ENABLE_LOG_STDOUT
#define ENABLE_LOG_STDOUT
#endif

#define INTERNAL_LOG(format, ...)  \
  do {                             \
    printf("SYSLOG: ");            \
    printf(format, ##__VA_ARGS__); \
    printf("\n");                  \
  } while (0);

#ifdef ENABLE_REPORT_SYSLOG
SyslogClient::SyslogClient()
    : sockfd_(-1),
      is_initialized_(false),
      available_(false),
      syslog_mutex_(nullptr),
      report_enabled_(true),
      remote_reporting_(false) {
  std::memset(&dest_addr_struct_, 0, sizeof(dest_addr_struct_));
  semaphore_timeout_ = pdMS_TO_TICKS(SYSLOG_SEMAPHORE_TIMEOUT_MS);

  const int max_retries = 3;
  int retry_cnt = 0;
  while (retry_cnt < max_retries) {
    syslog_mutex_ = xSemaphoreCreateMutex();
    if (syslog_mutex_ != nullptr) {
      INTERNAL_LOG("Syslog mutex created successfully");
      available_ = true;
      break;
    }
    INTERNAL_LOG("Failed to create syslog mutex, retrying");
    vTaskDelay(pdMS_TO_TICKS(100));
    retry_cnt++;
  }

  if (syslog_mutex_ == nullptr) {
    INTERNAL_LOG("Failed to create syslog mutex after %d retries", max_retries);
    available_ = false;
  }
}
#else
SyslogClient::SyslogClient() {}
#endif

esp_err_t SyslogClient::Initialize() {
#ifdef ENABLE_REPORT_SYSLOG
  if (!available_) {
    INTERNAL_LOG("Syslog client not available");
    return ESP_ERR_INVALID_STATE;
  }
  if (is_initialized_) {
    return ESP_OK;
  }

  esp_err_t ret =
      SyslogNVSGetOrDefault(&report_enabled_, NVSKey::SYSLOG_REPORT_STATE);
  if (ret != ESP_OK) {
    INTERNAL_LOG("Failed to get syslog report state from NVS, error: %d", ret);
    report_enabled_ = true;
  }
#endif
  return ESP_OK;
}

bool SyslogClient::InitializeClient() {
#ifdef ENABLE_REPORT_SYSLOG
  if (!available_) {
    INTERNAL_LOG("Syslog client not available");
    return false;
  }

  if (is_initialized_) {
    INTERNAL_LOG("Syslog client already initialized");
    return true;
  }

  if (xSemaphoreTake(syslog_mutex_, semaphore_timeout_) != pdTRUE) {
    INTERNAL_LOG("Failed to take mutex within %u ms",
                 SYSLOG_SEMAPHORE_TIMEOUT_MS);
    return false;
  }

  // Resolve syslog server hostname
  if (!ResolveHost()) {
    INTERNAL_LOG("Failed to resolve Syslog server");
    xSemaphoreGive(syslog_mutex_);
    return false;
  }

  // Create UDP socket
  sockfd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sockfd_ < 0) {
    perror("Failed to create UDP socket");
    INTERNAL_LOG("Socket creation failed with error: %d", errno);
    xSemaphoreGive(syslog_mutex_);
    return false;
  }

  is_initialized_ = true;
  xSemaphoreGive(syslog_mutex_);
  return true;
#else
  return false;
#endif
}

bool SyslogClient::ResolveHost() {
#ifdef ENABLE_REPORT_SYSLOG
  struct addrinfo hints, *res, *p;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET;        // Only IPv4
  hints.ai_socktype = SOCK_DGRAM;   // Only UDP
  hints.ai_protocol = IPPROTO_UDP;  // Only UDP
  int err = getaddrinfo(kSyslogHost, nullptr, &hints, &res);
  if (err != 0) {
    INTERNAL_LOG("getaddrinfo error: %d", err);
    // use fallback IP address
    INTERNAL_LOG("Using fallback IP address: %s", SYSLOG_FALLBACK_IP);
    err = getaddrinfo(SYSLOG_FALLBACK_IP, nullptr, &hints, &res);
    if (err != 0) {
      INTERNAL_LOG("getaddrinfo error: %d", err);
      return false;
    }
  }
  if (res == nullptr) {
    INTERNAL_LOG("getaddrinfo returned no results for host: %s", kSyslogHost);
    return false;
  }

  for (p = res; p != nullptr; p = p->ai_next) {
    if (p->ai_family == AF_INET) {
      struct sockaddr_in* ipv4 =
          reinterpret_cast<struct sockaddr_in*>(p->ai_addr);
      dest_addr_struct_.sin_family = AF_INET;
      dest_addr_struct_.sin_port = htons(kSyslogPort);
      dest_addr_struct_.sin_addr.s_addr = ipv4->sin_addr.s_addr;
      freeaddrinfo(res);
      return true;
    }
  }

  freeaddrinfo(res);
#endif
  return false;
}

int SyslogClient::ConstructMessage(char* buffer, size_t buffer_size,
                                   const char* format, va_list args) const {
#ifdef ENABLE_REPORT_SYSLOG
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
                 buffer_size);
    return -1;
  }

  size_t remaining = buffer_size - written;
  char* msg_start = buffer + written;

  // Append the formatted log message
  int msg_written = vsnprintf(msg_start, remaining, format, args);
  if (msg_written < 0 || static_cast<size_t>(msg_written) >= remaining) {
    // vsnprintf error or buffer too small
    INTERNAL_LOG("vsnprintf error or buffer too small: %d %d", msg_written,
                 remaining);
    return -1;
  }

  written += msg_written;
  return written;
#else
  return 0;
#endif
}

int SyslogClient::SendMessage(const char* format, va_list args) {
#ifdef ENABLE_REPORT_SYSLOG
  if (!is_initialized_) {
    INTERNAL_LOG("Syslog client not initialized");
    return -1;
  }
  if (!report_enabled_) {
    return 0;
  }

  if (xSemaphoreTake(syslog_mutex_, semaphore_timeout_) != pdTRUE) {
    INTERNAL_LOG("Failed to take mutex within %u ms",
                 SYSLOG_SEMAPHORE_TIMEOUT_MS);
    return -1;
  }

  // Construct the syslog message
  int message_length =
      ConstructMessage(kSyslogBuffer, kSyslogBufferSize, format, args);

  if (message_length < 0) {
    INTERNAL_LOG("Failed to construct syslog message");
    xSemaphoreGive(syslog_mutex_);
    return -1;
  }

  ssize_t bytes_sent =
      sendto(sockfd_, kSyslogBuffer, message_length, 0,
             reinterpret_cast<struct sockaddr*>(&dest_addr_struct_),
             sizeof(dest_addr_struct_));
  if (bytes_sent < 0) {
    perror("Failed to send syslog message");
    INTERNAL_LOG("Sendto failed with error: %d", errno);
    xSemaphoreGive(syslog_mutex_);
    return -1;
  }

  xSemaphoreGive(syslog_mutex_);
  return static_cast<int>(bytes_sent);
#else
  return 0;
#endif
}

void SyslogClient::Register(bool remote) {
#ifdef ENABLE_REPORT_SYSLOG
  if (remote && remote_reporting_) {
    return;
  }

  if (remote && !is_initialized_) {
    if (!InitializeClient()) {
      INTERNAL_LOG("Failed to initialize syslog client");
      return;
    }
  }

  if (xSemaphoreTake(syslog_mutex_, semaphore_timeout_) != pdTRUE) {
    INTERNAL_LOG("Failed to take mutex within %u ms",
                 SYSLOG_SEMAPHORE_TIMEOUT_MS);
    return;
  }

  vprintf_like_t esp_printf_func = nullptr;
  if (remote && !remote_reporting_) {
    // INTERNAL_LOG("Registering syslog client for remote reporting");
    remote_reporting_ = true;
    esp_printf_func = esp_log_set_vprintf(SyslogClient::Report);
  } else if (!remote && remote_reporting_) {
    // INTERNAL_LOG("Unregistering syslog client for remote reporting");
    remote_reporting_ = false;
    esp_printf_func = esp_log_set_vprintf(SyslogClient::LogToStdout);
  }
  if (esp_printf_func != nullptr) {
    esp_printf_ = esp_printf_func;
  }

  xSemaphoreGive(syslog_mutex_);
#endif
}

void SyslogClient::Cleanup() {
#ifdef ENABLE_REPORT_SYSLOG
  if (!is_initialized_) {
    return;
  }

  if (xSemaphoreTake(syslog_mutex_, semaphore_timeout_) != pdTRUE) {
    INTERNAL_LOG("Failed to take mutex within %u ms",
                 SYSLOG_SEMAPHORE_TIMEOUT_MS);
    return;
  }

  if (esp_printf_) {
    esp_log_set_vprintf(esp_printf_);
  }
  if (sockfd_ != -1) {
    close(sockfd_);
    sockfd_ = -1;
    is_initialized_ = false;
  }

  xSemaphoreGive(syslog_mutex_);
#endif
}

esp_err_t SyslogClient::SetReportEnabled(bool enabled) {
#ifdef ENABLE_REPORT_SYSLOG
  if (enabled == report_enabled_) {
    return ESP_OK;
  }

  INTERNAL_LOG("Setting syslog report state to %s",
               enabled ? "enabled" : "disabled");
  report_enabled_ = enabled;
  esp_err_t ret = SyslogNVSSet(enabled, NVSKey::SYSLOG_REPORT_STATE);
  if (ret != ESP_OK) {
    INTERNAL_LOG("Failed to set syslog report state to NVS, error: %d", ret);
  }
  if (!report_enabled_) {
    Cleanup();
    return ESP_OK;
  }

  is_initialized_ = false;  // trigger re-initialization
  return Initialize();
#endif
  return ESP_OK;
}

int SyslogClient::Report(const char* format, va_list args) {
  int written = 0;

#ifdef ENABLE_REPORT_SYSLOG
  written = GetInstance().SendMessage(format, args);
#endif

  written = SyslogClient::LogToStdout(format, args);
  return written;
}

int SyslogClient::LogToStdout(const char* format, va_list args) {
  int written = 0;
#ifdef ENABLE_LOG_STDOUT
  int size = vsnprintf(kLogBuffer, kLogBufferSize, format, args);
  LogCollector::GetInstance().Push(kLogBuffer, static_cast<size_t>(size));

  written = vprintf(format, args);

#endif
  return written;
}
