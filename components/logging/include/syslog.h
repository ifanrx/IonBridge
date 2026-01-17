#ifndef SYSLOG_H
#define SYSLOG_H

#include <cstdarg>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/semphr.h"
#include "lwip/sockets.h"  // IWYU pragma: keep
#include "portmacro.h"
#include "singleton.h"

class SyslogClient : public Singleton<SyslogClient> {
  friend class Singleton<SyslogClient>;

 public:
  /**
   * @brief Main syslog handler - sends to remote (if enabled) + console
   */
  static int LogToSyslog(const char* format, va_list args);

  /**
   * @brief Console-only logging handler
   */
  static int LogToConsole(const char* format, va_list args);

  /**
   * @brief Enable/disable remote logging preference (saved to NVS)
   */
  esp_err_t EnableRemoteLogging(bool enable);

  /**
   * @brief Start/stop remote logging (runtime operation)
   * Called automatically based on network conditions
   */
  esp_err_t StartRemoteLogging();
  esp_err_t StopRemoteLogging();

  /**
   * @brief Register syslog as the ESP log handler
   */
  void Register();

 private:
  SyslogClient();
  ~SyslogClient();

  bool IsNetworkAvailable();
  void SendToRemote(const char* format, va_list args);
  bool InitializeNetwork();
  void CleanupNetwork();
  int ConstructMessage(char* buffer, size_t buffer_sz, const char* format,
                       va_list args) const;

  // Network resources
  int sockfd_;
  struct sockaddr_in dest_addr_struct_;

  // State
  bool available_, remote_logging_enabled_, remote_logging_active_;
  SemaphoreHandle_t mutex_;
  TickType_t semphr_timeout_;

  // ESP log integration
  vprintf_like_t original_vprintf_;
};

#endif  // SYSLOG_H
