#ifndef SYSLOG_H
#define SYSLOG_H

#include <cstdarg>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/semphr.h"
#include "lwip/sockets.h"  // IWYU pragma: keep
#include "portmacro.h"
#include "sdkconfig.h"

class SyslogClient {
 public:
  /**
   * @brief Gets the singleton instance of SyslogClient.
   *
   * @return SyslogClient& Reference to the singleton instance.
   */
  static SyslogClient& GetInstance() {
    static SyslogClient instance;
    return instance;
  }

  /**
   * @brief Reports a message to the syslog server.
   *
   * @param format Format string (printf-style).
   * @param args Variadic arguments corresponding to the format string.
   * @return int Length of the constructed message, or -1 on failure.
   */
  static int Report(const char* format, va_list args);

  /**
   * @brief Logs a message to stdout.
   *
   * @param format Format string (printf-style).
   * @param args Variadic arguments corresponding to the format string.
   * @return int Length of the constructed message, or -1 on failure.
   */
  static int LogToStdout(const char* format, va_list args);

  /**
   * @brief Initializes the SyslogClient.
   *
   * @return esp_err_t ESP_OK on success, otherwise an error code.
   */
  esp_err_t Initialize();

  /**
   * @brief Sets the report enabled flag.
   *
   * @param enabled True to enable reporting, False to disable.
   * @return esp_err_t ESP_OK on success, otherwise an error code.
   */
  esp_err_t SetReportEnabled(bool enabled);

  /**
   * @brief Sends a syslog message over UDP.
   *
   * @param format Format string (printf-style).
   * @param args Variadic arguments corresponding to the format string.
   * @return int Length of the constructed message, or -1 on failure.
   */
  int SendMessage(const char* format, va_list args);

  /**
   * @brief Register syslog to esp log system
   */
  void Register(bool remote = false);

  /**
   * @brief Cleans up the SyslogClient by closing the socket.
   *        Call this during application shutdown.
   */
  void Cleanup();

 private:
  // Private Constructor for Singleton
  SyslogClient();

  // Delete Copy Constructor and Assignment Operator
  SyslogClient(const SyslogClient&) = delete;
  SyslogClient& operator=(const SyslogClient&) = delete;

  /**
   * @brief Initializes the SyslogClient.
   *
   * @return bool True on success, False on failure.
   */
  bool InitializeClient();

  /**
   * @brief Resolves the syslog server hostname to an IPv4 address.
   *
   * @return bool True on success, False on failure.
   */
  bool ResolveHost();

  /**
   * @brief Constructs a syslog message adhering to RFC 5424.
   *
   * @param buffer Buffer to store the constructed syslog message.
   * @param buffer_size Size of the buffer.
   * @param format Format string (printf-style).
   * @param args Variadic arguments corresponding to the format string.
   * @return int Length of the constructed message, or -1 on failure.
   */
  int ConstructMessage(char* buffer, size_t buffer_size, const char* format,
                       va_list args) const;

#ifdef CONFIG_ENABLE_REPORT_SYSLOG
  // Member Variables
  int sockfd_;                           // Socket file descriptor
  struct sockaddr_in dest_addr_struct_;  // Destination address structure
  bool is_initialized_;     // Flag indicating initialization status
  bool available_;          // Flag indicating mutex availability
  char serial_number_[17];  // Serial number (16 chars + null terminator)
  SemaphoreHandle_t syslog_mutex_;  // Mutex for thread safety
  TickType_t semaphore_timeout_;    // Timeout for waiting on the semaphore
  bool report_enabled_;             // Flag indicating if reporting is enabled
  bool remote_reporting_;  // Flag indicating if remote reporting is enabled
  vprintf_like_t esp_printf_ = nullptr;  // Original esp printf function
#endif
};

#endif  // SYSLOG_H
