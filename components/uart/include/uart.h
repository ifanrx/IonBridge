#ifndef UART_H_
#define UART_H_

#include <stdint.h>

#include "esp_check.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UART_MESSAGE_PAYLOAD_MAX 0xC0
#define UART_MESSAGE_MIN_SZ 0x0A
#define UART_MESSAGE_MAX_SZ (UART_MESSAGE_PAYLOAD_MAX + UART_MESSAGE_MIN_SZ)
#define UART_MESSAGE_VARIABLE_LENGTH 0xFF
#define UART_MESSAGE_LENGTH(uart_msg) ((uart_msg).length + UART_MESSAGE_MIN_SZ)

#define BCAST_ADDR 0x0F
#define FPGA_ADDR 0x0E
#define ESP32_ADDR 0x0D

typedef struct __attribute__((packed)) {
  uint16_t preamble;
  uint8_t addr;
  uint16_t command;
  uint8_t length;
  uint8_t payload[UART_MESSAGE_PAYLOAD_MAX];
  uint16_t checksum;
  uint16_t postamble;
} uart_message_t;

typedef enum {
  GET_PREAMBLE0,
  GET_PREAMBLE1,
  GET_ADDRESS,
  GET_COMMAND0,
  GET_COMMAND1,
  GET_LENGTH,
  GET_VALUE,
  GET_CHECKSUM0,
  GET_CHECKSUM1,
  GET_POSTAMBLE0,
  GET_POSTAMBLE1,
} uart_message_state_t;

typedef struct {
  uart_message_state_t state;
  uint8_t received_bytes;
} uart_state_t;

typedef struct {
  uint32_t reset_state_count;
  uint32_t resend_count;
  uint32_t sent_failed_count;
  uint32_t sent_count;
} uart_metrics_t;

esp_err_t uart_init();
esp_err_t uart_send(uint8_t mcu_addr, uint16_t command, const uint8_t *payload,
                    uint8_t length, int wait_delay_ms = 0,
                    bool wait_for_response = true, int send_timeout_ms = 200);
esp_err_t uart_read(uint8_t mcu_addr, uint16_t command, void *response,
                    uint8_t *response_length);

#define SEND_UART_MSG(mcu, cmd, req, req_size, resp, resp_size, format, ...) \
  do {                                                                       \
    ESP_RETURN_ON_ERROR(uart_send(mcu, cmd, req, req_size), TAG,             \
                        "uart_send to mcu %d failed: " format, mcu,          \
                        ##__VA_ARGS__);                                      \
    ESP_RETURN_ON_ERROR(uart_read(mcu, cmd, resp, resp_size), TAG,           \
                        "uart_read from mcu %d failed: " format, mcu,        \
                        ##__VA_ARGS__);                                      \
  } while (0)

#ifdef UART_FEED_TEST
void uart_feed_test();
#endif

uart_metrics_t *uart_get_metrics();

#ifdef __cplusplus
}
#endif

#endif
