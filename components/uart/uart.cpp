#include "driver/uart.h"

#include <endian.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/param.h>

#include <algorithm>

#include "driver/uart.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/portmacro.h"
#include "freertos/projdefs.h"
#include "hal/uart_types.h"
#include "ionbridge.h"
#include "port_manager.h"
#include "sdkconfig.h"
#include "soc/clk_tree_defs.h"
#include "task_map.h"
#include "uart.h"
#include "utils.h"

#ifdef CONFIG_FPGA_AS_UART_SWITCH
#define FPGA_AS_UART_SWITCH
#define UART_BAUD_RATE 921600
#else
#define UART_BAUD_RATE (CONFIG_UART_BAUD_RATE)
#endif

#define UART_PORT_NUM (uart_port_t)(CONFIG_UART_PORT_NUM)
#define UART_RXD (CONFIG_UART_RXD)
#define UART_TXD (CONFIG_UART_TXD)
#define UART_QUEUE_SIZE (CONFIG_UART_RX_QUEUE_SIZE)
#define UART_ENQUEUE_TIMEOUT \
  pdMS_TO_TICKS(CONFIG_UART_RX_QUEUE_ENQUEUE_TIMEOUT_MS)
#define UART_TX_TARGETS (CONFIG_UART_TX_TARGETS)
#define UART_EVENT_TASK_PRIORITY (CONFIG_UART_EVENT_TASK_PRIORITY)
#define UART_NOTIFY_INDEX (1)

#ifdef CONFIG_UART_TX_BUFFER_ENABLE
#define ENABLE_UART_TX_BUFFER
#endif
#ifdef CONFIG_UART_COMM_DEBUG
#define UART_COMM_DEBUG
#endif
#ifdef CONFIG_UART_COMM_DEBUG_DATA
#define UART_COMM_DEBUG_DATA
#endif
#ifdef CONFIG_UART_STATE_DEBUG
#define UART_STATE_DEBUG
#endif

#define HIGH_BYTE_U16(value) ((uint8_t)((value) >> 8))
#define LOW_BYTE_U16(value) ((uint8_t)((value) & 0xFF))
#define UART_MESSAGE_PREAMBLE 0x5A5A
#define UART_MESSAGE_PREAMBLE_H HIGH_BYTE_U16(UART_MESSAGE_PREAMBLE)
#define UART_MESSAGE_PREAMBLE_L LOW_BYTE_U16(UART_MESSAGE_PREAMBLE)
#define UART_MESSAGE_POSTAMBLE 0xA5A5
#define UART_MESSAGE_POSTAMBLE_H HIGH_BYTE_U16(UART_MESSAGE_POSTAMBLE)
#define UART_MESSAGE_POSTAMBLE_L LOW_BYTE_U16(UART_MESSAGE_POSTAMBLE)
#define UART_MESSAGE_PROHIBIT_COMMAND 0x00FF

#define UART_MESSAGE_SEND_ATTEMPTS (CONFIG_UART_MESSAGE_SEND_ATTEMPTS)
#define UART_CRITICAL_MESSAGE_SEND_ATTEMPTS \
  (CONFIG_UART_CRITICAL_MESSAGE_SEND_ATTEMPTS)
#define UART_MESSAGE_RECV_ATTEMPTS (CONFIG_UART_MESSAGE_RECV_ATTEMPTS)
#define UART_MESSAGE_RECV_TIMEOUT_MS (CONFIG_UART_MESSAGE_RECV_TIMEOUT_MS)
#define UART_CRITICAL_MESSAGE_RECV_TIMEOUT_MS \
  (CONFIG_UART_CRITICAL_MESSAGE_RECV_TIMEOUT_MS)

// Address(1 byte): SRC ADDR (4 bits) | DST ADDR (4 bits)
#define UART_MESSAGE_ADDR_MASK 0x0F
#define UART_MESSAGE_ADDR(addr) (uint8_t)(addr & UART_MESSAGE_ADDR_MASK)
#define UART_MESSAGE_ADDR_COMBINE(src_addr, dst_addr)      \
  ((uint8_t)((uint8_t)(UART_MESSAGE_ADDR(src_addr) << 4) | \
             UART_MESSAGE_ADDR(dst_addr)))
#define UART_MESSAGE_SRC_ADDR(addr) UART_MESSAGE_ADDR(addr >> 0x4)
#define UART_MESSAGE_DST_ADDR(addr) UART_MESSAGE_ADDR(addr)
#define UART_MESSAGE_TO_ESP32(addr) (UART_MESSAGE_DST_ADDR(addr) == ESP32_ADDR)
#define UART_MESSAGE_TO_FPGA(addr) (UART_MESSAGE_DST_ADDR(addr) == FPGA_ADDR)
#define UART_MESSAGE_FROM_FPGA(addr) (UART_MESSAGE_SRC_ADDR(addr) == FPGA_ADDR)
#define UART_MESSAGE_BROADCAST(addr) (UART_MESSAGE_DST_ADDR(addr) == BCAST_ADDR)
#define UART_RESET_DATA "\x5A\xA5\x5A\xA5"

#define UART_BUF_SIZE (UART_MESSAGE_MAX_SZ * 4)
#define UART_TX_TARGET_IS_FPGA(addr) \
  (UART_MESSAGE_TO_FPGA(addr) || UART_MESSAGE_FROM_FPGA(addr))
#define UART_TX_TARGET_INDEX(addr)                                             \
  ((UART_TX_TARGET_IS_FPGA(addr)                                               \
        ? (UART_TX_TARGETS - 1)                                                \
        : (UART_MESSAGE_ADDR(addr) < UART_TX_TARGETS ? UART_MESSAGE_ADDR(addr) \
                                                     : UART_TX_TARGETS - 1)))
#define EXPECTED_RESPONSE(message, command) (message.command == command)

#define LOG_UART_MESSAGE(tag, msg_ptr, level)                             \
  do {                                                                    \
    uint8_t data[UART_MESSAGE_MAX_SZ];                                    \
    size_t offset = 0;                                                    \
    uint16_t temp = htobe16((msg_ptr)->preamble);                         \
    memcpy(data + offset, &temp, sizeof(temp));                           \
    offset += sizeof(temp);                                               \
    memcpy(data + offset, &(msg_ptr)->addr, sizeof((msg_ptr)->addr));     \
    offset += sizeof((msg_ptr)->addr);                                    \
    temp = htobe16((msg_ptr)->command);                                   \
    memcpy(data + offset, &temp, sizeof(temp));                           \
    offset += sizeof(temp);                                               \
    memcpy(data + offset, &(msg_ptr)->length, sizeof((msg_ptr)->length)); \
    offset += sizeof((msg_ptr)->length);                                  \
    memcpy(data + offset, (msg_ptr)->payload, (msg_ptr)->length);         \
    offset += (msg_ptr)->length;                                          \
    temp = htobe16((msg_ptr)->checksum);                                  \
    memcpy(data + offset, &temp, sizeof(temp));                           \
    offset += sizeof(temp);                                               \
    temp = htobe16((msg_ptr)->postamble);                                 \
    memcpy(data + offset, &temp, sizeof(temp));                           \
    offset += sizeof(temp);                                               \
    ESP_LOG_BUFFER_HEX_LEVEL(tag, data, offset, level);                   \
  } while (0)

static const char *TAG = "UART";

typedef struct {
  SemaphoreHandle_t sem;
  uint16_t command;
#ifndef FPGA_AS_UART_SWITCH
  uint8_t mcu_addr;
#endif
} UartRxSemaphore;
typedef struct {
  SemaphoreHandle_t sem;
  uint16_t command;
  uint8_t mcu_addr;
} UartTxSemaphore;

static bool uart_initialized = false;
#ifdef FPGA_AS_UART_SWITCH
static UartRxSemaphore uart_rx_sems[UART_TX_TARGETS];
#else
static UartRxSemaphore uart_rx_sem;
#endif
static UartTxSemaphore uart_tx_sem;
static TaskMap uart_task_map{};
static uart_metrics_t uart_metrics{};

static QueueHandle_t uart_queue;
static QueueHandle_t uart_rx_queues[UART_TX_TARGETS];

// Forward declarations of helper functions
static void log_invalid_uart_message(const uart_message_t &message);
static void reset_uart_state();
static void dispatch_uart_message(const uart_message_t &message);
static esp_err_t encode_uart_message(const uart_message_t *message,
                                     uint8_t *data);

static uart_state_t UART_STATE = {
    .state = GET_PREAMBLE0,
    .received_bytes = 0,
};
static uart_message_t UART_MESSAGE = {};

#ifdef UART_STATE_DEBUG
static const char *uart_state_str(uart_message_state_t state) {
  switch (state) {
    case GET_PREAMBLE0:
      return "GET_PREAMBLE0";
    case GET_PREAMBLE1:
      return "GET_PREAMBLE1";
    case GET_ADDRESS:
      return "GET_ADDRESS";
    case GET_COMMAND0:
      return "GET_COMMAND0";
    case GET_COMMAND1:
      return "GET_COMMAND1";
    case GET_LENGTH:
      return "GET_LENGTH";
    case GET_VALUE:
      return "GET_VALUE";
    case GET_CHECKSUM0:
      return "GET_CHECKSUM0";
    case GET_CHECKSUM1:
      return "GET_CHECKSUM1";
    case GET_POSTAMBLE0:
      return "GET_POSTAMBLE0";
    case GET_POSTAMBLE1:
      return "GET_POSTAMBLE1";
    default:
      return "UNKNOWN";
  }
}

static bool handle_uart_byte(uart_message_t *message, const uint8_t *buffer,
                             size_t len, size_t idx) {
  uint8_t data = buffer[idx];
#else
static bool handle_uart_byte(uart_message_t *message, uint8_t data) {
#endif
  bool is_completed = false;
  switch (UART_STATE.state) {
    case GET_PREAMBLE0:
      if (data != UART_MESSAGE_PREAMBLE_H) {
        ESP_LOGW(TAG, "Invalid preamble 0: 0x%02X", data);
        goto RESET_STATE;
      }
      UART_STATE.state = GET_PREAMBLE1;
      break;
    case GET_PREAMBLE1:
      if (data != UART_MESSAGE_PREAMBLE_L) {
        ESP_LOGW(TAG, "Invalid preamble 1: 0x%02X", data);
        goto RESET_STATE;
      }
      message->preamble = UART_MESSAGE_PREAMBLE;
      UART_STATE.state = GET_ADDRESS;
      break;
    case GET_ADDRESS:
      // Extract host and mcu from data
      message->addr = data;
      UART_STATE.state = GET_COMMAND0;
      break;
    case GET_COMMAND0:
      // Lower octet should be reset.
      message->command = data << 8;
      UART_STATE.state = GET_COMMAND1;
      break;
    case GET_COMMAND1:
      // Upper octet should be preserved.
      message->command |= data;
      UART_STATE.state = GET_LENGTH;
      break;
    case GET_LENGTH:
      if (data > UART_MESSAGE_MAX_SZ) {
        ESP_LOGW(TAG, "Invalid length: %d", data);
        goto RESET_STATE;
      }
      message->length = data;
      UART_STATE.received_bytes = 0;
      if (message->length == 0) {
        UART_STATE.state = GET_CHECKSUM0;
      } else {
        UART_STATE.state = GET_VALUE;
      }
      break;
    case GET_VALUE:
      if (UART_STATE.received_bytes >= message->length) {
        ESP_LOGW(TAG, "Too many bytes: want %d, got %d", message->length,
                 UART_STATE.received_bytes);
        goto RESET_STATE;
      }
      message->payload[UART_STATE.received_bytes++] = data;
      if (UART_STATE.received_bytes == message->length) {
        UART_STATE.state = GET_CHECKSUM0;
      }
      break;
    case GET_CHECKSUM0:
      message->checksum = data << 8;
      UART_STATE.state = GET_CHECKSUM1;
      break;
    case GET_CHECKSUM1:
      message->checksum |= data;
      UART_STATE.state = GET_POSTAMBLE0;
      break;
    case GET_POSTAMBLE0:
      if (data != UART_MESSAGE_POSTAMBLE_H) {
        ESP_LOGW(TAG, "Invalid postamble 0: 0x%02X", data);
        goto RESET_STATE;
      }
      UART_STATE.state = GET_POSTAMBLE1;
      break;
    case GET_POSTAMBLE1:
      if (data != UART_MESSAGE_POSTAMBLE_L) {
        ESP_LOGW(TAG, "Invalid postamble 1: 0x%02X", data);
        goto RESET_STATE;
      }
      message->postamble = UART_MESSAGE_POSTAMBLE;
      is_completed = true;
      break;
  }
  return is_completed;
RESET_STATE:
#ifdef UART_STATE_DEBUG
  ESP_LOGW(TAG, "Resetting state: %s => GET_PREAMBLE0",
           uart_state_str(UART_STATE.state));
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, buffer, len, ESP_LOG_WARN);
#endif
  uart_metrics.reset_state_count++;
  reset_uart_state();
#ifdef CONFIG_SYSTEM_REBOOT_ON_UART_RESET
  esp_restart();
#endif
  return false;
}

static bool validate_uart_message(uart_message_t *message) {
  // Messages from FPGA are not expected to have checksum
  // And command is expected to be 0
  uint8_t src_addr = UART_MESSAGE_SRC_ADDR(message->addr),
          dst_addr = UART_MESSAGE_DST_ADDR(message->addr);
  ESP_LOGD(TAG, "Validating message from SRC 0x%X to DST 0x%X, command: 0x%04X",
           src_addr, dst_addr, message->command);
  if (!(UART_MESSAGE_TO_ESP32(message->addr) ||
        UART_MESSAGE_BROADCAST(message->addr))) {
    ESP_LOGW(TAG, "Invalid message: 0x%X => 0x%X, command: 0x%04X", src_addr,
             dst_addr, message->command);
    LOG_UART_MESSAGE(TAG, message, ESP_LOG_WARN);
    return false;
  }
  if (!UART_MESSAGE_FROM_FPGA(message->addr)) {
    uint16_t checksum = crc16(message->payload, message->length);
    if (checksum != message->checksum) {
      ESP_LOGW(TAG, "Invalid checksum, got 0x%04X, want 0x%04X",
               message->checksum, checksum);
      LOG_UART_MESSAGE(TAG, message, ESP_LOG_WARN);
      return false;
    }
  } else if (message->checksum != 0) {
    ESP_LOGW(TAG, "Invalid message from FPGA, command: 0x%04X",
             message->command);
    LOG_UART_MESSAGE(TAG, message, ESP_LOG_WARN);
    return false;
  }
  ESP_LOGD(TAG, "Valid message");
  return true;
}

static void handle_uart_data_event(const uint8_t *data, size_t data_len) {
  for (uint8_t i = 0; i < data_len; i++) {
#ifdef UART_STATE_DEBUG
    bool is_completed = handle_uart_byte(&UART_MESSAGE, data, data_len, i);
#else
    bool is_completed = handle_uart_byte(&UART_MESSAGE, data[i]);
#endif
    if (!is_completed) {
      continue;
    }

    // Validate the completed UART message
    if (!validate_uart_message(&UART_MESSAGE)) {
      log_invalid_uart_message(UART_MESSAGE);
      reset_uart_state();
      continue;
    }

    ESP_LOGD(TAG,
             "Received valid response message with command: 0x%04x, from 0x%X",
             UART_MESSAGE.command, UART_MESSAGE_SRC_ADDR(UART_MESSAGE.addr));
    dispatch_uart_message(UART_MESSAGE);
    reset_uart_state();
  }
}

void uart_event_task(void *parameters) {
  uart_event_t event;
  uint8_t data[UART_MESSAGE_MAX_SZ];
  int data_len = 0;

  while (uart_initialized) {
    if (xQueueReceive(uart_queue, &event, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    bzero(data, sizeof(data));
    switch (event.type) {
      case UART_PATTERN_DET: {
        ESP_LOGD(TAG, "pattern detected");
        break;
      }
      case UART_DATA: {
#ifndef CONFIG_IDF_TARGET_LINUX
        ESP_LOGD(TAG, "data received: %d", event.size);
#endif
        if (event.size == 0) {
          ESP_LOGW(TAG, "data size: 0");
          uart_flush_input(UART_PORT_NUM);
          continue;  // ignore empty data
        }
        if (event.size > UART_MESSAGE_MAX_SZ) {
#ifndef CONFIG_IDF_TARGET_LINUX
          ESP_LOGW(TAG, "data size: %d", event.size);
#endif
          uart_flush_input(UART_PORT_NUM);
          continue;  // ignore too large data
        }
        data_len = uart_read_bytes(UART_PORT_NUM, data, event.size,
                                   100 / portTICK_PERIOD_MS);
        if (data_len <= 0) {
          ESP_LOGW(TAG, "read failed");
          continue;  // ignore invalid data
        }

        ESP_LOGD(TAG, "Received %d bytes out of %d", data_len, data_len);
        ESP_LOG_BUFFER_HEXDUMP(TAG, data, data_len, ESP_LOG_DEBUG);
        handle_uart_data_event(data, data_len);
        break;
      }
      // Event of HW FIFO overflow detected
      case UART_FIFO_OVF:
        ESP_LOGW(TAG, "hw fifo overflow");
        // If fifo overflow happened, you should consider adding flow control
        // for your application. The ISR has already reset the rx FIFO, As an
        // example, we directly flush the rx buffer here in order to read more
        // data.
        uart_flush_input(UART_PORT_NUM);
        xQueueReset(uart_queue);
        break;
      // Event of UART ring buffer full
      case UART_BUFFER_FULL:
        ESP_LOGW(TAG, "rx buffer full");
        // If buffer full happened, you should consider increasing your buffer
        // size As an example, we directly flush the rx buffer here in order to
        // read more data.
        uart_flush_input(UART_PORT_NUM);
        xQueueReset(uart_queue);
        break;
      // Event of UART RX break detected
      case UART_BREAK:
        ESP_LOGD(TAG, "rx break");
        break;
      // Event of UART parity check error
      case UART_PARITY_ERR:
        ESP_LOGE(TAG, "parity error");
        break;
      // Event of UART frame error
      case UART_FRAME_ERR:
        ESP_LOGE(TAG, "frame error");
        break;
      // Others
      default:
        ESP_LOGD(TAG, "event type: %d", event.type);
        break;
    }
  }
  vTaskDelete(NULL);
}

esp_err_t uart_init() {
  uart_config_t uart_config = {
      .baud_rate = (int)UART_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 122,
#ifdef CONFIG_IDF_TARGET_LINUX
      .source_clk = 4,  // mock for testing
#else
      .source_clk = UART_SCLK_DEFAULT,
#endif
      .flags = {0},
  };

  ESP_RETURN_ON_ERROR(uart_param_config(UART_PORT_NUM, &uart_config), TAG,
                      "uart_param_config");

  ESP_RETURN_ON_ERROR(uart_set_pin(UART_PORT_NUM, UART_TXD, UART_RXD,
                                   UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                      TAG, "uart_set_pin");

  ESP_RETURN_ON_ERROR(uart_driver_install(UART_PORT_NUM,
#ifdef ENABLE_UART_TX_BUFFER
                                          UART_BUF_SIZE, UART_BUF_SIZE,
#else
                                          UART_BUF_SIZE * 2, 0,
#endif
                                          UART_QUEUE_SIZE, &uart_queue, 0),
                      TAG, "uart_driver_install");

  uart_tx_sem.sem = xSemaphoreCreateMutex();
  ESP_RETURN_ON_FALSE(uart_tx_sem.sem != NULL, ESP_ERR_NO_MEM, TAG,
                      "xSemaphoreCreateMutex: tx_sem");
  uart_tx_sem.command = 0xFFFF;
  uart_tx_sem.mcu_addr = 0xFF;
#ifndef FPGA_AS_UART_SWITCH
  uart_rx_sem.sem = xSemaphoreCreateMutex();
  ESP_RETURN_ON_FALSE(uart_rx_sem.sem != NULL, ESP_ERR_NO_MEM, TAG,
                      "xSemaphoreCreateMutex: rx_sem");
  uart_rx_sem.command = 0xFFFF;
  uart_rx_sem.mcu_addr = 0xFF;
#endif

  for (size_t i = 0; i < UART_TX_TARGETS; i++) {
#ifdef FPGA_AS_UART_SWITCH
    uart_rx_sems[i].sem = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(uart_rx_sems[i].sem != NULL, ESP_ERR_NO_MEM, TAG,
                        "xSemaphoreCreateMutex: rx_sems[%d]", i);
    uart_rx_sems[i].command = 0xFFFF;
#endif
    uart_rx_queues[i] = xQueueCreate(UART_QUEUE_SIZE, sizeof(uart_message_t));
    ESP_RETURN_ON_FALSE(uart_rx_queues[i] != NULL, ESP_ERR_NO_MEM, TAG,
                        "xQueueCreate");
  }
  uart_initialized = true;
  xTaskCreate(uart_event_task, "uart_event", 3 * 1024, NULL,
              UART_EVENT_TASK_PRIORITY, NULL);
  return ESP_OK;
}

esp_err_t encode_uart_message(const uart_message_t *message, uint8_t *data) {
  if (data == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  int offset = 0;
  uint16_t preamble = htobe16(message->preamble);
  memcpy(data, &preamble, sizeof(message->preamble));
  offset += sizeof(message->preamble);
  data[offset] = message->addr;
  offset += sizeof(message->addr);
  uint16_t command = htobe16(message->command);
  memcpy(data + offset, &command, sizeof(message->command));
  offset += sizeof(message->command);
  memcpy(data + offset, &message->length, sizeof(message->length));
  offset += sizeof(message->length);
  memcpy(data + offset, &message->payload, message->length);
  offset += message->length;
  uint16_t checksum = htobe16(message->checksum);
  memcpy(data + offset, &checksum, sizeof(message->checksum));
  offset += sizeof(message->checksum);
  uint16_t postamble = htobe16(message->postamble);
  memcpy(data + offset, &postamble, sizeof(message->postamble));

#if defined(UART_COMM_DEBUG) && defined(UART_COMM_DEBUG_DATA)
  ESP_LOGI(TAG, "Encoding message with length: %d", offset + 2);
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, offset + 2, ESP_LOG_INFO);
#endif

  return ESP_OK;
}

esp_err_t uart_send(uint8_t mcu_addr, uint16_t command, const uint8_t *payload,
                    uint8_t length, int wait_delay_ms, bool wait_for_response,
                    int send_timeout_ms) {
  esp_err_t ret = ESP_OK;
  TaskHandle_t curr_task = NULL;
  UartRxSemaphore *rx_sem;
  uint8_t msg_data_size = UART_MESSAGE_MIN_SZ + length;
  uint8_t *msg_data;
  uart_message_t message;
  uint8_t retry_limit = UART_CRITICAL_MESSAGE_SEND_ATTEMPTS;

  int sent = 0, send_attempts = 0, receive_attempts = 0,
      receive_timeout_offset = 0;
  uint32_t receive_timeout = UART_CRITICAL_MESSAGE_RECV_TIMEOUT_MS;
  if (UART_MESSAGE_TO_FPGA(mcu_addr)) {
    receive_timeout = UART_MESSAGE_RECV_TIMEOUT_MS;
    retry_limit = UART_MESSAGE_SEND_ATTEMPTS;
  }

  ESP_RETURN_ON_FALSE(payload != NULL, ESP_ERR_INVALID_ARG, TAG,
                      "Payload is NULL");
  ESP_RETURN_ON_FALSE(length <= UART_MESSAGE_PAYLOAD_MAX, ESP_ERR_INVALID_SIZE,
                      TAG, "Payload length %d exceeds UART_MESSAGE_PAYLOAD_MAX",
                      length);
  ESP_RETURN_ON_FALSE(uart_initialized, ESP_ERR_INVALID_STATE, TAG,
                      "UART not initialized");

#ifdef FPGA_AS_UART_SWITCH
  rx_sem = &uart_rx_sems[UART_TX_TARGET_INDEX(mcu_addr)];
#else
  rx_sem = &uart_rx_sem;
#endif
  ESP_RETURN_ON_FALSE(
      xSemaphoreTake(rx_sem->sem, pdMS_TO_TICKS(send_timeout_ms)) == pdTRUE,
      ESP_ERR_TIMEOUT, TAG,
      "Failed to acquire semaphore for command 0x%04x to MCU 0x%X, taken by "
#ifdef FPGA_AS_UART_SWITCH
      "0x%04x",
      command, mcu_addr, rx_sem->command);
#else
      "0x%04x, taker address: 0x%02X",
      command, mcu_addr, rx_sem->command, rx_sem->mcu_addr);
#endif

  // Claim the semaphore
  rx_sem->command = command;
#ifndef FPGA_AS_UART_SWITCH
  rx_sem->mcu_addr = mcu_addr;
#endif
  uart_metrics.sent_count++;

  // Construct the message
  message = {
      .preamble = UART_MESSAGE_PREAMBLE,
      .addr = UART_MESSAGE_ADDR_COMBINE(ESP32_ADDR, mcu_addr),
      .command = command,
      .length = length,
      .payload = {},
      .checksum = 0,
      .postamble = UART_MESSAGE_POSTAMBLE,
  };
  memset(message.payload, 0, UART_MESSAGE_PAYLOAD_MAX);
  memcpy(message.payload, payload, length);
  message.checksum = crc16(message.payload, length);

  // Encode the message
  msg_data = (uint8_t *)malloc(msg_data_size);
  ESP_GOTO_ON_FALSE(msg_data != NULL, ESP_ERR_NO_MEM, EXIT, TAG,
                    "Failed to allocate memory for msg_data");
  ESP_GOTO_ON_ERROR(encode_uart_message(&message, msg_data), EXIT, TAG,
                    "Failed to encode UART message");

  if (wait_for_response) {
    // Register the current task to the task map that waits for a response
    curr_task = xTaskGetCurrentTaskHandle();
    ESP_GOTO_ON_FALSE(
        uart_task_map.add_task(mcu_addr, command, curr_task), ESP_ERR_TIMEOUT,
        EXIT, TAG,
        "Failed to add task to task map, potential concurrency issue detected");
  }

SENDING:
  send_attempts++;
  ESP_GOTO_ON_FALSE(
      xSemaphoreTake(uart_tx_sem.sem, pdMS_TO_TICKS(send_timeout_ms)) == pdTRUE,
      ESP_ERR_TIMEOUT, RESENDING, TAG,
      "uart_tx_sem taken by MCU 0x%X for command 0x%04X", uart_tx_sem.mcu_addr,
      uart_tx_sem.command);
  // Claim the TX semaphore
  uart_tx_sem.mcu_addr = mcu_addr;
  uart_tx_sem.command = command;
  ESP_LOGD(TAG, "Sending message (command: 0x%04X) to MCU 0x%X, attempt: %d",
           command, mcu_addr, send_attempts);
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, (const char *)msg_data, msg_data_size,
                           ESP_LOG_DEBUG);
  sent = uart_write_bytes(UART_PORT_NUM, (const char *)msg_data, msg_data_size);

  uart_tx_sem.mcu_addr = 0xFF;
  uart_tx_sem.command = 0xFFFF;
  xSemaphoreGive(uart_tx_sem.sem);

  ESP_GOTO_ON_FALSE(sent != -1, ESP_ERR_INVALID_ARG, EXIT, TAG,
                    "uart_write_bytes returned -1");
  ESP_GOTO_ON_FALSE(sent == msg_data_size, ESP_ERR_NOT_FINISHED, RESENDING, TAG,
                    "Only sent %d bytes out of %d", sent, msg_data_size);
#ifndef ENABLE_UART_TX_BUFFER
  ESP_GOTO_ON_ERROR(uart_wait_tx_done(UART_PORT_NUM, 100), RESENDING, TAG,
                    "uart_wait_tx_done failed");
#endif
  ESP_LOGD(TAG, "Message sent, command: 0x%04X, length: %d, attempt: %d",
           command, sent, send_attempts);
  if (!wait_for_response) {
    goto EXIT;
  }
  receive_timeout_offset = wait_delay_ms;
RECEIVING:
  receive_attempts++;
  if (xTaskNotifyWaitIndexed(
          UART_NOTIFY_INDEX, 0, 0, NULL,
          pdMS_TO_TICKS(receive_timeout + receive_timeout_offset)) == pdPASS) {
    ESP_LOGD(TAG, "Received a task notification");
    goto EXIT;
  }
  receive_timeout_offset = 0;
  ESP_GOTO_ON_FALSE(
      receive_attempts >= UART_MESSAGE_RECV_ATTEMPTS, ESP_ERR_TIMEOUT,
      RECEIVING, TAG,
      "Response message for command 0x%04X timed out after %" PRIu32
      "ms, attempt: %d/%d",
      command, receive_timeout, receive_attempts, UART_MESSAGE_RECV_ATTEMPTS);
  goto RESENDING;
RESENDING:
  uart_metrics.resend_count++;
  ESP_GOTO_ON_FALSE(
      send_attempts < retry_limit, ESP_ERR_TIMEOUT, EXIT, TAG,
      "Giving up on sending message command 0x%04X to MCU 0x%X after %d/%d "
      "attempts",
      command, mcu_addr, send_attempts, retry_limit);
  ESP_LOGD(TAG,
           "Resending message (command 0x%04X) to MCU 0x%X, attempt: "
           "%d/%d",
           command, mcu_addr, send_attempts, retry_limit);
  goto SENDING;
EXIT:
  if (msg_data != NULL) {
    free(msg_data);
  }
  if (ret != ESP_OK) {
    ESP_ERROR_COMPLAIN(ret, "uart_send");
    uart_metrics.sent_failed_count++;
    if (wait_for_response) {
      uart_task_map.remove_task(mcu_addr, command);
    }
  }
  // Release the RX semaphore
  rx_sem->command = 0xFFFF;
#ifndef FPGA_AS_UART_SWITCH
  rx_sem->mcu_addr = 0xFF;
#endif
  xSemaphoreGive(rx_sem->sem);
  return ret;
}

esp_err_t uart_read(uint8_t mcu_addr, uint16_t command, void *response,
                    uint8_t *response_length) {
  esp_err_t ret = ESP_OK;
  uint32_t receive_timeout = UART_MESSAGE_RECV_TIMEOUT_MS;
  uart_message_t message;
  QueueHandle_t response_queue;
  TickType_t timeout = pdMS_TO_TICKS(receive_timeout);

  ESP_RETURN_ON_FALSE(response != NULL && response_length != NULL,
                      ESP_ERR_INVALID_ARG, TAG,
                      "response or response_length is NULL");
  ESP_RETURN_ON_FALSE(uart_initialized, ESP_ERR_INVALID_STATE, TAG,
                      "UART not initialized");

  response_queue = uart_rx_queues[UART_TX_TARGET_INDEX(mcu_addr)];
  ESP_RETURN_ON_FALSE(
      xQueueReceive(response_queue, &message, timeout) == pdTRUE,
      ESP_ERR_INVALID_STATE, TAG,
      "Received a task notification but mcu %d response queue is empty",
      mcu_addr);
  ESP_LOGD(TAG, "Received message, length: %d", message.length);
  ESP_RETURN_ON_FALSE(message.command != UART_MESSAGE_PROHIBIT_COMMAND,
                      ESP_ERR_NOT_ALLOWED, TAG,
                      "Sending a prohibited command 0x%04X to MCU 0x%X",
                      command, mcu_addr);
  ESP_RETURN_ON_FALSE(EXPECTED_RESPONSE(message, command),
                      ESP_ERR_INVALID_RESPONSE, TAG,
                      "Received an invalid response message from MCU %d, "
                      "want: 0x%04X, got: 0x%04X",
                      mcu_addr, command, message.command);
  ESP_RETURN_ON_FALSE(
      *response_length == UART_MESSAGE_VARIABLE_LENGTH ||
          message.length >= *response_length,
      ESP_ERR_INVALID_SIZE, TAG,
      "Not enough bytes in response for command 0x%04X, length: "
      "%d, want: %d, MCU: %d",
      command, message.length, *response_length, mcu_addr);
  // Copy the response payload
  *response_length = std::min(*response_length, message.length);
  memcpy(response, message.payload, *response_length);
  return ret;
}

#ifdef UART_FEED_TEST
void uart_feed_test() {
  uint8_t data[] = {0x80};
  while (true) {
    uart_write_bytes(UART_PORT_NUM, (const char *)data, 1);
  }
}
#endif

uart_metrics_t *uart_get_metrics() { return &uart_metrics; }

// Helper function to log details of an invalid UART message
static void log_invalid_uart_message(const uart_message_t &message) {
  uint16_t command = message.command;
  uint8_t addr = message.addr;
  uint8_t src_addr = UART_MESSAGE_SRC_ADDR(addr);
  uint16_t dst_addr = UART_MESSAGE_DST_ADDR(addr);

  ESP_LOGW(
      TAG,
      "Received an invalid message with command: 0x%04x, from 0x%X => 0x%X",
      command, src_addr, dst_addr);
  LOG_UART_MESSAGE(TAG, &message, ESP_LOG_WARN);
}

// Helper function to reset the UART state after processing a message
static void reset_uart_state() {
  UART_STATE.state = GET_PREAMBLE0;
  UART_STATE.received_bytes = 0;
  memset(&UART_MESSAGE, 0, sizeof(UART_MESSAGE));
}

// Helper function to dispatch a valid UART message to the appropriate task or
// PortManager
static void dispatch_uart_message(const uart_message_t &message) {
  uint16_t command = message.command;
  uint8_t addr = message.addr;
  uint8_t src_addr = UART_MESSAGE_SRC_ADDR(addr);
  uint8_t queue_idx = UART_TX_TARGET_INDEX(src_addr);
  TaskHandle_t task;
  bool send_success = false;

  ESP_LOGD(TAG, "Received valid response message with cmd 0x%04x from 0x%X",
           command, src_addr);

  // Retrieve the task associated with the source address and command
  task = uart_task_map.get_task(src_addr, command);
  if (task == NULL) {
    if (UART_MESSAGE_FROM_FPGA(addr)) {
      ESP_LOGW(TAG, "Received cmd 0x%04x from FPGA without a command handler",
               command);
      LOG_UART_MESSAGE(TAG, &message, ESP_LOG_WARN);
      return;
    }

    // No associated task found; enqueue the message to PortManager for further
    // handling
    PortManager::GetInstance().EnqueueUARTMessage(message, src_addr);
    ESP_LOGD(TAG,
             "No associated task found for command 0x%04X from 0x%X. Enqueued "
             "to PortManager.",
             command, src_addr);
    return;  // Exit the function or continue to the next iteration, depending
             // on context
  }

  // Associated task found; proceed to notify the task

  // Remove the task from the task map as it is about to be notified
  uart_task_map.remove_task(src_addr, command);
  ESP_LOGD(TAG, "Sending response message for command 0x%04X to queue %d",
           command, queue_idx);

  // Attempt to send the UART message to the designated queue
  send_success = (xQueueSend(uart_rx_queues[queue_idx], &message,
                             UART_ENQUEUE_TIMEOUT) == pdTRUE);
  if (!send_success) {
    ESP_LOGW(TAG,
             "Failed to send response message for command 0x%04X from 0x%X",
             command, src_addr);
    return;
  }

#ifndef CONFIG_IDF_TARGET_LINUX
  ESP_LOGD(TAG, "Notifying task 0x%" PRIx32 " for command 0x%04X",
           (uint32_t)task, command);
#endif
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // Notify the task from ISR context
  xTaskNotifyIndexedFromISR(task, UART_NOTIFY_INDEX, command,
                            eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
}
