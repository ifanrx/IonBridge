#include <cstdint>
#include <cstring>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/projdefs.h"
#include "freertos/semphr.h"
#include "gpu_data_types.h"
#include "rpc.h"
#include "sdkconfig.h"
#include "uart.h"

#define LOCK_TIMEOUT_MS (CONFIG_UART_MESSAGE_RECV_TIMEOUT_MS + 50)

static const char *TAG = "GPU";

static struct {
  SemaphoreHandle_t lock;
} rpc_lock = {
    .lock = xSemaphoreCreateMutex(),
};

static uint8_t generate_nonce() { return (uint8_t)(esp_random() & 0xFF); }

static esp_err_t send_gpu_command(uint16_t command, const uint8_t *request,
                                  uint8_t request_size, RPCResponse *response) {
  ESP_RETURN_ON_FALSE(request != nullptr && response != nullptr,
                      ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

  ESP_RETURN_ON_FALSE(
      xSemaphoreTake(rpc_lock.lock, pdMS_TO_TICKS(LOCK_TIMEOUT_MS)),
      ESP_ERR_TIMEOUT, TAG, "Failed to take lock for command 0x%04X", command);

  uint8_t resp_size = sizeof(RPCResponse);
  esp_err_t ret;

  ret = uart_send(GPU_ADDR, command, request, request_size);
  if (ret == ESP_OK) {
    ret = uart_read(GPU_ADDR, command, (uint8_t *)response, &resp_size);
  }

  if (ret == ESP_OK && response->status != RPC_STATUS_OK) {
    ESP_LOGE(TAG, "GPU command 0x%04X failed with status: 0x%02X", command,
             response->status);
    ret = ESP_ERR_INVALID_RESPONSE;
  }

  xSemaphoreGive(rpc_lock.lock);
  return ret;
}

esp_err_t rpc::gpu::set_backlight_intensity(uint8_t intensity) {
  SetBacklightIntensityRequest req = {
      .nonce = generate_nonce(),
      .intensity = intensity,
  };
  RPCResponse resp = {};

  return send_gpu_command(CMD_SET_BACKLIGHT_INTENSITY, (const uint8_t *)&req,
                          sizeof(req), &resp);
}

esp_err_t rpc::gpu::set_display_status(bool on) {
  SetDisplayStatusRequest req = {
      .on = static_cast<uint8_t>(on ? 1 : 0),
  };
  RPCResponse resp = {};

  return send_gpu_command(CMD_SET_DISPLAY_STATUS, (const uint8_t *)&req,
                          sizeof(req), &resp);
}

esp_err_t rpc::gpu::set_animation_index(GPUAnimationIndex animation_index,
                                        PreCompositeMode pre_composite_mode,
                                        PostCompositeMode post_composite_mode) {
  SetAnimationIndexRequest req = {
      .nonce = generate_nonce(),
      .animation_index = animation_index,
      .pre_composite_mode = pre_composite_mode,
      .post_composite_mode = post_composite_mode,
  };
  RPCResponse resp = {};

  return send_gpu_command(CMD_SET_ANIMATION_INDEX, (const uint8_t *)&req,
                          sizeof(req), &resp);
}

esp_err_t rpc::gpu::set_animation_data(const SetAnimationDataRequest &data) {
  RPCResponse resp = {};

  return send_gpu_command(CMD_SET_ANIMATION_DATA, (const uint8_t *)&data,
                          sizeof(data), &resp);
}

esp_err_t rpc::gpu::draw_text(const DrawTextRequest &request) {
  RPCResponse resp = {};

  return send_gpu_command(CMD_DRAW_TEXT, (const uint8_t *)&request,
                          sizeof(request), &resp);
}

esp_err_t rpc::gpu::prepare_framebuffer(
    const PrepareFramebufferRequest &request) {
  RPCResponse resp = {};

  return send_gpu_command(CMD_PREPARE_FRAMEBUFFER, (const uint8_t *)&request,
                          sizeof(request), &resp);
}

esp_err_t rpc::gpu::set_framebuffer_chunk(
    const SetFramebufferRequest &request) {
  RPCResponse resp = {};

  return send_gpu_command(CMD_SET_FRAMEBUFFER, (const uint8_t *)&request,
                          sizeof(request), &resp);
}
