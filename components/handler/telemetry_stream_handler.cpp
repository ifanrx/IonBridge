#include "telemetry_stream_handler.h"

#include <cstdint>
#include <vector>

#include "esp_err.h"
#include "esp_log.h"
#include "telemetry_task.h"

static const char *TAG = "TelemetryStream";

esp_err_t TelemetryStreamHandler::StartTelemetryStream(
    const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  TelemetryTask::GetInstance().SubscribeTelemetryStream();
  return ESP_OK;
}

esp_err_t TelemetryStreamHandler::StopTelemetryStream(
    const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  TelemetryTask::GetInstance().UnsubscribeTelemetryStream();
  return ESP_OK;
}

esp_err_t TelemetryStreamHandler::SetTelemetryDebug(
    const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  if (request.size() < 1) {
    ESP_LOGE(TAG, "Invalid request size: %zu", request.size());
    return ESP_ERR_INVALID_SIZE;
  }
  bool debug = request[0] != 0;
  TelemetryTask::GetInstance().SetDebug(debug);
  return ESP_OK;
}
