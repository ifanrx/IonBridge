#include "telemetry_stream_handler.h"

#include <cstdint>
#include <vector>

#include "app.h"
#include "esp_err.h"
#include "esp_log.h"
#include "telemetry_task.h"

static const char *TAG = "TelemetryStream";

esp_err_t TelemetryStreamHandler::StartTelemetryStream(
    AppContext &ctx, const std::vector<uint8_t> &request,
    std::vector<uint8_t> &response) {
  if (TelemetryTask::GetInstance() == nullptr) {
    ESP_LOGE(TAG, "TelemetryTask is not initialized");
    return ESP_ERR_INVALID_STATE;
  }
  TelemetryTask::GetInstance()->SubscribeTelemetryStream();
  return ESP_OK;
}

esp_err_t TelemetryStreamHandler::StopTelemetryStream(
    AppContext &ctx, const std::vector<uint8_t> &request,
    std::vector<uint8_t> &response) {
  if (TelemetryTask::GetInstance() == nullptr) {
    ESP_LOGE(TAG, "TelemetryTask is not initialized");
    return ESP_ERR_INVALID_STATE;
  }
  TelemetryTask::GetInstance()->UnsubscribeTelemetryStream();
  return ESP_OK;
}
