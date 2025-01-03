#include "telemetry_stream_handler.h"

#include "telemetry_task.h"

static const char *TAG __attribute__((unused)) = "TelemetryStream";

esp_err_t TelemetryStreamHandler::StartTelemetryStream(
    AppContext &ctx, const std::vector<uint8_t> &request,
    std::vector<uint8_t> &response) {
  TelemetryTask::GetInstance()->SubscribeTelemetryStream();
  return ESP_OK;
}

esp_err_t TelemetryStreamHandler::StopTelemetryStream(
    AppContext &ctx, const std::vector<uint8_t> &request,
    std::vector<uint8_t> &response) {
  TelemetryTask::GetInstance()->UnsubscribeTelemetryStream();
  return ESP_OK;
}
