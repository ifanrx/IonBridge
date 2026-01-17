#include "telemetry_stream_handler.h"

#include "admin.pb.h"
#include "command.pb.h"
#include "esp_err.h"
#include "telemetry_task.h"

using namespace TelemetryStreamHandler;

esp_err_t FromMQTT::StartTelemetryStream(const pb_CommandRequest &request,
                                         pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_start_telemetry_stream_tag;
  TelemetryTask::GetInstance().SubscribeTelemetryStream();
  return ESP_OK;
}

esp_err_t FromMQTT::StopTelemetryStream(const pb_CommandRequest &request,
                                        pb_CommandResponse &response) {
  response.which_payload = pb_CommandResponse_stop_telemetry_stream_tag;
  TelemetryTask::GetInstance().UnsubscribeTelemetryStream();
  return ESP_OK;
}

esp_err_t FromMQTT::SetTelemetryDebug(const pb_AdminCommandRequest &request,
                                      pb_AdminCommandResponse &response) {
  response.which_payload = pb_AdminCommandResponse_set_telemetry_debug_tag;
  bool debug = request.payload.set_telemetry_debug.enable;
  TelemetryTask::GetInstance().SetDebug(debug);
  return ESP_OK;
}
