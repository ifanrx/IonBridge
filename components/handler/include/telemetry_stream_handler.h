#ifndef TELEMETRY_STREAM_HANDLER_H_
#define TELEMETRY_STREAM_HANDLER_H_

#include "admin.pb.h"
#include "command.pb.h"
#include "esp_err.h"

namespace TelemetryStreamHandler {
namespace FromMQTT {
esp_err_t StartTelemetryStream(const pb_CommandRequest &request,
                               pb_CommandResponse &response);
esp_err_t StopTelemetryStream(const pb_CommandRequest &request,
                              pb_CommandResponse &response);
esp_err_t SetTelemetryDebug(const pb_AdminCommandRequest &request,
                            pb_AdminCommandResponse &response);
};  // namespace FromMQTT
};  // namespace TelemetryStreamHandler

#endif /* TELEMETRY_STREAM_HANDLER_H_ */
