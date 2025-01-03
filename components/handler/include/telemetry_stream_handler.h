#ifndef TELEMETRY_STREAM_HANDLER_H_
#define TELEMETRY_STREAM_HANDLER_H_

#include <stdint.h>

#include <vector>

#include "app.h"
#include "esp_err.h"

namespace TelemetryStreamHandler {
esp_err_t StartTelemetryStream(AppContext &ctx,
                               const std::vector<uint8_t> &request,
                               std::vector<uint8_t> &response);
esp_err_t StopTelemetryStream(AppContext &ctx,
                              const std::vector<uint8_t> &request,
                              std::vector<uint8_t> &response);
}  // namespace TelemetryStreamHandler

#endif /* TELEMETRY_STREAM_HANDLER_H_ */
