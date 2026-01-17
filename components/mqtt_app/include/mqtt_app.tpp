#ifndef MQTT_APP_T
#define MQTT_APP_T

#ifdef CONFIG_ENABLE_FACTORY_MODE

#include "esp_log.h"

template <MqttTopicParser::Command::CommandSource ExpectedSource,
          typename CmdEnumT, typename PbRequestT,
          const pb_msgdesc_t* PbRequestFields, typename PbResponseT>
void MQTTClient::ProcessCommandMsg(MQTTMsg& msg) {
  const char* TAG = "MQTTClient";
  ESP_LOGW(
      TAG,
      "MQTT disabled in factory mode; dropping command 0x%02x from source %d",
      static_cast<int>(msg.cmd.id), static_cast<int>(msg.cmd.source));
  (void)ExpectedSource;
  (void)PbRequestFields;
  (void)msg;
}

#else  // CONFIG_ENABLE_FACTORY_MODE

#include <memory>

#include "app.h"
#include "esp_log.h"
#include "pb_decode.h"

template <MqttTopicParser::Command::CommandSource ExpectedSource,
          typename CmdEnumT, typename PbRequestT,
          const pb_msgdesc_t* PbRequestFields, typename PbResponseT>
void MQTTClient::ProcessCommandMsg(MQTTMsg& msg) {
  const char* TAG = "MQTTClient";
  // 1. Verify Source
  if (msg.cmd.source != ExpectedSource) {
    ESP_LOGE(TAG, "Invalid command source: %d, expected %d",
             (int)msg.cmd.source, (int)ExpectedSource);
    return;
  }

  const char* sourceStr =
      (ExpectedSource == MqttTopicParser::Command::CommandSource::ENDUSER)
          ? "enduser"
          : "admin";
  ESP_LOGD(TAG, "Processing %s message", sourceStr);

  // 2. Extract Command and Data (assuming ID is already validated)
  CmdEnumT cmd = static_cast<CmdEnumT>(msg.cmd.id);
  const std::vector<uint8_t>& data = msg.data;  // Use const reference

  // 3. Prepare Protobuf Request/Response on heap to avoid stack overflow
  // (pb_AdminCommandRequest is 3560 bytes, pb_AdminCommandResponse is 542)
  auto request = std::make_unique<PbRequestT>();
  auto response = std::make_unique<PbResponseT>();
  *request = {};
  *response = {};
  pb_istream_t stream = pb_istream_from_buffer(
      reinterpret_cast<const pb_byte_t*>(data.data()), data.size());

  // 4. Decode Request and Execute
  if (pb_decode(&stream, PbRequestFields, request.get())) {
    ESP_LOGD(TAG, "MQTT Message decoded, cmd: 0x%02x", static_cast<int>(cmd));
    // Execute the command (assuming App method is overloaded/templated)
    App::GetInstance().ExecMQTTSrv(cmd, *request, *response);
    response->id = request->id;  // Copy request ID to response ID
  } else {
    // Handle Decode Failure
    ESP_LOGW(TAG, "Failed to decode %s request: %s", sourceStr,
             PB_GET_ERROR(&stream));
    response->status =
        pb_CommandStatus_FAILURE;  // Assuming common status enum/value
    response->id = 0;  // Or copy request.id if appropriate even on decode
                       // failure? Original code set 0.
  }

  // 5. Publish Response (assuming Publish is overloaded/templated)
  this->Publish(cmd, *response, 0);  // Assuming 0 is the QoS
}

#endif  // CONFIG_ENABLE_FACTORY_MODE

#endif
