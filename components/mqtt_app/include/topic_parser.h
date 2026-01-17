#ifndef TOPIC_PARSER_H
#define TOPIC_PARSER_H

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "service.h"

namespace MqttTopicParser {
namespace Topic {
constexpr std::string_view DEVICE_ENDUSER_REQUEST_TMPL =
    "device/%s/enduser/request/+";
constexpr std::string_view DEVICE_ADMIN_REQUEST_TMPL =
    "device/%s/admin/request/+";
constexpr std::string_view DEVICE_SUBSCRIBE_TOPIC_TMPL =
    "device/%s/+/request/+";
constexpr std::string_view DEVICE_ENDUSER_RESPONSE_TMPL =
    "device/%s/enduser/response/%d";
constexpr std::string_view DEVICE_ADMIN_RESPONSE_TMPL =
    "device/%s/admin/response/%d";
constexpr std::string_view DEVICE_TELEMETRY_TMPL = "device/%s/telemetry/%d";

std::string GetSubscribeTopic();
std::string GetEnduserRespTopic(ServiceCommand cmd);
std::string GetAdminRespTopic(AdminCommand cmd);
std::string GetTelemetryTopic(TelemetryCommand cmd);
};  // namespace Topic

namespace Command {
enum class CommandSource : uint8_t {
  ENDUSER,
  ADMIN,
  TELEMETRY,
};

struct CommandInfo {
  CommandSource source;
  uint8_t id;

  bool IsValid() const;
  std::string ToString() const;
};

std::optional<CommandInfo> ParseRequest(std::string_view topic);

};  // namespace Command
};  // namespace MqttTopicParser

#endif
