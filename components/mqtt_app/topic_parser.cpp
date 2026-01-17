#include "topic_parser.h"

#include <charconv>
#include <cstdio>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "machine_info.h"
#include "service.h"

using namespace MqttTopicParser;

// Helper function (optional, reduces repetition)
namespace {  // Anonymous namespace for internal linkage
std::string FormatTopicWithCmd(const char* fmt, const std::string& psn,
                               uint8_t cmd_id) {
  // First call to snprintf to determine the required buffer size.
  // Use %u format specifier for uint8_t (promoted to unsigned int).
  int required_size =
      snprintf(nullptr, 0, fmt, psn.c_str(), static_cast<unsigned int>(cmd_id));

  // Check for snprintf error during size calculation
  if (required_size < 0) {
    // Consider logging an error here
    return "";  // Return empty string on failure
  }

  // Allocate buffer. Ensure <vector> is included and no name collisions exist.
  std::vector<char> buffer(required_size + 1);  // +1 for the null terminator

  // Second call to snprintf to actually format the string into the buffer
  int written_size = snprintf(buffer.data(), buffer.size(), fmt, psn.c_str(),
                              static_cast<unsigned int>(cmd_id));

  // Check for snprintf error during writing
  if (written_size < 0 || written_size >= buffer.size()) {
    // This check catches runtime errors or unexpected truncation
    // Consider logging an error here
    return "";  // Return empty string on failure
  }

  // Construct the string from the buffer, using the actual written size
  return std::string(buffer.data(), written_size);
}

// Helper function using snprintf for topics with only PSN
std::string FormatTopicWithPSN(const char* fmt, const std::string& psn) {
  int required_size = snprintf(nullptr, 0, fmt, psn.c_str());
  if (required_size < 0) {
    return "";
  }

  std::vector<char> buffer(required_size + 1);  // +1 for null terminator

  int written_size = snprintf(buffer.data(), buffer.size(), fmt, psn.c_str());
  if (written_size < 0 || written_size >= buffer.size()) {
    return "";
  }

  return std::string(buffer.data(), written_size);
}
}  // namespace

std::string Topic::GetSubscribeTopic() {
  return FormatTopicWithPSN(
      std::string(Topic::DEVICE_SUBSCRIBE_TOPIC_TMPL).c_str(),
      MachineInfo::GetInstance().GetPSN());
}

std::string Topic::GetEnduserRespTopic(ServiceCommand cmd) {
  return FormatTopicWithCmd(
      std::string(Topic::DEVICE_ENDUSER_RESPONSE_TMPL).c_str(),
      MachineInfo::GetInstance().GetPSN(), (uint8_t)cmd);
}

std::string Topic::GetAdminRespTopic(AdminCommand cmd) {
  return FormatTopicWithCmd(
      std::string(Topic::DEVICE_ADMIN_RESPONSE_TMPL).c_str(),
      MachineInfo::GetInstance().GetPSN(), (uint8_t)cmd);
}

std::string Topic::GetTelemetryTopic(TelemetryCommand cmd) {
  return FormatTopicWithCmd(std::string(Topic::DEVICE_TELEMETRY_TMPL).c_str(),
                            MachineInfo::GetInstance().GetPSN(), (uint8_t)cmd);
}

bool Command::CommandInfo::IsValid() const {
  uint8_t max_cmd;
  switch (this->source) {
    case CommandSource::ENDUSER:
      max_cmd = static_cast<uint8_t>(ServiceCommand::MAX_CMD);
      break;
    case CommandSource::ADMIN:
      max_cmd = static_cast<uint8_t>(AdminCommand::MAX_CMD);
      break;
    case CommandSource::TELEMETRY:
      max_cmd = static_cast<uint8_t>(TelemetryCommand::MAX_CMD);
      break;
    default:
      return false;
  }
  return this->id < max_cmd;
}

std::string Command::CommandInfo::ToString() const {
  std::string source_str;
  switch (this->source) {
    case CommandSource::ENDUSER:
      source_str = "enduser";
      break;
    case CommandSource::ADMIN:
      source_str = "admin";
      break;
    case CommandSource::TELEMETRY:
      source_str = "telemetry";
      break;
    default:
      source_str = "unknown";
      break;
  }
  return "CommandInfo{source: " + source_str +
         ", id: " + std::to_string(static_cast<int>(this->id)) + "}";
}

std::optional<Command::CommandInfo> Command::ParseRequest(
    std::string_view topic) {
  // 1. Find the last '/' (breform command ID)
  size_t last_slash_pos = topic.find_last_of('/');
  if (last_slash_pos == std::string_view::npos ||
      last_slash_pos + 1 >= topic.length()) {
    return std::nullopt;  // No command ID part or topic ends with '/'
  }
  std::string_view command_id_str = topic.substr(last_slash_pos + 1);

  // 2. Find the second-to-last '/' (before "request")
  size_t second_last_slash_pos = topic.find_last_of('/', last_slash_pos - 1);
  if (second_last_slash_pos == std::string_view::npos ||
      second_last_slash_pos + 1 >= topic.length()) {
    return std::nullopt;  // Malformed topic, missing segment before ID
  }
  std::string_view request_segment = topic.substr(
      second_last_slash_pos + 1, last_slash_pos - second_last_slash_pos - 1);

  // 3. Verify the segment is "request"
  if (request_segment != "request") {
    return std::nullopt;  // Not a request topic
  }

  // 4. Find the third-to-last '/' (before source "enduser" or "admin")
  size_t third_last_slash_pos =
      topic.find_last_of('/', second_last_slash_pos - 1);
  if (third_last_slash_pos == std::string_view::npos ||
      third_last_slash_pos + 1 >= topic.length()) {
    return std::nullopt;  // Malformed topic, missing segment before "request"
  }
  std::string_view source_segment =
      topic.substr(third_last_slash_pos + 1,
                   second_last_slash_pos - third_last_slash_pos - 1);

  // 5. Determine the source
  CommandSource source;
  if (source_segment == "enduser") {
    source = CommandSource::ENDUSER;
  } else if (source_segment == "admin") {
    source = CommandSource::ADMIN;
  } else {
    return std::nullopt;  // Source segment is neither "enduser" nor "admin"
  }

  // 6. Parse the command ID string as an integer
  uint8_t command_id;
  auto result = std::from_chars(command_id_str.data(),
                                command_id_str.data() + command_id_str.size(),
                                command_id);

  // Check if parsing succeeded and consumed the entire string
  if (result.ec != std::errc()) {
    return std::nullopt;  // Parsing failed
  }

  if (result.ptr != command_id_str.data() + command_id_str.size()) {
    return std::nullopt;  // Didn't consume the whole ID part
  }

  // 7. Validate the command ID and source
  CommandInfo info = {source, command_id};
  if (info.IsValid()) {
    return info;  // Return the parsed command info
  }

  return std::nullopt;  // Invalid command ID or source
}
