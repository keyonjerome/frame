#include "Protocol.h"

#include <algorithm>
#include <cctype>
#include <sstream>

namespace {

bool StartsWith(const std::string& value, const std::string& prefix) {
  return value.rfind(prefix, 0) == 0;
}

}  // namespace

std::string TrimAsciiWhitespace(const std::string& value) {
  const auto begin = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  });
  if (begin == value.end()) {
    return "";
  }

  const auto end = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  });
  return std::string(begin, end.base());
}

ParsedCommand ParseCommandLine(const std::string& line) {
  const std::string command = TrimAsciiWhitespace(line);
  if (command.empty()) {
    ParsedCommand parsed;
    parsed.type = CommandType::kEmpty;
    return parsed;
  }

  if (command == "PING") {
    ParsedCommand parsed;
    parsed.type = CommandType::kPing;
    return parsed;
  }
  if (command == "STATUS") {
    ParsedCommand parsed;
    parsed.type = CommandType::kStatus;
    return parsed;
  }
  if (command == "STOP") {
    ParsedCommand parsed;
    parsed.type = CommandType::kStop;
    return parsed;
  }

  // START is the only command that carries an argument. We keep the parser
  // intentionally simple: one command word, followed by one raw path token.
  if (StartsWith(command, "START")) {
    if (command.size() <= 6 || command[5] != ' ') {
      ParsedCommand parsed;
      parsed.type = CommandType::kInvalid;
      parsed.error_reason = "internal_error";
      return parsed;
    }

    const std::string output_dir = TrimAsciiWhitespace(command.substr(6));
    if (output_dir.empty()) {
      ParsedCommand parsed;
      parsed.type = CommandType::kInvalid;
      parsed.error_reason = "internal_error";
      return parsed;
    }

    ParsedCommand parsed;
    parsed.type = CommandType::kStart;
    parsed.argument = output_dir;
    return parsed;
  }

  ParsedCommand parsed;
  parsed.type = CommandType::kInvalid;
  parsed.error_reason = "internal_error";
  return parsed;
}

std::string StateToString(RecorderState state) {
  switch (state) {
    case RecorderState::IDLE:
      return "IDLE";
    case RecorderState::STARTING:
      return "STARTING";
    case RecorderState::RECORDING:
      return "RECORDING";
    case RecorderState::STOPPING:
      return "STOPPING";
    case RecorderState::ERROR:
      return "ERROR";
  }
  return "ERROR";
}

int HealthyFlag(RecorderState state) {
  return state == RecorderState::ERROR ? 0 : 1;
}

std::string DashIfEmpty(const std::string& value) {
  return value.empty() ? "-" : value;
}

std::string JsonEscape(const std::string& value) {
  std::ostringstream stream;
  for (const unsigned char ch : value) {
    switch (ch) {
      case '"':
        stream << "\\\"";
        break;
      case '\\':
        stream << "\\\\";
        break;
      case '\b':
        stream << "\\b";
        break;
      case '\f':
        stream << "\\f";
        break;
      case '\n':
        stream << "\\n";
        break;
      case '\r':
        stream << "\\r";
        break;
      case '\t':
        stream << "\\t";
        break;
      default:
        if (ch < 0x20) {
          stream << "\\u";
          const char* digits = "0123456789abcdef";
          stream << '0' << '0' << digits[(ch >> 4) & 0x0f] << digits[ch & 0x0f];
        } else {
          stream << static_cast<char>(ch);
        }
        break;
    }
  }
  return stream.str();
}

std::string FormatStatusLine(const StatusSnapshot& snapshot,
                             const CameraStatusSnapshot& camera) {
  std::ostringstream stream;
  stream << "{\"type\":\"status\","
         << "\"state\":\"" << StateToString(snapshot.state) << "\","
         << "\"healthy\":" << (HealthyFlag(snapshot.state) != 0 ? "true" : "false") << ","
         << "\"current_file\":\"" << JsonEscape(snapshot.current_file) << "\","
         << "\"last_error\":\"" << JsonEscape(snapshot.last_error) << "\","
         << "\"capture_path\":\"" << JsonEscape(snapshot.capture_path) << "\","
         << "\"local_recording_warning\":\""
         << JsonEscape(snapshot.local_recording_warning) << "\","
         << "\"camera\":{";

  if (!camera.checked) {
    stream << "\"ok\":false,\"error\":\"not_checked\"";
  } else if (camera.ok) {
    stream << "\"ok\":true,"
           << "\"info\":" << camera.info_json << ","
           << "\"mode\":\"" << JsonEscape(camera.mode) << "\","
           << "\"media\":{"
           << "\"remain_minutes\":\"" << JsonEscape(camera.media_remain_minutes) << "\","
           << "\"free\":\"" << JsonEscape(camera.media_free) << "\","
           << "\"total\":\"" << JsonEscape(camera.media_total) << "\","
           << "\"dcim_visible\":" << (camera.dcim_visible ? "true" : "false") << "}";
  } else {
    stream << "\"ok\":false,\"error\":\"" << JsonEscape(camera.error) << "\"";
  }

  stream << "}}";
  return stream.str();
}

std::string FormatHeartbeatLine(const StatusSnapshot& snapshot) {
  std::ostringstream stream;
  stream << "{\"type\":\"heartbeat\","
         << "\"state\":\"" << StateToString(snapshot.state) << "\","
         << "\"healthy\":" << (HealthyFlag(snapshot.state) != 0 ? "true" : "false") << ","
         << "\"current_file\":\"" << JsonEscape(snapshot.current_file) << "\","
         << "\"capture_path\":\"" << JsonEscape(snapshot.capture_path) << "\"}";
  return stream.str();
}

std::string FormatErrorLine(const std::string& reason) {
  return "{\"type\":\"error\",\"ok\":false,\"reason\":\"" + JsonEscape(reason) + "\"}";
}

std::string FormatOkLine(const std::string& type, const std::string& state) {
  return "{\"type\":\"" + JsonEscape(type) + "\",\"ok\":true,\"state\":\"" +
         JsonEscape(state) + "\"}";
}

std::string FormatPongLine() {
  return "{\"type\":\"pong\",\"ok\":true}";
}
