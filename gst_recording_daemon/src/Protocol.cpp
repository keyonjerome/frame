#include "Protocol.h"

#include <algorithm>
#include <cctype>

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

std::string FormatStatusLine(const StatusSnapshot& snapshot) {
  return "STATUS " + StateToString(snapshot.state) + " " +
         std::to_string(HealthyFlag(snapshot.state)) + " " +
         DashIfEmpty(snapshot.current_file) + " " +
         DashIfEmpty(snapshot.last_error);
}

std::string FormatHeartbeatLine(const StatusSnapshot& snapshot) {
  return "HEARTBEAT " + StateToString(snapshot.state) + " " +
         std::to_string(HealthyFlag(snapshot.state)) + " " +
         DashIfEmpty(snapshot.current_file);
}

std::string FormatErrorLine(const std::string& reason) {
  return "ERR " + DashIfEmpty(reason);
}
