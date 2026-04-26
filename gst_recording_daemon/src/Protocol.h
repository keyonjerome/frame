#pragma once

#include <string>

// The daemon intentionally exposes only a tiny set of lifecycle states.
// Keeping the enum small makes the wire protocol and logs easy to inspect.
enum class RecorderState {
  IDLE,
  STARTING,
  RECORDING,
  STOPPING,
  ERROR
};

// Commands are parsed from a simple ASCII line protocol. We represent the result
// as a tiny tagged structure so higher-level code does not need to repeat string
// comparisons or ad-hoc tokenization.
enum class CommandType {
  kEmpty,
  kPing,
  kStatus,
  kStart,
  kStop,
  kInvalid
};

struct ParsedCommand {
  CommandType type{CommandType::kInvalid};
  std::string argument;
  std::string error_reason{"internal_error"};
};

// This structure carries the daemon status in one place so both STATUS and
// HEARTBEAT formatting can reuse the same data.
struct StatusSnapshot {
  RecorderState state{RecorderState::IDLE};
  std::string current_file;
  std::string last_error;
  std::string capture_path{"none"};
  std::string local_recording_warning;
};

struct CameraStatusSnapshot {
  bool checked{false};
  bool ok{false};
  std::string info_json;
  std::string error;
  std::string mode;
  std::string media_remain_minutes;
  std::string media_free;
  std::string media_total;
  bool dcim_visible{false};
};

std::string TrimAsciiWhitespace(const std::string& value);
ParsedCommand ParseCommandLine(const std::string& line);

std::string StateToString(RecorderState state);
int HealthyFlag(RecorderState state);
std::string DashIfEmpty(const std::string& value);

std::string JsonEscape(const std::string& value);
std::string FormatStatusLine(const StatusSnapshot& snapshot,
                             const CameraStatusSnapshot& camera);
std::string FormatHeartbeatLine(const StatusSnapshot& snapshot);
std::string FormatErrorLine(const std::string& reason);
std::string FormatOkLine(const std::string& type, const std::string& state);
std::string FormatPongLine();
