#include "RecorderDaemon.h"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <system_error>
#include <utility>

namespace {

constexpr char kDefaultError[] = "internal_error";
constexpr std::chrono::milliseconds kHeartbeatPeriod{100};

}  // namespace

RecorderDaemon::RecorderDaemon(std::string device_path,
                               std::string camera_host,
                               int fps,
                               std::string socket_path)
    : device_path_(std::move(device_path)),
      camera_host_(std::move(camera_host)),
      fps_(fps),
      socket_path_(std::move(socket_path)),
      control_server_(socket_path_),
      gst_recorder_(device_path_, camera_host_, fps_) {
  gst_recorder_.set_runtime_error_callback(
      [this](const std::string& error_token) { on_recorder_runtime_error(error_token); });
}

RecorderDaemon::~RecorderDaemon() {
  request_shutdown();

  if (heartbeat_thread_.joinable()) {
    heartbeat_thread_.join();
  }
}

bool RecorderDaemon::initialize() {
  return control_server_.start(
      [this](const std::string& line) { return handle_command(line); });
}

void RecorderDaemon::run() {
  heartbeat_thread_ = std::thread(&RecorderDaemon::heartbeat_loop, this);

  std::unique_lock<std::mutex> lock(state_mutex_);
  shutdown_cv_.wait(lock, [this]() { return shutdown_requested_.load(); });
  lock.unlock();

  if (heartbeat_thread_.joinable()) {
    heartbeat_thread_.join();
  }
}

void RecorderDaemon::request_shutdown() {
  const bool already_requested = shutdown_requested_.exchange(true);
  if (already_requested) {
    return;
  }

  shutdown_cv_.notify_all();
  control_server_.stop();

  bool should_stop_recording = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    should_stop_recording =
        (state_ == RecorderState::STARTING || state_ == RecorderState::RECORDING);
  }

  if (should_stop_recording) {
    std::string ignored_error;
    stop_recording(ignored_error);
  } else {
    gst_recorder_.shutdown();
  }
}

std::string RecorderDaemon::handle_command(const std::string& line) {
  const ParsedCommand command = ParseCommandLine(line);
  switch (command.type) {
    case CommandType::kEmpty:
      return "";
    case CommandType::kPing:
      return FormatPongLine();
    case CommandType::kStatus:
      return get_status_line();
    case CommandType::kStart: {
      std::string error;
      if (!start_recording(command.argument, error)) {
        return FormatErrorLine(error);
      }
      return FormatOkLine("start", "RECORDING");
    }
    case CommandType::kStop: {
      std::string error;
      if (!stop_recording(error)) {
        return FormatErrorLine(error);
      }
      return FormatOkLine("stop", "IDLE");
    }
    case CommandType::kInvalid:
      return FormatErrorLine(command.error_reason);
  }

  return FormatErrorLine(kDefaultError);
}

bool RecorderDaemon::start_recording(const std::string& output_dir, std::string& err) {
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ == RecorderState::STARTING || state_ == RecorderState::RECORDING ||
        state_ == RecorderState::STOPPING) {
      err = "already_recording";
      return false;
    }
  }

  if (shutdown_requested_.load()) {
    err = kDefaultError;
    return false;
  }

  gst_recorder_.shutdown();

  std::filesystem::path output_directory(output_dir);
  std::error_code fs_error;
  std::filesystem::create_directories(output_directory, fs_error);
  if (fs_error) {
    transition_to_error(kDefaultError, true);
    err = kDefaultError;
    return false;
  }

  const std::filesystem::path output_stem = output_directory / generate_output_filename();

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = RecorderState::STARTING;
    current_output_file_.clear();
    last_error_.clear();
  }

  std::string actual_output_file;
  if (!gst_recorder_.start(output_stem.lexically_normal().string(), actual_output_file, err)) {
    transition_to_error(err, true);
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = RecorderState::RECORDING;
    current_output_file_ = actual_output_file;
  }

  err.clear();
  return true;
}

bool RecorderDaemon::stop_recording(std::string& err) {
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ != RecorderState::STARTING && state_ != RecorderState::RECORDING) {
      err = "not_recording";
      return false;
    }

    state_ = RecorderState::STOPPING;
    last_error_.clear();
  }

  if (!gst_recorder_.stop(err)) {
    transition_to_error(err, true);
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = RecorderState::IDLE;
    current_output_file_.clear();
    last_error_.clear();
  }

  err.clear();
  return true;
}

std::string RecorderDaemon::get_status_line() const {
  const GstRecorder::CameraStatus recorder_camera = gst_recorder_.fetch_camera_status();
  CameraStatusSnapshot camera;
  camera.checked = recorder_camera.checked;
  camera.ok = recorder_camera.ok;
  camera.info_json = recorder_camera.info_json;
  camera.error = recorder_camera.error;
  camera.mode = recorder_camera.mode;
  camera.media_remain_minutes = recorder_camera.media_remain_minutes;
  camera.media_free = recorder_camera.media_free;
  camera.media_total = recorder_camera.media_total;
  camera.dcim_visible = recorder_camera.dcim_visible;
  return FormatStatusLine(get_status_snapshot(), camera);
}

void RecorderDaemon::heartbeat_loop() {
  while (!shutdown_requested_.load()) {
    std::this_thread::sleep_for(kHeartbeatPeriod);
    if (shutdown_requested_.load()) {
      break;
    }

    if (!control_server_.has_client()) {
      continue;
    }

    if (!control_server_.send_line_to_client(FormatHeartbeatLine(get_status_snapshot()))) {
      continue;
    }
  }
}

void RecorderDaemon::on_recorder_runtime_error(const std::string& error_token) {
  transition_to_error(error_token, false);
}

void RecorderDaemon::transition_to_error(const std::string& error_token,
                                         bool clear_current_file) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_ = RecorderState::ERROR;
  last_error_ = error_token.empty() ? kDefaultError : error_token;
  if (clear_current_file) {
    current_output_file_.clear();
  }
}

StatusSnapshot RecorderDaemon::get_status_snapshot() const {
  StatusSnapshot snapshot;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    snapshot.state = state_;
    snapshot.current_file = current_output_file_;
    snapshot.last_error = last_error_;
  }
  snapshot.capture_path = capture_path_to_string(gst_recorder_.active_capture_path());
  snapshot.local_recording_warning = gst_recorder_.local_recording_warning();
  return snapshot;
}

std::string RecorderDaemon::generate_output_filename() const {
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);

  std::tm local_time{};
  localtime_r(&now_time, &local_time);

  std::ostringstream stream;
  stream << "recording_" << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return stream.str();
}

std::string RecorderDaemon::capture_path_to_string(GstRecorder::CapturePath capture_path) {
  switch (capture_path) {
    case GstRecorder::CapturePath::kHdmi:
      return "hdmi";
    case GstRecorder::CapturePath::kSrt:
      return "srt";
    case GstRecorder::CapturePath::kNone:
      return "none";
  }
  return "none";
}
