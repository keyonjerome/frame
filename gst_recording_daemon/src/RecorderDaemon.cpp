#include "RecorderDaemon.h"

#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>
#include <vector>

namespace {

constexpr char kDefaultError[] = "internal_error";
constexpr std::chrono::milliseconds kPollInterval{100};
constexpr std::chrono::seconds kStopTimeout{2};

bool starts_with(const std::string& value, const std::string& prefix) {
  return value.rfind(prefix, 0) == 0;
}

std::string to_lower_copy(const std::string& value) {
  std::string lowered = value;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return lowered;
}

}  // namespace

RecorderDaemon::RecorderDaemon(std::string device_path, int fps, std::string socket_path)
    : device_path_(std::move(device_path)), fps_(fps), socket_path_(std::move(socket_path)) {}

RecorderDaemon::~RecorderDaemon() {
  request_shutdown();

  if (socket_thread_.joinable()) {
    socket_thread_.join();
  }
  if (heartbeat_thread_.joinable()) {
    heartbeat_thread_.join();
  }
  if (bus_thread_.joinable()) {
    bus_thread_.join();
  }

  clear_pipeline(true);
  cleanup_socket_file();
}

bool RecorderDaemon::initialize() {
  return setup_server_socket();
}

void RecorderDaemon::run() {
  socket_thread_ = std::thread(&RecorderDaemon::socket_loop, this);
  heartbeat_thread_ = std::thread(&RecorderDaemon::heartbeat_loop, this);

  std::unique_lock<std::mutex> lock(state_mutex_);
  shutdown_cv_.wait(lock, [this]() { return shutdown_requested_.load(); });
  lock.unlock();

  if (socket_thread_.joinable()) {
    socket_thread_.join();
  }
  if (heartbeat_thread_.joinable()) {
    heartbeat_thread_.join();
  }
  if (bus_thread_.joinable()) {
    bus_thread_.join();
  }
}

void RecorderDaemon::request_shutdown() {
  const bool already_requested = shutdown_requested_.exchange(true);
  if (!already_requested) {
    shutdown_cv_.notify_all();
  }

  int server_fd = server_fd_;
  if (server_fd != -1) {
    ::shutdown(server_fd, SHUT_RDWR);
    ::close(server_fd);
    server_fd_ = -1;
  }

  int client_fd = -1;
  {
    std::lock_guard<std::mutex> client_lock(client_mutex_);
    client_fd = active_client_fd_;
    active_client_fd_ = -1;
  }
  if (client_fd != -1) {
    ::shutdown(client_fd, SHUT_RDWR);
    ::close(client_fd);
  }

  std::string ignored_error;
  stop_recording(ignored_error);
  clear_pipeline(true);
  cleanup_socket_file();
}

std::string RecorderDaemon::handle_command(const std::string& line) {
  const std::string command = trim(line);
  if (command.empty()) {
    return "";
  }

  if (command == "PING") {
    return "PONG";
  }
  if (command == "STATUS") {
    return get_status_line();
  }
  if (command == "STOP") {
    std::string error;
    if (!stop_recording(error)) {
      return "ERR " + error;
    }
    return "OK STOPPING";
  }
  if (starts_with(command, "START")) {
    if (command.size() <= 6 || command[5] != ' ') {
      return "ERR internal_error";
    }

    const std::string output_dir = trim(command.substr(6));
    if (output_dir.empty()) {
      return "ERR internal_error";
    }

    std::string error;
    if (!start_recording(output_dir, error)) {
      return "ERR " + error;
    }
    return "OK STARTING";
  }

  return "ERR internal_error";
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

  clear_pipeline(true);

  std::filesystem::path output_directory(output_dir);
  std::error_code fs_error;
  std::filesystem::create_directories(output_directory, fs_error);
  if (fs_error) {
    transition_to_error(kDefaultError, true);
    err = kDefaultError;
    return false;
  }

  const std::filesystem::path output_file = output_directory / generate_output_filename();
  const std::string pipeline_description = build_pipeline_description();

  GError* parse_error = nullptr;
  GstElement* pipeline = gst_parse_launch(pipeline_description.c_str(), &parse_error);
  if (pipeline == nullptr) {
    if (parse_error != nullptr) {
      g_error_free(parse_error);
    }
    transition_to_error("pipeline_build_failed", true);
    err = "pipeline_build_failed";
    return false;
  }

  GstElement* source = gst_bin_get_by_name(GST_BIN(pipeline), "source");
  GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
  if (source == nullptr || sink == nullptr) {
    if (source != nullptr) {
      gst_object_unref(source);
    }
    if (sink != nullptr) {
      gst_object_unref(sink);
    }
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    transition_to_error("pipeline_build_failed", true);
    err = "pipeline_build_failed";
    return false;
  }

  g_object_set(G_OBJECT(source), "device", device_path_.c_str(), nullptr);
  g_object_set(G_OBJECT(sink), "location", output_file.c_str(), nullptr);
  gst_object_unref(source);
  gst_object_unref(sink);

  GstBus* bus = gst_element_get_bus(pipeline);
  if (bus == nullptr) {
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    transition_to_error("pipeline_build_failed", true);
    err = "pipeline_build_failed";
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = RecorderState::STARTING;
    current_output_file_ = output_file.lexically_normal().string();
    last_error_.clear();
    pipeline_ = pipeline;
    bus_ = bus;
    stop_requested_ = false;
    eos_received_ = false;
  }

  const GstStateChangeReturn state_result = gst_element_set_state(pipeline, GST_STATE_PLAYING);
  if (state_result == GST_STATE_CHANGE_FAILURE) {
    std::string error_token = "pipeline_start_failed";
    GstMessage* error_message =
        gst_bus_timed_pop_filtered(bus, 200 * GST_MSECOND, GST_MESSAGE_ERROR);
    if (error_message != nullptr) {
      GError* gst_error = nullptr;
      gchar* debug_info = nullptr;
      gst_message_parse_error(error_message, &gst_error, &debug_info);
      std::string combined_message;
      if (gst_error != nullptr && gst_error->message != nullptr) {
        combined_message += gst_error->message;
      }
      if (debug_info != nullptr) {
        if (!combined_message.empty()) {
          combined_message += " ";
        }
        combined_message += debug_info;
      }
      const std::string mapped_token = map_error_to_token(combined_message);
      if (mapped_token == "device_busy") {
        error_token = mapped_token;
      }
      if (gst_error != nullptr) {
        g_error_free(gst_error);
      }
      g_free(debug_info);
      gst_message_unref(error_message);
    }

    clear_pipeline(true);
    transition_to_error(error_token, true);
    err = error_token;
    return false;
  }

  const std::uint64_t generation = bus_generation_.fetch_add(1) + 1;
  gst_object_ref(bus);
  bus_thread_ = std::thread(&RecorderDaemon::bus_loop, this, bus, generation);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = RecorderState::RECORDING;
  }

  err.clear();
  return true;
}

bool RecorderDaemon::stop_recording(std::string& err) {
  GstElement* pipeline = nullptr;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ != RecorderState::STARTING && state_ != RecorderState::RECORDING) {
      err = "not_recording";
      return false;
    }

    state_ = RecorderState::STOPPING;
    stop_requested_ = true;
    eos_received_ = false;
    last_error_.clear();
    pipeline = pipeline_;
  }

  if (pipeline == nullptr) {
    transition_to_error(kDefaultError, true);
    err = kDefaultError;
    return false;
  }

  gst_element_send_event(pipeline, gst_event_new_eos());

  std::unique_lock<std::mutex> lock(state_mutex_);
  eos_cv_.wait_for(lock, kStopTimeout, [this]() { return eos_received_ || state_ == RecorderState::ERROR; });
  lock.unlock();

  clear_pipeline(true);

  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (state_ != RecorderState::ERROR) {
      state_ = RecorderState::IDLE;
      last_error_.clear();
    }
  }

  err.clear();
  return true;
}

std::string RecorderDaemon::get_status_line() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return "STATUS " + state_to_string(state_) + " " + std::to_string(healthy_flag(state_)) + " " +
         dash_if_empty(current_output_file_) + " " + dash_if_empty(last_error_);
}

std::string RecorderDaemon::get_heartbeat_line() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return "HEARTBEAT " + state_to_string(state_) + " " + std::to_string(healthy_flag(state_)) +
         " " + dash_if_empty(current_output_file_);
}

std::string RecorderDaemon::generate_output_filename() const {
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);

  std::tm local_time{};
  localtime_r(&now_time, &local_time);

  std::ostringstream stream;
  stream << "recording_" << std::put_time(&local_time, "%Y%m%d_%H%M%S") << ".mp4";
  return stream.str();
}

std::string RecorderDaemon::build_pipeline_description() const {
  std::ostringstream stream;
  stream << "v4l2src name=source io-mode=mmap do-timestamp=true "
         << "! video/x-h264,width=1920,height=1080,framerate=" << fps_ << "/1 "
         << "! queue "
         << "! h264parse "
         << "! mp4mux name=mux "
         << "! filesink name=sink sync=false";
  return stream.str();
}

void RecorderDaemon::socket_loop() {
  std::string pending_input;

  while (!shutdown_requested_.load()) {
    std::vector<pollfd> poll_fds;
    poll_fds.push_back({server_fd_, POLLIN, 0});

    const int client_fd = get_active_client_fd();
    if (client_fd != -1) {
      poll_fds.push_back({client_fd, POLLIN | POLLERR | POLLHUP, 0});
    }

    const int poll_result = ::poll(poll_fds.data(), poll_fds.size(), static_cast<int>(kPollInterval.count()));
    if (poll_result < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (!shutdown_requested_.load()) {
        std::cerr << "socket poll failed: " << std::strerror(errno) << std::endl;
      }
      break;
    }
    if (poll_result == 0) {
      continue;
    }

    if ((poll_fds[0].revents & POLLIN) != 0) {
      const int new_client_fd = ::accept(server_fd_, nullptr, nullptr);
      if (new_client_fd >= 0) {
        bool reject_client = false;
        {
          std::lock_guard<std::mutex> lock(client_mutex_);
          if (active_client_fd_ != -1) {
            reject_client = true;
          } else {
            active_client_fd_ = new_client_fd;
            pending_input.clear();
          }
        }

        if (reject_client) {
          ::close(new_client_fd);
        }
      }
    }

    if (client_fd == -1 || poll_fds.size() < 2) {
      continue;
    }

    const short client_events = poll_fds[1].revents;
    if ((client_events & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      disconnect_client_if(client_fd);
      pending_input.clear();
      continue;
    }
    if ((client_events & POLLIN) == 0) {
      continue;
    }

    char buffer[1024];
    const ssize_t bytes_read = ::read(client_fd, buffer, sizeof(buffer));
    if (bytes_read <= 0) {
      disconnect_client_if(client_fd);
      pending_input.clear();
      continue;
    }

    pending_input.append(buffer, static_cast<std::size_t>(bytes_read));
    std::size_t newline_pos = pending_input.find('\n');
    while (newline_pos != std::string::npos) {
      std::string line = pending_input.substr(0, newline_pos);
      pending_input.erase(0, newline_pos + 1);
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }

      const std::string response = handle_command(line);
      if (!response.empty() && !send_line_to_active_client(response)) {
        disconnect_client_if(client_fd);
        pending_input.clear();
        break;
      }

      newline_pos = pending_input.find('\n');
    }
  }
}

void RecorderDaemon::heartbeat_loop() {
  while (!shutdown_requested_.load()) {
    std::this_thread::sleep_for(kPollInterval);
    if (shutdown_requested_.load()) {
      break;
    }

    const int client_fd = get_active_client_fd();
    if (client_fd == -1) {
      continue;
    }

    if (!send_line_to_active_client(get_heartbeat_line())) {
      disconnect_client_if(client_fd);
    }
  }
}

void RecorderDaemon::bus_loop(GstBus* bus, std::uint64_t generation) {
  while (generation == bus_generation_.load()) {
    GstMessage* message = gst_bus_timed_pop(bus, 100 * GST_MSECOND);
    if (message == nullptr) {
      continue;
    }

    const GstMessageType message_type = GST_MESSAGE_TYPE(message);
    if (message_type == GST_MESSAGE_EOS) {
      bool stop_requested = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        stop_requested = stop_requested_;
        eos_received_ = true;
      }
      eos_cv_.notify_all();

      if (!stop_requested) {
        transition_to_error(kDefaultError, false);
      }

      gst_message_unref(message);
      break;
    }

    if (message_type == GST_MESSAGE_ERROR) {
      GError* gst_error = nullptr;
      gchar* debug_info = nullptr;
      gst_message_parse_error(message, &gst_error, &debug_info);

      std::string combined_message;
      if (gst_error != nullptr && gst_error->message != nullptr) {
        combined_message += gst_error->message;
      }
      if (debug_info != nullptr) {
        if (!combined_message.empty()) {
          combined_message += " ";
        }
        combined_message += debug_info;
      }

      transition_to_error(map_error_to_token(combined_message), false);

      if (gst_error != nullptr) {
        g_error_free(gst_error);
      }
      g_free(debug_info);
      gst_message_unref(message);
      break;
    }

    gst_message_unref(message);
  }

  gst_object_unref(bus);
}

bool RecorderDaemon::setup_server_socket() {
  if (socket_path_.size() >= sizeof(sockaddr_un{}.sun_path)) {
    std::cerr << "socket path is too long: " << socket_path_ << std::endl;
    return false;
  }

  cleanup_socket_file();

  server_fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (server_fd_ == -1) {
    std::cerr << "failed to create Unix socket: " << std::strerror(errno) << std::endl;
    return false;
  }

  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::snprintf(address.sun_path, sizeof(address.sun_path), "%s", socket_path_.c_str());

  if (::bind(server_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == -1) {
    std::cerr << "failed to bind Unix socket: " << std::strerror(errno) << std::endl;
    ::close(server_fd_);
    server_fd_ = -1;
    return false;
  }

  if (::listen(server_fd_, 4) == -1) {
    std::cerr << "failed to listen on Unix socket: " << std::strerror(errno) << std::endl;
    ::close(server_fd_);
    server_fd_ = -1;
    cleanup_socket_file();
    return false;
  }

  return true;
}

void RecorderDaemon::cleanup_socket_file() {
  if (!socket_path_.empty()) {
    ::unlink(socket_path_.c_str());
  }
}

void RecorderDaemon::disconnect_client_if(int fd) {
  int fd_to_close = -1;
  {
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (active_client_fd_ == fd) {
      fd_to_close = active_client_fd_;
      active_client_fd_ = -1;
    }
  }

  if (fd_to_close != -1) {
    ::shutdown(fd_to_close, SHUT_RDWR);
    ::close(fd_to_close);
  }
}

int RecorderDaemon::get_active_client_fd() const {
  std::lock_guard<std::mutex> lock(client_mutex_);
  return active_client_fd_;
}

bool RecorderDaemon::send_line_to_active_client(const std::string& line) {
  std::lock_guard<std::mutex> lock(client_mutex_);
  if (active_client_fd_ == -1) {
    return false;
  }

  std::string payload = line;
  payload.push_back('\n');

  const char* data = payload.data();
  std::size_t remaining = payload.size();
  while (remaining > 0) {
    const ssize_t sent =
        ::send(active_client_fd_, data, remaining, MSG_NOSIGNAL);
    if (sent <= 0) {
      return false;
    }
    data += sent;
    remaining -= static_cast<std::size_t>(sent);
  }

  return true;
}

void RecorderDaemon::clear_pipeline(bool clear_current_file) {
  bus_generation_.fetch_add(1);

  GstElement* pipeline = nullptr;
  GstBus* bus = nullptr;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    pipeline = pipeline_;
    pipeline_ = nullptr;
    bus = bus_;
    bus_ = nullptr;
    stop_requested_ = false;
    eos_received_ = false;
    if (clear_current_file) {
      current_output_file_.clear();
    }
  }

  if (pipeline != nullptr) {
    gst_element_set_state(pipeline, GST_STATE_NULL);
  }

  if (bus_thread_.joinable()) {
    bus_thread_.join();
  }

  if (bus != nullptr) {
    gst_object_unref(bus);
  }
  if (pipeline != nullptr) {
    gst_object_unref(pipeline);
  }
}

void RecorderDaemon::transition_to_error(const std::string& error_token, bool clear_current_file) {
  GstElement* pipeline = nullptr;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = RecorderState::ERROR;
    last_error_ = error_token;
    stop_requested_ = false;
    eos_received_ = false;
    if (clear_current_file) {
      current_output_file_.clear();
    }
    pipeline = pipeline_;
  }

  if (pipeline != nullptr) {
    gst_element_set_state(pipeline, GST_STATE_NULL);
  }

  eos_cv_.notify_all();
}

std::string RecorderDaemon::trim(const std::string& value) {
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

std::string RecorderDaemon::state_to_string(RecorderState state) {
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

std::string RecorderDaemon::dash_if_empty(const std::string& value) {
  return value.empty() ? "-" : value;
}

int RecorderDaemon::healthy_flag(RecorderState state) {
  return state == RecorderState::ERROR ? 0 : 1;
}

std::string RecorderDaemon::map_error_to_token(const std::string& message) {
  const std::string lowered = to_lower_copy(message);
  if (lowered.find("device or resource busy") != std::string::npos ||
      lowered.find("resource busy") != std::string::npos ||
      lowered.find("busy") != std::string::npos) {
    return "device_busy";
  }
  return kDefaultError;
}
