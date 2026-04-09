#pragma once

#include <gst/gst.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>

enum class RecorderState {
  IDLE,
  STARTING,
  RECORDING,
  STOPPING,
  ERROR
};

class RecorderDaemon {
 public:
  RecorderDaemon(std::string device_path, int fps, std::string socket_path);
  ~RecorderDaemon();

  bool initialize();
  void run();
  void request_shutdown();

  std::string handle_command(const std::string& line);
  bool start_recording(const std::string& output_dir, std::string& err);
  bool stop_recording(std::string& err);
  std::string get_status_line() const;

 private:
  std::string get_heartbeat_line() const;
  std::string generate_output_filename() const;
  std::string build_pipeline_description() const;

  void socket_loop();
  void heartbeat_loop();
  void bus_loop(GstBus* bus, std::uint64_t generation);

  bool setup_server_socket();
  void cleanup_socket_file();
  void disconnect_client_if(int fd);
  int get_active_client_fd() const;
  bool send_line_to_active_client(const std::string& line);

  void clear_pipeline(bool clear_current_file);
  void transition_to_error(const std::string& error_token, bool clear_current_file);

  static std::string trim(const std::string& value);
  static std::string state_to_string(RecorderState state);
  static std::string dash_if_empty(const std::string& value);
  static int healthy_flag(RecorderState state);
  static std::string map_error_to_token(const std::string& message);

  const std::string device_path_;
  const int fps_;
  const std::string socket_path_;

  mutable std::mutex state_mutex_;
  mutable std::mutex client_mutex_;
  std::condition_variable shutdown_cv_;
  std::condition_variable eos_cv_;

  RecorderState state_{RecorderState::IDLE};
  std::string current_output_file_;
  std::string last_error_;
  GstElement* pipeline_{nullptr};
  GstBus* bus_{nullptr};
  bool stop_requested_{false};
  bool eos_received_{false};

  int server_fd_{-1};
  int active_client_fd_{-1};

  std::thread socket_thread_;
  std::thread heartbeat_thread_;
  std::thread bus_thread_;

  std::atomic<bool> shutdown_requested_{false};
  std::atomic<std::uint64_t> bus_generation_{0};
};
