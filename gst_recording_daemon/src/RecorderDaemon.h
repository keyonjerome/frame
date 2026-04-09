#pragma once

#include "ControlServer.h"
#include "GstRecorder.h"
#include "Protocol.h"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

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
  void heartbeat_loop();
  void on_recorder_runtime_error(const std::string& error_token);
  void transition_to_error(const std::string& error_token, bool clear_current_file);
  StatusSnapshot get_status_snapshot() const;
  std::string generate_output_filename() const;

  const std::string device_path_;
  const int fps_;
  const std::string socket_path_;

  mutable std::mutex state_mutex_;
  std::condition_variable shutdown_cv_;

  RecorderState state_{RecorderState::IDLE};
  std::string current_output_file_;
  std::string last_error_;

  ControlServer control_server_;
  GstRecorder gst_recorder_;

  std::thread heartbeat_thread_;
  std::atomic<bool> shutdown_requested_{false};
};
