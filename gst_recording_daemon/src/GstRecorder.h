#pragma once

#include <gst/gst.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

class GstRecorder {
 public:
  using RuntimeErrorCallback = std::function<void(const std::string&)>;

  GstRecorder(std::string device_path, int fps);
  ~GstRecorder();

  void set_runtime_error_callback(RuntimeErrorCallback callback);

  bool start(const std::string& output_file, std::string& error_token);
  bool stop(std::string& error_token);
  void shutdown();

 private:
  void bus_loop(GstBus* bus, std::uint64_t generation);
  std::string build_pipeline_description() const;
  static std::string map_error_to_token(const std::string& message);

  const std::string device_path_;
  const int fps_;

  mutable std::mutex mutex_;
  std::condition_variable eos_cv_;

  GstElement* pipeline_{nullptr};
  GstBus* bus_{nullptr};
  bool stop_requested_{false};
  bool eos_received_{false};
  bool runtime_error_seen_{false};
  std::string runtime_error_token_;

  RuntimeErrorCallback runtime_error_callback_;

  std::thread bus_thread_;
  std::atomic<std::uint64_t> bus_generation_{0};
};
