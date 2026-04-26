#pragma once

#include <gst/gst.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

class GstRecorder {
 public:
  using RuntimeErrorCallback = std::function<void(const std::string&)>;

  enum class CapturePath {
    kNone,
    kHdmi,
    kSrt,
  };

  struct CameraStatus {
    bool checked{false};
    bool ok{false};
    std::string info_json;
    std::string error;
    std::string mode;
    std::string local_recording_warning;
    std::string media_remain_minutes;
    std::string media_free;
    std::string media_total;
    bool dcim_visible{false};
  };

  GstRecorder(std::string hdmi_device_path, std::string camera_host, int fps);
  ~GstRecorder();

  void set_runtime_error_callback(RuntimeErrorCallback callback);

  bool start(const std::string& output_stem,
             std::string& actual_output_file,
             std::string& error_token);
  bool stop(std::string& error_token);
  void shutdown();
  bool fetch_camera_info(std::string& info_json, std::string& error_token) const;
  CameraStatus fetch_camera_status() const;
  CapturePath active_capture_path() const;
  std::string local_recording_warning() const;

 private:
  void bus_loop(GstBus* bus, std::uint64_t generation);
  bool start_pipeline(const std::string& output_file,
                      CapturePath capture_path,
                      std::string& error_token);
  std::string build_pipeline_description(CapturePath capture_path) const;
  bool query_stream_profile(std::string& profile_json, std::string& error_token) const;
  bool start_camera_srt(std::string& error_token) const;
  void stop_camera_srt() const;
  bool start_camera_recording(std::string& warning_token);
  void stop_camera_recording();
  bool poll_camera_mode(const std::string& expected_mode,
                        std::chrono::milliseconds timeout,
                        std::string& mode,
                        std::string& error_token) const;
  bool http_get(const std::string& path,
                std::string& body,
                std::string& error_token,
                int* status_code = nullptr,
                int timeout_seconds = 2) const;
  static std::string map_error_to_token(const std::string& message);

  const std::string hdmi_device_path_;
  const std::string camera_host_;
  const int fps_;

  mutable std::mutex mutex_;
  std::condition_variable eos_cv_;

  GstElement* pipeline_{nullptr};
  GstBus* bus_{nullptr};
  bool stop_requested_{false};
  bool eos_received_{false};
  bool runtime_error_seen_{false};
  std::string runtime_error_token_;
  CapturePath active_capture_path_{CapturePath::kNone};
  std::string local_recording_warning_;

  RuntimeErrorCallback runtime_error_callback_;

  std::thread bus_thread_;
  std::atomic<std::uint64_t> bus_generation_{0};
};
