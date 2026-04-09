#include "GstRecorder.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <string>
#include <utility>

namespace {

constexpr char kDefaultError[] = "internal_error";
constexpr std::chrono::seconds kStopTimeout{2};

std::string ToLowerCopy(const std::string& value) {
  std::string lowered = value;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return lowered;
}

}  // namespace

GstRecorder::GstRecorder(std::string device_path, int fps)
    : device_path_(std::move(device_path)), fps_(fps) {}

GstRecorder::~GstRecorder() {
  shutdown();
}

void GstRecorder::set_runtime_error_callback(RuntimeErrorCallback callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  runtime_error_callback_ = std::move(callback);
}

bool GstRecorder::start(const std::string& output_file, std::string& error_token) {
  // Starting a new recording always begins from a clean slate. That keeps the
  // control flow easier to understand and avoids carrying any stale pipeline
  // objects across attempts.
  shutdown();

  GError* parse_error = nullptr;
  GstElement* pipeline = gst_parse_launch(build_pipeline_description().c_str(), &parse_error);
  if (pipeline == nullptr) {
    if (parse_error != nullptr) {
      g_error_free(parse_error);
    }
    error_token = "pipeline_build_failed";
    return false;
  }

  // We intentionally build the pipeline with names for the source and sink.
  // That lets us keep the launch string readable while still setting the
  // device path and output file from normal C++ code afterward.
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
    error_token = "pipeline_build_failed";
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
    error_token = "pipeline_build_failed";
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    pipeline_ = pipeline;
    bus_ = bus;
    stop_requested_ = false;
    eos_received_ = false;
    runtime_error_seen_ = false;
    runtime_error_token_.clear();
  }

  // Moving to PLAYING is the point where the pipeline really tries to acquire
  // the device and start data flow. This is where "device busy" style failures
  // usually surface.
  const GstStateChangeReturn state_result = gst_element_set_state(pipeline, GST_STATE_PLAYING);
  if (state_result == GST_STATE_CHANGE_FAILURE) {
    error_token = "pipeline_start_failed";

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

    shutdown();
    return false;
  }

  const std::uint64_t generation = bus_generation_.fetch_add(1) + 1;
  gst_object_ref(bus);
  bus_thread_ = std::thread(&GstRecorder::bus_loop, this, bus, generation);

  error_token.clear();
  return true;
}

bool GstRecorder::stop(std::string& error_token) {
  GstElement* pipeline = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pipeline_ == nullptr) {
      error_token = kDefaultError;
      return false;
    }

    stop_requested_ = true;
    eos_received_ = false;
    runtime_error_seen_ = false;
    runtime_error_token_.clear();
    pipeline = pipeline_;
  }

  // For MP4, EOS matters. `mp4mux` writes the final container metadata only
  // once the stream ends cleanly. Without EOS, the file can exist on disk but
  // still be incomplete or unplayable.
  gst_element_send_event(pipeline, gst_event_new_eos());

  std::unique_lock<std::mutex> lock(mutex_);
  eos_cv_.wait_for(lock, kStopTimeout, [this]() { return eos_received_ || runtime_error_seen_; });

  const bool saw_runtime_error = runtime_error_seen_;
  const std::string runtime_error_token = runtime_error_token_;
  lock.unlock();

  shutdown();

  if (saw_runtime_error) {
    error_token = runtime_error_token.empty() ? kDefaultError : runtime_error_token;
    return false;
  }

  error_token.clear();
  return true;
}

void GstRecorder::shutdown() {
  bus_generation_.fetch_add(1);

  GstElement* pipeline = nullptr;
  GstBus* bus = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pipeline = pipeline_;
    pipeline_ = nullptr;
    bus = bus_;
    bus_ = nullptr;
    stop_requested_ = false;
    eos_received_ = false;
    runtime_error_seen_ = false;
    runtime_error_token_.clear();
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

void GstRecorder::bus_loop(GstBus* bus, std::uint64_t generation) {
  while (generation == bus_generation_.load()) {
    GstMessage* message = gst_bus_timed_pop(bus, 100 * GST_MSECOND);
    if (message == nullptr) {
      continue;
    }

    const GstMessageType message_type = GST_MESSAGE_TYPE(message);
    if (message_type == GST_MESSAGE_EOS) {
      bool should_report_error = false;
      RuntimeErrorCallback callback;

      {
        std::lock_guard<std::mutex> lock(mutex_);
        eos_received_ = true;
        should_report_error = !stop_requested_;
        if (should_report_error) {
          runtime_error_seen_ = true;
          runtime_error_token_ = kDefaultError;
          callback = runtime_error_callback_;
        }
      }

      eos_cv_.notify_all();

      // Reaching EOS without an explicit STOP is treated as unexpected because
      // this daemon expects the capture stream to run until another process
      // tells it to stop.
      if (should_report_error && callback) {
        callback(kDefaultError);
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

      const std::string error_token = map_error_to_token(combined_message);
      RuntimeErrorCallback callback;
      GstElement* pipeline = nullptr;

      {
        std::lock_guard<std::mutex> lock(mutex_);
        runtime_error_seen_ = true;
        runtime_error_token_ = error_token;
        callback = runtime_error_callback_;
        pipeline = pipeline_;
      }

      if (pipeline != nullptr) {
        gst_element_set_state(pipeline, GST_STATE_NULL);
      }

      eos_cv_.notify_all();

      if (callback) {
        callback(error_token);
      }

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

std::string GstRecorder::build_pipeline_description() const {
  // This is intentionally written as a single human-readable launch string.
  // For a small daemon, `gst_parse_launch()` keeps the pipeline easier to read
  // than constructing every element and pad by hand.
  //
  // The pipeline assumes the USB capture device already produces H.264:
  //
  //   v4l2src
  //     Reads compressed frames from the Linux video device.
  //   video/x-h264,...
  //     Narrows the expected format to H.264 at 1080p and the requested FPS.
  //   queue
  //     Inserts a thread boundary so downstream muxing and file I/O do not
  //     directly stall the capture source.
  //   h264parse
  //     Normalizes the H.264 stream into a form the MP4 muxer expects.
  //   mp4mux
  //     Wraps the H.264 elementary stream into an MP4 container.
  //   filesink
  //     Writes the final bytes to disk.
  std::string description =
      "v4l2src name=source io-mode=mmap do-timestamp=true "
      "! video/x-h264,width=1920,height=1080,framerate=" +
      std::to_string(fps_) + "/1 "
      "! queue "
      "! h264parse "
      "! mp4mux name=mux "
      "! filesink name=sink sync=false";
  return description;
}

std::string GstRecorder::map_error_to_token(const std::string& message) {
  const std::string lowered = ToLowerCopy(message);
  if (lowered.find("device or resource busy") != std::string::npos ||
      lowered.find("resource busy") != std::string::npos ||
      lowered.find("busy") != std::string::npos) {
    return "device_busy";
  }
  return kDefaultError;
}
