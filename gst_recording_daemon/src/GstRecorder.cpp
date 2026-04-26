#include "GstRecorder.h"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <netdb.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <exception>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

constexpr char kDefaultError[] = "internal_error";
constexpr char kCameraUnreachable[] = "camera_unreachable";
constexpr char kCameraStreamMismatch[] = "camera_stream_mismatch";
constexpr char kZcamSrtStreamIndex[] = "stream1";
constexpr std::chrono::seconds kStopTimeout{2};
constexpr int kRecordHttpTimeoutSeconds = 20;
constexpr char kHostSrtAddress[] = "10.98.32.2";
constexpr int kSrtPort = 7001;
constexpr int kTargetHdmiWidth = 3840;
constexpr int kTargetHdmiHeight = 2160;
constexpr double kTargetHdmiFps = 30.0;
constexpr std::chrono::milliseconds kCameraRecordPollTimeout{5000};
constexpr std::chrono::milliseconds kCameraRecordPollPeriod{250};

struct HdmiCaptureFormat {
  std::uint32_t fourcc{0};
  std::uint32_t width{1920};
  std::uint32_t height{1080};
  std::uint32_t fps_numerator{1};
  std::uint32_t fps_denominator{30};
};

bool IoctlRetry(int fd, unsigned long request, void* arg) {
  while (true) {
    if (::ioctl(fd, request, arg) == 0) {
      return true;
    }
    if (errno != EINTR) {
      return false;
    }
  }
}

std::string ToLowerCopy(const std::string& value) {
  std::string lowered = value;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return lowered;
}

const char* StateChangeReturnToString(GstStateChangeReturn value) {
  switch (value) {
    case GST_STATE_CHANGE_FAILURE:
      return "FAILURE";
    case GST_STATE_CHANGE_SUCCESS:
      return "SUCCESS";
    case GST_STATE_CHANGE_ASYNC:
      return "ASYNC";
    case GST_STATE_CHANGE_NO_PREROLL:
      return "NO_PREROLL";
  }
  return "UNKNOWN";
}

std::string CombineGstErrorMessage(const GError* gst_error, const gchar* debug_info) {
  std::ostringstream stream;
  if (gst_error != nullptr && gst_error->message != nullptr) {
    stream << gst_error->message;
  }
  if (debug_info != nullptr && debug_info[0] != '\0') {
    if (stream.tellp() > 0) {
      stream << " | ";
    }
    stream << debug_info;
  }
  return stream.str();
}

std::string TrimCopy(const std::string& value) {
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

bool LooksLikeJsonObject(const std::string& body) {
  const std::string trimmed = TrimCopy(body);
  return trimmed.size() >= 2 && trimmed.front() == '{' && trimmed.back() == '}';
}

std::string StripHttpSchemeAndPath(const std::string& value) {
  std::string host = value;
  const std::string http_prefix = "http://";
  const std::string https_prefix = "https://";
  if (host.rfind(http_prefix, 0) == 0) {
    host.erase(0, http_prefix.size());
  } else if (host.rfind(https_prefix, 0) == 0) {
    host.erase(0, https_prefix.size());
  }

  const std::size_t slash = host.find('/');
  if (slash != std::string::npos) {
    host.erase(slash);
  }
  return host;
}

bool BodyContainsEncoder(const std::string& body, const std::string& encoder) {
  std::string compact;
  compact.reserve(body.size());
  for (const unsigned char ch : body) {
    if (std::isspace(ch) == 0) {
      compact.push_back(static_cast<char>(std::tolower(ch)));
    }
  }
  return compact.find("\"encodertype\":\"" + ToLowerCopy(encoder) + "\"") != std::string::npos;
}

bool BodyContainsFailureCode(const std::string& body) {
  std::string compact;
  compact.reserve(body.size());
  for (const unsigned char ch : body) {
    if (std::isspace(ch) == 0) {
      compact.push_back(static_cast<char>(ch));
    }
  }
  return compact.find("\"code\":-1") != std::string::npos ||
         compact.find("\"code\":1") != std::string::npos;
}

std::string ExtractJsonStringValue(const std::string& body, const std::string& key) {
  const std::string compact = ToLowerCopy(body);
  const std::string needle = "\"" + ToLowerCopy(key) + "\":\"";
  const std::size_t key_pos = compact.find(needle);
  if (key_pos == std::string::npos) {
    return "";
  }
  const std::size_t value_begin = key_pos + needle.size();
  const std::size_t value_end = compact.find('"', value_begin);
  if (value_end == std::string::npos) {
    return "";
  }
  return compact.substr(value_begin, value_end - value_begin);
}

std::size_t ParseContentLength(const std::string& headers) {
  const std::string lowered = ToLowerCopy(headers);
  const std::string key = "content-length:";
  const std::size_t key_pos = lowered.find(key);
  if (key_pos == std::string::npos) {
    return 0;
  }

  const std::size_t value_begin = key_pos + key.size();
  const std::size_t value_end = lowered.find("\r\n", value_begin);
  const std::string value = TrimCopy(headers.substr(
      value_begin,
      value_end == std::string::npos ? std::string::npos : value_end - value_begin));
  try {
    return static_cast<std::size_t>(std::stoull(value));
  } catch (const std::exception&) {
    return 0;
  }
}

std::string UrlEncode(const std::string& value) {
  std::ostringstream stream;
  const char* digits = "0123456789ABCDEF";
  for (const unsigned char ch : value) {
    if ((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') ||
        (ch >= '0' && ch <= '9') || ch == '-' || ch == '_' || ch == '.' || ch == '~') {
      stream << static_cast<char>(ch);
    } else {
      stream << '%' << digits[(ch >> 4) & 0x0f] << digits[ch & 0x0f];
    }
  }
  return stream.str();
}

std::string BuildSrtPushUrl() {
  return "srt://" + std::string(kHostSrtAddress) + ":" + std::to_string(kSrtPort);
}

std::string FourccToString(std::uint32_t fourcc) {
  char value[5] = {
      static_cast<char>(fourcc & 0xff),
      static_cast<char>((fourcc >> 8) & 0xff),
      static_cast<char>((fourcc >> 16) & 0xff),
      static_cast<char>((fourcc >> 24) & 0xff),
      '\0',
  };
  return value;
}

int FormatPriority(std::uint32_t fourcc) {
  switch (fourcc) {
    case V4L2_PIX_FMT_MJPEG:
    case V4L2_PIX_FMT_JPEG:
      return 0;
    case V4L2_PIX_FMT_H264:
      return 1;
    case V4L2_PIX_FMT_YUYV:
      return 2;
  }
  return 100;
}

bool IsSupportedHdmiFourcc(std::uint32_t fourcc) {
  return FormatPriority(fourcc) < 100;
}

double FpsValue(const HdmiCaptureFormat& format) {
  if (format.fps_numerator == 0) {
    return 0.0;
  }
  return static_cast<double>(format.fps_denominator) /
         static_cast<double>(format.fps_numerator);
}

double CaptureFormatScore(const HdmiCaptureFormat& format) {
  const double target_pixels =
      static_cast<double>(kTargetHdmiWidth) * static_cast<double>(kTargetHdmiHeight);
  const double pixels = static_cast<double>(format.width) * static_cast<double>(format.height);
  const double pixel_score = std::abs(target_pixels - pixels) / target_pixels;
  const double aspect =
      static_cast<double>(format.width) / std::max(1.0, static_cast<double>(format.height));
  const double target_aspect =
      static_cast<double>(kTargetHdmiWidth) / static_cast<double>(kTargetHdmiHeight);
  const double aspect_score = std::abs(target_aspect - aspect);
  const double fps_score = std::abs(kTargetHdmiFps - FpsValue(format)) / kTargetHdmiFps;
  return pixel_score * 1000.0 + aspect_score * 100.0 + fps_score +
         static_cast<double>(FormatPriority(format.fourcc)) * 0.01;
}

void AddFrameIntervalCandidates(int fd,
                                std::uint32_t fourcc,
                                std::uint32_t width,
                                std::uint32_t height,
                                std::vector<HdmiCaptureFormat>& candidates) {
  bool found_interval = false;
  for (std::uint32_t interval_index = 0;; ++interval_index) {
    v4l2_frmivalenum interval{};
    interval.index = interval_index;
    interval.pixel_format = fourcc;
    interval.width = width;
    interval.height = height;
    if (!IoctlRetry(fd, VIDIOC_ENUM_FRAMEINTERVALS, &interval)) {
      break;
    }

    if (interval.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
      candidates.push_back({fourcc,
                            width,
                            height,
                            interval.discrete.numerator,
                            interval.discrete.denominator});
      found_interval = true;
      continue;
    }

    const v4l2_fract& min = interval.stepwise.min;
    const v4l2_fract& max = interval.stepwise.max;
    const double min_fps =
        min.numerator == 0 ? 0.0 : static_cast<double>(min.denominator) / min.numerator;
    const double max_fps =
        max.numerator == 0 ? 0.0 : static_cast<double>(max.denominator) / max.numerator;
    const bool target_in_range =
        kTargetHdmiFps >= std::min(min_fps, max_fps) &&
        kTargetHdmiFps <= std::max(min_fps, max_fps);
    if (target_in_range) {
      candidates.push_back({fourcc, width, height, 1, 30});
    } else if (std::abs(min_fps - kTargetHdmiFps) < std::abs(max_fps - kTargetHdmiFps)) {
      candidates.push_back({fourcc, width, height, min.numerator, min.denominator});
    } else {
      candidates.push_back({fourcc, width, height, max.numerator, max.denominator});
    }
    found_interval = true;
    break;
  }

  if (!found_interval) {
    candidates.push_back({fourcc, width, height, 1, 30});
  }
}

HdmiCaptureFormat SelectBestHdmiCaptureFormat(const std::string& device_path, int fallback_fps) {
  std::vector<HdmiCaptureFormat> candidates;
  const int fd = ::open(device_path.c_str(), O_RDWR | O_NONBLOCK);
  if (fd == -1) {
    std::cerr << "[gst_recording_daemon] Could not open HDMI capture device for caps probe"
              << " device=" << device_path << " errno=" << std::strerror(errno)
              << "; using fallback 1080p MJPEG caps" << std::endl;
    return {V4L2_PIX_FMT_MJPEG, 1920, 1080, 1, static_cast<std::uint32_t>(fallback_fps)};
  }

  for (std::uint32_t format_index = 0;; ++format_index) {
    v4l2_fmtdesc format{};
    format.index = format_index;
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (!IoctlRetry(fd, VIDIOC_ENUM_FMT, &format)) {
      break;
    }
    if (!IsSupportedHdmiFourcc(format.pixelformat)) {
      continue;
    }

    for (std::uint32_t size_index = 0;; ++size_index) {
      v4l2_frmsizeenum size{};
      size.index = size_index;
      size.pixel_format = format.pixelformat;
      if (!IoctlRetry(fd, VIDIOC_ENUM_FRAMESIZES, &size)) {
        break;
      }

      if (size.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
        AddFrameIntervalCandidates(fd,
                                   format.pixelformat,
                                   size.discrete.width,
                                   size.discrete.height,
                                   candidates);
        continue;
      }

      const std::uint32_t width =
          std::min<std::uint32_t>(std::max<std::uint32_t>(kTargetHdmiWidth,
                                                          size.stepwise.min_width),
                                  size.stepwise.max_width);
      const std::uint32_t height =
          std::min<std::uint32_t>(std::max<std::uint32_t>(kTargetHdmiHeight,
                                                          size.stepwise.min_height),
                                  size.stepwise.max_height);
      AddFrameIntervalCandidates(fd, format.pixelformat, width, height, candidates);
      break;
    }
  }

  ::close(fd);

  if (candidates.empty()) {
    std::cerr << "[gst_recording_daemon] HDMI capture caps probe found no supported formats"
              << " device=" << device_path << "; using fallback 1080p MJPEG caps"
              << std::endl;
    return {V4L2_PIX_FMT_MJPEG, 1920, 1080, 1, static_cast<std::uint32_t>(fallback_fps)};
  }

  const auto best = std::min_element(
      candidates.begin(), candidates.end(), [](const auto& lhs, const auto& rhs) {
        return CaptureFormatScore(lhs) < CaptureFormatScore(rhs);
      });

  std::cerr << "[gst_recording_daemon] Selected HDMI capture format"
            << " fourcc=" << FourccToString(best->fourcc)
            << " width=" << best->width
            << " height=" << best->height
            << " fps=" << FpsValue(*best) << std::endl;
  return *best;
}

std::string CapturePathToString(GstRecorder::CapturePath capture_path) {
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

}  // namespace

GstRecorder::GstRecorder(std::string hdmi_device_path, std::string camera_host, int fps)
    : hdmi_device_path_(std::move(hdmi_device_path)),
      camera_host_(std::move(camera_host)),
      fps_(fps) {}

GstRecorder::~GstRecorder() {
  shutdown();
}

void GstRecorder::set_runtime_error_callback(RuntimeErrorCallback callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  runtime_error_callback_ = std::move(callback);
}

bool GstRecorder::start(const std::string& output_stem,
                        std::string& actual_output_file,
                        std::string& error_token) {
  // Starting a new recording always begins from a clean slate. That keeps the
  // control flow easier to understand and avoids carrying any stale pipeline
  // objects across attempts.
  shutdown();
  actual_output_file.clear();

  std::string camera_info;
  if (!fetch_camera_info(camera_info, error_token)) {
    std::cerr << "[gst_recording_daemon] Camera /info preflight failed token="
              << (error_token.empty() ? kCameraUnreachable : error_token) << std::endl;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      local_recording_warning_ = kCameraUnreachable;
    }
  } else {
    std::cerr << "[gst_recording_daemon] Camera /info OK body=" << camera_info << std::endl;
  }

  std::string ignored_body;
  std::string ignored_error;
  int ignored_status = 0;
  http_get("/ctrl/mode?action=query", ignored_body, ignored_error, &ignored_status);
  http_get("/ctrl/rec?action=remain", ignored_body, ignored_error, &ignored_status);
  http_get("/ctrl/card?action=query_free", ignored_body, ignored_error, &ignored_status);
  http_get("/ctrl/card?action=query_total", ignored_body, ignored_error, &ignored_status);
  http_get("/DCIM/", ignored_body, ignored_error, &ignored_status);

  const std::string hdmi_output_file = output_stem + ".mp4";
  std::string hdmi_error;
  if (start_pipeline(hdmi_output_file, CapturePath::kHdmi, hdmi_error)) {
    actual_output_file = hdmi_output_file;
    std::string recording_warning;
    if (!start_camera_recording(recording_warning)) {
      std::lock_guard<std::mutex> lock(mutex_);
      local_recording_warning_ = recording_warning.empty() ? "camera_local_record_failed"
                                                           : recording_warning;
    }
    error_token.clear();
    return true;
  }

  std::cerr << "[gst_recording_daemon] HDMI recording failed token="
            << (hdmi_error.empty() ? kDefaultError : hdmi_error)
            << "; trying SRT fallback" << std::endl;
  shutdown();

  std::string stream_profile;
  if (!query_stream_profile(stream_profile, error_token)) {
    std::cerr << "[gst_recording_daemon] Camera " << kZcamSrtStreamIndex
              << " profile check failed token="
              << (error_token.empty() ? kCameraStreamMismatch : error_token)
              << " body=" << stream_profile << std::endl;
    if (error_token.empty()) {
      error_token = kCameraStreamMismatch;
    }
    return false;
  }
  std::cerr << "[gst_recording_daemon] Camera " << kZcamSrtStreamIndex
            << " profile OK body=" << stream_profile << std::endl;

  stop_camera_srt();

  const std::string srt_output_file = output_stem + ".mkv";
  if (!start_pipeline(srt_output_file, CapturePath::kSrt, error_token)) {
    if (error_token.empty()) {
      error_token = hdmi_error.empty() ? kDefaultError : hdmi_error;
    }
    shutdown();
    return false;
  }

  if (!start_camera_srt(error_token)) {
    std::cerr << "[gst_recording_daemon] Failed to start camera SRT stream token="
              << (error_token.empty() ? kCameraUnreachable : error_token) << std::endl;
    if (error_token.empty()) {
      error_token = kCameraUnreachable;
    }
    shutdown();
    return false;
  }

  actual_output_file = srt_output_file;
  std::string recording_warning;
  if (!start_camera_recording(recording_warning)) {
    std::lock_guard<std::mutex> lock(mutex_);
    local_recording_warning_ = recording_warning.empty() ? "camera_local_record_failed"
                                                         : recording_warning;
  }

  error_token.clear();
  return true;
}

bool GstRecorder::start_pipeline(const std::string& output_file,
                                 CapturePath capture_path,
                                 std::string& error_token) {
  const std::string pipeline_description = build_pipeline_description(capture_path);
  std::cerr << "[gst_recording_daemon] Starting recording"
            << " capture_path=" << CapturePathToString(capture_path)
            << " hdmi_device=" << hdmi_device_path_
            << " camera=" << camera_host_
            << " fps=" << fps_
            << " output=" << output_file
            << " pipeline=\"" << pipeline_description << "\""
            << std::endl;

  GError* parse_error = nullptr;
  GstElement* pipeline = gst_parse_launch(pipeline_description.c_str(), &parse_error);
  if (pipeline == nullptr) {
    std::cerr << "[gst_recording_daemon] Failed to parse GStreamer pipeline: "
              << (parse_error != nullptr && parse_error->message != nullptr ? parse_error->message
                                                                            : "unknown parse error")
              << std::endl;
    if (parse_error != nullptr) {
      g_error_free(parse_error);
    }
    error_token = "pipeline_build_failed";
    return false;
  }

  // We intentionally build the pipeline with named elements. That lets us keep
  // the launch string readable while still setting runtime properties from C++.
  GstElement* source = gst_bin_get_by_name(GST_BIN(pipeline), "source");
  GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
  if ((capture_path == CapturePath::kHdmi && source == nullptr) || sink == nullptr) {
    std::cerr << "[gst_recording_daemon] Pipeline is missing required named elements:"
              << " source=" << (source != nullptr ? "present" : "missing")
              << " sink=" << (sink != nullptr ? "present" : "missing") << std::endl;
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

  if (capture_path == CapturePath::kHdmi) {
    g_object_set(G_OBJECT(source), "device", hdmi_device_path_.c_str(), nullptr);
  }
  g_object_set(G_OBJECT(sink), "location", output_file.c_str(), nullptr);
  if (source != nullptr) {
    gst_object_unref(source);
  }
  gst_object_unref(sink);

  GstBus* bus = gst_element_get_bus(pipeline);
  if (bus == nullptr) {
    std::cerr << "[gst_recording_daemon] Failed to get GStreamer bus from pipeline" << std::endl;
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
    active_capture_path_ = capture_path;
    local_recording_warning_.clear();
  }

  // Moving to PLAYING is the point where the pipeline really tries to acquire
  // the device and start data flow. This is where "device busy" style failures
  // usually surface.
  const GstStateChangeReturn state_result = gst_element_set_state(pipeline, GST_STATE_PLAYING);
  std::cerr << "[gst_recording_daemon] gst_element_set_state(..., PLAYING) returned "
            << StateChangeReturnToString(state_result) << std::endl;
  if (state_result == GST_STATE_CHANGE_FAILURE) {
    error_token = "pipeline_start_failed";

    GstMessage* error_message =
        gst_bus_timed_pop_filtered(bus, 200 * GST_MSECOND, GST_MESSAGE_ERROR);
    if (error_message != nullptr) {
      GError* gst_error = nullptr;
      gchar* debug_info = nullptr;
      gst_message_parse_error(error_message, &gst_error, &debug_info);

      const std::string combined_message = CombineGstErrorMessage(gst_error, debug_info);
      std::cerr << "[gst_recording_daemon] Pipeline failed to enter PLAYING: "
                << (combined_message.empty() ? "no detailed error from GStreamer" : combined_message)
                << std::endl;

      const std::string mapped_token = map_error_to_token(combined_message);
      if (mapped_token != kDefaultError) {
        error_token = mapped_token;
      }

      if (gst_error != nullptr) {
        g_error_free(gst_error);
      }
      g_free(debug_info);
      gst_message_unref(error_message);
    } else {
      std::cerr << "[gst_recording_daemon] Pipeline failed to enter PLAYING but no GST_MESSAGE_ERROR "
                   "was available on the bus within 200 ms"
                << std::endl;
    }

    shutdown();
    return false;
  }

  const std::uint64_t generation = bus_generation_.fetch_add(1) + 1;
  gst_object_ref(bus);
  bus_thread_ = std::thread(&GstRecorder::bus_loop, this, bus, generation);

  error_token.clear();
  std::cerr << "[gst_recording_daemon] Recording pipeline started successfully capture_path="
            << CapturePathToString(capture_path) << std::endl;
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

  // EOS gives the container muxer a chance to flush indexes and final metadata
  // before the file is closed.
  std::cerr << "[gst_recording_daemon] Sending EOS to stop recording cleanly" << std::endl;
  gst_element_send_event(pipeline, gst_event_new_eos());

  std::unique_lock<std::mutex> lock(mutex_);
  eos_cv_.wait_for(lock, kStopTimeout, [this]() { return eos_received_ || runtime_error_seen_; });

  const bool saw_runtime_error = runtime_error_seen_;
  const std::string runtime_error_token = runtime_error_token_;
  lock.unlock();

  shutdown();
  stop_camera_recording();

  if (saw_runtime_error) {
    std::cerr << "[gst_recording_daemon] Stop completed with runtime error token="
              << (runtime_error_token.empty() ? kDefaultError : runtime_error_token)
              << std::endl;
    error_token = runtime_error_token.empty() ? kDefaultError : runtime_error_token;
    return false;
  }

  error_token.clear();
  std::cerr << "[gst_recording_daemon] Recording stopped cleanly" << std::endl;
  return true;
}

void GstRecorder::shutdown() {
  bus_generation_.fetch_add(1);

  GstElement* pipeline = nullptr;
  GstBus* bus = nullptr;
  CapturePath capture_path = CapturePath::kNone;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pipeline = pipeline_;
    pipeline_ = nullptr;
    bus = bus_;
    bus_ = nullptr;
    capture_path = active_capture_path_;
    active_capture_path_ = CapturePath::kNone;
    stop_requested_ = false;
    eos_received_ = false;
    runtime_error_seen_ = false;
    runtime_error_token_.clear();
  }

  if (pipeline != nullptr) {
    std::cerr << "[gst_recording_daemon] Tearing down GStreamer pipeline" << std::endl;
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

  if (pipeline != nullptr && capture_path == CapturePath::kSrt) {
    stop_camera_srt();
  }
}

bool GstRecorder::fetch_camera_info(std::string& info_json, std::string& error_token) const {
  std::string body;
  int status_code = 0;
  if (!http_get("/info", body, error_token, &status_code)) {
    error_token = kCameraUnreachable;
    return false;
  }

  if (status_code != 200 || !LooksLikeJsonObject(body)) {
    error_token = kCameraUnreachable;
    return false;
  }

  info_json = TrimCopy(body);
  error_token.clear();
  return true;
}

GstRecorder::CameraStatus GstRecorder::fetch_camera_status() const {
  CameraStatus status;
  status.checked = true;
  status.ok = fetch_camera_info(status.info_json, status.error);

  std::string body;
  std::string error;
  int status_code = 0;
  if (http_get("/ctrl/mode?action=query", body, error, &status_code) && status_code == 200) {
    status.mode = ExtractJsonStringValue(body, "msg");
  }

  if (http_get("/ctrl/rec?action=remain", body, error, &status_code) && status_code == 200 &&
      !BodyContainsFailureCode(body)) {
    status.media_remain_minutes = ExtractJsonStringValue(body, "msg");
  }

  if (http_get("/ctrl/card?action=query_free", body, error, &status_code) && status_code == 200 &&
      !BodyContainsFailureCode(body)) {
    status.media_free = ExtractJsonStringValue(body, "msg");
  }

  if (http_get("/ctrl/card?action=query_total", body, error, &status_code) &&
      status_code == 200 && !BodyContainsFailureCode(body)) {
    status.media_total = ExtractJsonStringValue(body, "msg");
  }

  if (http_get("/DCIM/", body, error, &status_code) && status_code == 200 &&
      !BodyContainsFailureCode(body)) {
    status.dcim_visible = true;
  }

  status.local_recording_warning = local_recording_warning();
  return status;
}

GstRecorder::CapturePath GstRecorder::active_capture_path() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_capture_path_;
}

std::string GstRecorder::local_recording_warning() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return local_recording_warning_;
}

bool GstRecorder::query_stream_profile(std::string& profile_json,
                                       std::string& error_token) const {
  std::string body;
  int status_code = 0;
  if (!http_get(std::string("/ctrl/stream_setting?action=query&index=") + kZcamSrtStreamIndex,
                body,
                error_token,
                &status_code)) {
    error_token = kCameraUnreachable;
    return false;
  }

  profile_json = TrimCopy(body);
  if (status_code != 200 || !LooksLikeJsonObject(body) || !BodyContainsEncoder(body, "h264")) {
    error_token = kCameraStreamMismatch;
    return false;
  }

  error_token.clear();
  return true;
}

bool GstRecorder::start_camera_srt(std::string& error_token) const {
  const std::string path =
      "/ctrl/srt?action=start&url=" + UrlEncode(BuildSrtPushUrl());

  std::string body;
  int status_code = 0;
  if (!http_get(path, body, error_token, &status_code)) {
    error_token = kCameraUnreachable;
    return false;
  }
  if (status_code != 200 || !LooksLikeJsonObject(body)) {
    error_token = kCameraUnreachable;
    return false;
  }

  std::cerr << "[gst_recording_daemon] Camera SRT start response body=" << TrimCopy(body)
            << std::endl;
  error_token.clear();
  return true;
}

void GstRecorder::stop_camera_srt() const {
  std::string body;
  std::string ignored_error;
  int status_code = 0;
  if (http_get("/ctrl/srt?action=stop", body, ignored_error, &status_code)) {
    std::cerr << "[gst_recording_daemon] Camera SRT stop response status=" << status_code
              << " body=" << TrimCopy(body) << std::endl;
  }
}

bool GstRecorder::start_camera_recording(std::string& warning_token) {
  warning_token.clear();

  std::string body;
  int status_code = 0;
  if (!http_get("/ctrl/rec?action=start",
                body,
                warning_token,
                &status_code,
                kRecordHttpTimeoutSeconds)) {
    warning_token = warning_token.empty() ? "camera_local_record_failed" : warning_token;
    std::cerr << "[gst_recording_daemon] Camera local recording start request failed token="
              << warning_token << std::endl;
  } else if (status_code != 200 || !LooksLikeJsonObject(body) || BodyContainsFailureCode(body)) {
    // Keep the response body in logs for firmware-specific failures, but do not
    // make local recording authoritative over the host capture.
    warning_token = "camera_local_record_failed";
    std::cerr << "[gst_recording_daemon] Camera local recording start response unexpected"
              << " status=" << status_code << " body=" << TrimCopy(body) << std::endl;
  } else {
    std::cerr << "[gst_recording_daemon] Camera local recording start response body="
              << TrimCopy(body) << std::endl;
  }

  std::string mode;
  std::string poll_error;
  if (poll_camera_mode("rec_ing", kCameraRecordPollTimeout, mode, poll_error)) {
    std::lock_guard<std::mutex> lock(mutex_);
    local_recording_warning_.clear();
    return true;
  }

  if (warning_token.empty()) {
    warning_token = poll_error.empty() ? "camera_local_record_unconfirmed" : poll_error;
  }
  std::cerr << "[gst_recording_daemon] Camera local recording was not confirmed"
            << " mode=" << (mode.empty() ? "-" : mode)
            << " token=" << warning_token << std::endl;
  return false;
}

void GstRecorder::stop_camera_recording() {
  std::string body;
  std::string ignored_error;
  int status_code = 0;
  if (http_get("/ctrl/rec?action=stop",
               body,
               ignored_error,
               &status_code,
               kRecordHttpTimeoutSeconds)) {
    std::cerr << "[gst_recording_daemon] Camera local recording stop response status="
              << status_code << " body=" << TrimCopy(body) << std::endl;
  } else {
    std::cerr << "[gst_recording_daemon] Camera local recording stop request failed token="
              << (ignored_error.empty() ? kCameraUnreachable : ignored_error) << std::endl;
  }

  std::string mode;
  std::string poll_error;
  const bool mode_ok = poll_camera_mode("rec", kCameraRecordPollTimeout, mode, poll_error);
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_ok) {
    local_recording_warning_.clear();
  } else {
    local_recording_warning_ =
        ignored_error.empty() ? "camera_local_stop_unconfirmed" : ignored_error;
  }
}

bool GstRecorder::poll_camera_mode(const std::string& expected_mode,
                                   std::chrono::milliseconds timeout,
                                   std::string& mode,
                                   std::string& error_token) const {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  mode.clear();
  error_token.clear();

  while (std::chrono::steady_clock::now() <= deadline) {
    std::string body;
    int status_code = 0;
    if (http_get("/ctrl/mode?action=query", body, error_token, &status_code) &&
        status_code == 200) {
      const std::string compact = ToLowerCopy(body);
      const std::string needle = "\"msg\":\"" + ToLowerCopy(expected_mode) + "\"";
      mode = ExtractJsonStringValue(body, "msg");
      if (compact.find(needle) != std::string::npos) {
        error_token.clear();
        return true;
      }
    }
    std::this_thread::sleep_for(kCameraRecordPollPeriod);
  }

  if (error_token.empty()) {
    error_token = "camera_local_record_unconfirmed";
  }
  return false;
}

bool GstRecorder::http_get(const std::string& path,
                           std::string& body,
                           std::string& error_token,
                           int* status_code,
                           int timeout_seconds) const {
  body.clear();
  if (status_code != nullptr) {
    *status_code = 0;
  }

  const std::string host = StripHttpSchemeAndPath(camera_host_);
  if (host.empty()) {
    error_token = kCameraUnreachable;
    return false;
  }

  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  addrinfo* results = nullptr;
  const int gai_result = ::getaddrinfo(host.c_str(), "80", &hints, &results);
  if (gai_result != 0) {
    std::cerr << "[gst_recording_daemon] Failed to resolve camera host " << host << ": "
              << gai_strerror(gai_result) << std::endl;
    error_token = kCameraUnreachable;
    return false;
  }

  int fd = -1;
  for (addrinfo* candidate = results; candidate != nullptr; candidate = candidate->ai_next) {
    fd = ::socket(candidate->ai_family, candidate->ai_socktype, candidate->ai_protocol);
    if (fd == -1) {
      continue;
    }

    timeval timeout{};
    timeout.tv_sec = timeout_seconds;
    timeout.tv_usec = 0;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    if (::connect(fd, candidate->ai_addr, candidate->ai_addrlen) == 0) {
      break;
    }

    ::close(fd);
    fd = -1;
  }
  ::freeaddrinfo(results);

  if (fd == -1) {
    std::cerr << "[gst_recording_daemon] Failed to connect to camera HTTP host=" << host
              << " errno=" << std::strerror(errno) << std::endl;
    error_token = kCameraUnreachable;
    return false;
  }

  const std::string request = "GET " + path + " HTTP/1.1\r\nHost: " + host +
                              "\r\nConnection: close\r\nAccept: application/json\r\n\r\n";
  const char* data = request.data();
  std::size_t remaining = request.size();
  while (remaining > 0) {
    const ssize_t sent = ::send(fd, data, remaining, MSG_NOSIGNAL);
    if (sent <= 0) {
      std::cerr << "[gst_recording_daemon] Failed to send camera HTTP request path=" << path
                << " errno=" << std::strerror(errno) << std::endl;
      ::close(fd);
      error_token = kCameraUnreachable;
      return false;
    }
    data += sent;
    remaining -= static_cast<std::size_t>(sent);
  }

  std::string response;
  char buffer[4096];
  while (true) {
    const ssize_t bytes_read = ::recv(fd, buffer, sizeof(buffer), 0);
    if (bytes_read > 0) {
      response.append(buffer, static_cast<std::size_t>(bytes_read));
      const std::size_t header_end = response.find("\r\n\r\n");
      if (header_end != std::string::npos) {
        const std::size_t content_length = ParseContentLength(response.substr(0, header_end));
        if (content_length > 0 && response.size() >= header_end + 4 + content_length) {
          break;
        }
      }
      continue;
    }
    if (bytes_read == 0) {
      break;
    }
    if (errno == EINTR) {
      continue;
    }
    std::cerr << "[gst_recording_daemon] Failed to read camera HTTP response path=" << path
              << " errno=" << std::strerror(errno) << std::endl;
    ::close(fd);
    error_token = kCameraUnreachable;
    return false;
  }
  ::close(fd);

  const std::size_t header_end = response.find("\r\n\r\n");
  if (header_end == std::string::npos) {
    error_token = kCameraUnreachable;
    return false;
  }

  const std::string status_line = response.substr(0, response.find("\r\n"));
  std::istringstream status_stream(status_line);
  std::string http_version;
  int parsed_status = 0;
  status_stream >> http_version >> parsed_status;
  if (status_code != nullptr) {
    *status_code = parsed_status;
  }
  body = response.substr(header_end + 4);
  error_token.clear();
  return parsed_status > 0;
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
      if (should_report_error) {
        std::cerr << "[gst_recording_daemon] Unexpected EOS received from recording pipeline"
                  << std::endl;
      }
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

      const std::string combined_message = CombineGstErrorMessage(gst_error, debug_info);

      const std::string error_token = map_error_to_token(combined_message);
      RuntimeErrorCallback callback;
      GstElement* pipeline = nullptr;

      std::cerr << "[gst_recording_daemon] Runtime GStreamer error token=" << error_token
                << " details="
                << (combined_message.empty() ? "no detailed error from GStreamer" : combined_message)
                << std::endl;

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

std::string GstRecorder::build_pipeline_description(CapturePath capture_path) const {
  // This is intentionally written as a single human-readable launch string.
  // For a small daemon, `gst_parse_launch()` keeps the pipeline easier to read
  // than constructing every element and pad by hand.
  if (capture_path == CapturePath::kHdmi) {
    const HdmiCaptureFormat format =
        SelectBestHdmiCaptureFormat(hdmi_device_path_, std::max(fps_, 1));
    const int key_frame_interval = std::max(1, static_cast<int>(std::round(FpsValue(format))));
    const std::string caps =
        "width=" + std::to_string(format.width) +
        ",height=" + std::to_string(format.height) +
        ",framerate=" + std::to_string(format.fps_denominator) +
        "/" + std::to_string(format.fps_numerator);
    const std::string encoder =
        "! queue "
        "! videoconvert "
        "! video/x-raw,format=I420 "
        "! queue "
        "! x264enc tune=zerolatency speed-preset=veryfast bitrate=20000 key-int-max=" +
        std::to_string(key_frame_interval) +
        " "
        "! h264parse config-interval=-1 "
        "! video/x-h264,stream-format=avc,alignment=au "
        "! mp4mux name=mux "
        "! filesink name=sink sync=false";

    if (format.fourcc == V4L2_PIX_FMT_H264) {
      return "v4l2src name=source io-mode=mmap do-timestamp=true "
             "! video/x-h264," +
             caps +
             " "
             "! queue "
             "! h264parse config-interval=-1 "
             "! video/x-h264,stream-format=avc,alignment=au "
             "! mp4mux name=mux "
             "! filesink name=sink sync=false";
    }

    if (format.fourcc == V4L2_PIX_FMT_YUYV) {
      return "v4l2src name=source io-mode=mmap do-timestamp=true "
             "! video/x-raw,format=YUY2," +
             caps + " " + encoder;
    }

    return "v4l2src name=source io-mode=mmap do-timestamp=true "
           "! image/jpeg," +
           caps +
           " "
           "! queue "
           "! jpegdec " +
           encoder;
  }

  return "srtsrc uri=\"srt://:" + std::to_string(kSrtPort) +
         "?mode=listener&latency=200\" "
         "! tsdemux name=demux "
         "matroskamux name=mux "
         "! filesink name=sink sync=false "
         "demux. ! video/x-h264 ! queue "
         "! h264parse "
         "! video/x-h264,stream-format=avc,alignment=au "
         "! mux. "
         "demux. ! audio/mpeg,mpegversion=4 ! queue "
         "! aacparse "
         "! mux.";
}

std::string GstRecorder::map_error_to_token(const std::string& message) {
  const std::string lowered = ToLowerCopy(message);
  if (lowered.find("srt") != std::string::npos ||
      lowered.find("connection") != std::string::npos ||
      lowered.find("timed out") != std::string::npos ||
      lowered.find("timeout") != std::string::npos ||
      lowered.find("refused") != std::string::npos ||
      lowered.find("network") != std::string::npos) {
    return kCameraUnreachable;
  }
  if (lowered.find("device or resource busy") != std::string::npos ||
      lowered.find("resource busy") != std::string::npos ||
      lowered.find("busy") != std::string::npos) {
    return "device_busy";
  }
  if (lowered.find("no such file") != std::string::npos ||
      lowered.find("cannot identify device") != std::string::npos ||
      lowered.find("not found") != std::string::npos) {
    return "device_unavailable";
  }
  return kDefaultError;
}
