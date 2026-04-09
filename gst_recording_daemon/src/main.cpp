#include "RecorderDaemon.h"

#include <gst/gst.h>
#include <signal.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

namespace {

constexpr char kDefaultDevice[] = "/dev/video0";
constexpr char kDefaultSocketPath[] = "/tmp/filmer_recorder.sock";
constexpr int kDefaultFps = 60;

volatile sig_atomic_t g_shutdown_requested = 0;

void handle_signal(int) {
  g_shutdown_requested = 1;
}

void print_usage(const char* program_name) {
  std::cerr << "Usage: " << program_name
            << " [--device /dev/video0] [--fps 60] [--socket-path /tmp/filmer_recorder.sock]"
            << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  std::string device_path = kDefaultDevice;
  std::string socket_path = kDefaultSocketPath;
  int fps = kDefaultFps;

  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    if (arg == "--device") {
      if (index + 1 >= argc) {
        print_usage(argv[0]);
        return 1;
      }
      device_path = argv[++index];
      continue;
    }
    if (arg == "--fps") {
      if (index + 1 >= argc) {
        print_usage(argv[0]);
        return 1;
      }
      try {
        fps = std::stoi(argv[++index]);
      } catch (const std::exception&) {
        std::cerr << "invalid fps value" << std::endl;
        return 1;
      }
      continue;
    }
    if (arg == "--socket-path") {
      if (index + 1 >= argc) {
        print_usage(argv[0]);
        return 1;
      }
      socket_path = argv[++index];
      continue;
    }

    print_usage(argv[0]);
    return 1;
  }

  if (fps <= 0) {
    std::cerr << "fps must be positive" << std::endl;
    return 1;
  }

  gst_init(&argc, &argv);

  struct sigaction action {};
  action.sa_handler = handle_signal;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;
  sigaction(SIGINT, &action, nullptr);
  sigaction(SIGTERM, &action, nullptr);

  {
    RecorderDaemon daemon(device_path, fps, socket_path);
    if (!daemon.initialize()) {
      gst_deinit();
      return 1;
    }

    std::atomic<bool> run_finished{false};
    std::thread daemon_thread([&daemon, &run_finished]() {
      daemon.run();
      run_finished.store(true);
    });

    while (!g_shutdown_requested && !run_finished.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    daemon.request_shutdown();
    daemon_thread.join();
  }

  gst_deinit();
  return 0;
}
