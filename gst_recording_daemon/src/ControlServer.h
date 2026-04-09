#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

class ControlServer {
 public:
  using CommandHandler = std::function<std::string(const std::string&)>;

  explicit ControlServer(std::string socket_path);
  ~ControlServer();

  bool start(CommandHandler command_handler);
  void stop();

  bool has_client() const;
  bool send_line_to_client(const std::string& line);

 private:
  void run_loop();
  void cleanup_socket_file();
  void disconnect_client_if(int fd);
  int get_active_client_fd() const;

  const std::string socket_path_;

  mutable std::mutex client_mutex_;
  CommandHandler command_handler_;

  int server_fd_{-1};
  int active_client_fd_{-1};

  std::thread server_thread_;
  std::atomic<bool> stop_requested_{false};
};
