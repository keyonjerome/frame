#include "ControlServer.h"

#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <utility>
#include <vector>

namespace {

constexpr int kListenBacklog = 4;
constexpr int kPollTimeoutMs = 100;

}  // namespace

ControlServer::ControlServer(std::string socket_path)
    : socket_path_(std::move(socket_path)) {}

ControlServer::~ControlServer() {
  stop();
}

bool ControlServer::start(CommandHandler command_handler) {
  if (socket_path_.size() >= sizeof(sockaddr_un{}.sun_path)) {
    std::cerr << "socket path is too long: " << socket_path_ << std::endl;
    return false;
  }

  command_handler_ = std::move(command_handler);
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
    cleanup_socket_file();
    return false;
  }

  if (::listen(server_fd_, kListenBacklog) == -1) {
    std::cerr << "failed to listen on Unix socket: " << std::strerror(errno) << std::endl;
    ::close(server_fd_);
    server_fd_ = -1;
    cleanup_socket_file();
    return false;
  }

  stop_requested_.store(false);
  server_thread_ = std::thread(&ControlServer::run_loop, this);
  return true;
}

void ControlServer::stop() {
  const bool already_stopped = stop_requested_.exchange(true);

  int server_fd = server_fd_;
  if (server_fd != -1) {
    ::shutdown(server_fd, SHUT_RDWR);
    ::close(server_fd);
    server_fd_ = -1;
  }

  int client_fd = -1;
  {
    std::lock_guard<std::mutex> lock(client_mutex_);
    client_fd = active_client_fd_;
    active_client_fd_ = -1;
  }
  if (client_fd != -1) {
    ::shutdown(client_fd, SHUT_RDWR);
    ::close(client_fd);
  }

  if (server_thread_.joinable()) {
    server_thread_.join();
  }

  if (!already_stopped) {
    cleanup_socket_file();
  }
}

bool ControlServer::has_client() const {
  return get_active_client_fd() != -1;
}

bool ControlServer::send_line_to_client(const std::string& line) {
  std::lock_guard<std::mutex> lock(client_mutex_);
  if (active_client_fd_ == -1) {
    return false;
  }

  std::string payload = line;
  payload.push_back('\n');

  const char* data = payload.data();
  std::size_t remaining = payload.size();
  while (remaining > 0) {
    const ssize_t sent = ::send(active_client_fd_, data, remaining, MSG_NOSIGNAL);
    if (sent <= 0) {
      return false;
    }
    data += sent;
    remaining -= static_cast<std::size_t>(sent);
  }

  return true;
}

void ControlServer::run_loop() {
  // The server loop is intentionally plain:
  // 1. poll for either a new connection or data from the current client
  // 2. accept exactly one client at a time
  // 3. reject any second client while the first one is still attached
  // 4. forward complete lines to the daemon callback
  std::string pending_input;

  while (!stop_requested_.load()) {
    std::vector<pollfd> poll_fds;
    poll_fds.push_back({server_fd_, POLLIN, 0});

    const int client_fd = get_active_client_fd();
    if (client_fd != -1) {
      poll_fds.push_back({client_fd, POLLIN | POLLERR | POLLHUP, 0});
    }

    const int poll_result = ::poll(poll_fds.data(), poll_fds.size(), kPollTimeoutMs);
    if (poll_result < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (!stop_requested_.load()) {
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

      const std::string response = command_handler_ ? command_handler_(line) : "";
      if (!response.empty() && !send_line_to_client(response)) {
        disconnect_client_if(client_fd);
        pending_input.clear();
        break;
      }

      newline_pos = pending_input.find('\n');
    }
  }
}

void ControlServer::cleanup_socket_file() {
  if (!socket_path_.empty()) {
    ::unlink(socket_path_.c_str());
  }
}

void ControlServer::disconnect_client_if(int fd) {
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

int ControlServer::get_active_client_fd() const {
  std::lock_guard<std::mutex> lock(client_mutex_);
  return active_client_fd_;
}
