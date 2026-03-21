#include "robot_diag_control_cpp/preview_process_manager.hpp"

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

namespace robot_diag_control_cpp
{
namespace
{
PreviewStatusSnapshot preview_disabled(
  GatewayStateStore & store, PreviewProfile profile, const std::string & last_error)
{
  return store.set_preview_disabled(profile, last_error);
}
} // namespace

PreviewProcessManager::PreviewProcessManager(
  GatewayStateStore & store, PreviewCommandFactory command_factory,
  std::chrono::milliseconds stop_timeout, std::chrono::milliseconds startup_timeout)
: _store(store),
  _command_factory(std::move(command_factory)),
  _stop_timeout(stop_timeout),
  _startup_timeout(startup_timeout)
{
}

PreviewProcessManager::~PreviewProcessManager()
{
  std::lock_guard<std::mutex> lock(_mutex);
  if (_child_pid > 0) {
    stop_child_locked();
    const auto current = _store.get_preview_status();
    _store.set_preview_disabled(current.profile);
  }
}

PreviewControlResult PreviewProcessManager::set_enabled(
  bool enabled, std::optional<PreviewProfile> profile)
{
  std::lock_guard<std::mutex> lock(_mutex);
  poll_locked();

  const auto current = _store.get_preview_status();
  const PreviewProfile target_profile = profile.value_or(current.profile);
  if (enabled) {
    return enable_locked(target_profile);
  }

  return disable_locked(target_profile);
}

void PreviewProcessManager::poll()
{
  std::lock_guard<std::mutex> lock(_mutex);
  poll_locked();
}

PreviewControlResult PreviewProcessManager::enable_locked(PreviewProfile target_profile)
{
  const auto current = _store.get_preview_status();
  if (current.state == PreviewState::Running && _child_pid > 0 &&
    current.profile == target_profile && current.last_error.empty())
  {
    return PreviewControlResult{true, "preview already running", current};
  }

  if (_child_pid > 0 && !stop_child_locked()) {
    const auto preview = preview_disabled(
      _store, current.profile, "preview process did not stop cleanly");
    return PreviewControlResult{false, preview.last_error, preview};
  }

  if (!_command_factory) {
    const auto preview = preview_disabled(_store, target_profile, "preview command not configured");
    return PreviewControlResult{false, preview.last_error, preview};
  }

  const auto resolution = _command_factory(target_profile);
  if (!resolution.ok) {
    const auto preview = preview_disabled(_store, target_profile, resolution.message);
    return PreviewControlResult{false, resolution.message, preview};
  }

  int exec_error_pipe[2];
  if (pipe(exec_error_pipe) != 0) {
    const auto message = std::string("failed to create preview exec pipe: ") + std::strerror(errno);
    const auto preview = preview_disabled(_store, target_profile, message);
    return PreviewControlResult{false, message, preview};
  }

  if (fcntl(exec_error_pipe[1], F_SETFD, FD_CLOEXEC) != 0) {
    const auto saved_errno = errno;
    close(exec_error_pipe[0]);
    close(exec_error_pipe[1]);
    const auto message =
      std::string("failed to configure preview exec pipe: ") + std::strerror(saved_errno);
    const auto preview = preview_disabled(_store, target_profile, message);
    return PreviewControlResult{false, message, preview};
  }

  const pid_t child_pid = fork();
  if (child_pid < 0) {
    const auto saved_errno = errno;
    close(exec_error_pipe[0]);
    close(exec_error_pipe[1]);
    const auto message = std::string("failed to fork preview process: ") +
      std::strerror(saved_errno);
    const auto preview = preview_disabled(_store, target_profile, message);
    return PreviewControlResult{false, message, preview};
  }

  if (child_pid == 0) {
    close(exec_error_pipe[0]);

    const int devnull_fd = open("/dev/null", O_RDWR);
    if (devnull_fd < 0) {
      const int saved_errno = errno;
      const auto written = write(exec_error_pipe[1], &saved_errno, sizeof(saved_errno));
      (void)written;
      _exit(127);
    }

    if (dup2(devnull_fd, STDIN_FILENO) < 0 ||
      dup2(devnull_fd, STDOUT_FILENO) < 0 ||
      dup2(devnull_fd, STDERR_FILENO) < 0)
    {
      const int saved_errno = errno;
      close(devnull_fd);
      const auto written = write(exec_error_pipe[1], &saved_errno, sizeof(saved_errno));
      (void)written;
      _exit(127);
    }

    if (devnull_fd > STDERR_FILENO) {
      close(devnull_fd);
    }

    std::vector<std::string> argv_storage;
    argv_storage.reserve(1 + resolution.command.arguments.size());
    argv_storage.push_back(resolution.command.executable);
    argv_storage.insert(
      argv_storage.end(),
      resolution.command.arguments.begin(),
      resolution.command.arguments.end());

    std::vector<char *> argv;
    argv.reserve(argv_storage.size() + 1);
    for (auto & argument : argv_storage) {
      argv.push_back(argument.data());
    }
    argv.push_back(nullptr);

    execvp(resolution.command.executable.c_str(), argv.data());

    const int saved_errno = errno;
    const auto written = write(exec_error_pipe[1], &saved_errno, sizeof(saved_errno));
    (void)written;
    _exit(127);
  }

  close(exec_error_pipe[1]);
  int exec_errno = 0;
  const auto read_size = read(exec_error_pipe[0], &exec_errno, sizeof(exec_errno));
  close(exec_error_pipe[0]);

  if (read_size > 0) {
    int wait_status = 0;
    const auto waited_pid = waitpid(child_pid, &wait_status, 0);
    (void)waited_pid;
    const auto message =
      std::string("failed to exec preview command: ") + std::strerror(exec_errno);
    const auto preview = preview_disabled(_store, target_profile, message);
    return PreviewControlResult{false, message, preview};
  }

  _child_pid = child_pid;
  const auto startup_deadline = std::chrono::steady_clock::now() + _startup_timeout;
  while (std::chrono::steady_clock::now() < startup_deadline) {
    int wait_status = 0;
    if (child_exited_locked(wait_status)) {
      _child_pid = -1;
      const auto message = wait_status_message(wait_status);
      const auto preview = preview_disabled(_store, target_profile, message);
      return PreviewControlResult{false, message, preview};
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }

  const auto preview = _store.set_preview_running(target_profile);
  return PreviewControlResult{true, "preview started", preview};
}

PreviewControlResult PreviewProcessManager::disable_locked(PreviewProfile target_profile)
{
  const auto current = _store.get_preview_status();
  if (_child_pid <= 0) {
    return PreviewControlResult{true, "preview already disabled", current};
  }

  if (!stop_child_locked()) {
    const auto preview = preview_disabled(
      _store, current.profile, "preview process did not stop cleanly");
    return PreviewControlResult{false, preview.last_error, preview};
  }

  const auto preview = _store.set_preview_disabled(target_profile);
  return PreviewControlResult{true, "preview stopped", preview};
}

void PreviewProcessManager::poll_locked()
{
  if (_child_pid <= 0) {
    return;
  }

  int wait_status = 0;
  if (!child_exited_locked(wait_status)) {
    return;
  }

  const auto current = _store.get_preview_status();
  _child_pid = -1;
  _store.set_preview_disabled(current.profile, wait_status_message(wait_status));
}

bool PreviewProcessManager::child_exited_locked(int & wait_status)
{
  const pid_t result = waitpid(_child_pid, &wait_status, WNOHANG);
  if (result == 0) {
    return false;
  }

  if (result < 0) {
    if (errno == ECHILD) {
      wait_status = 0;
      return true;
    }

    const auto current = _store.get_preview_status();
    _child_pid = -1;
    _store.set_preview_disabled(
      current.profile,
      std::string("failed to poll preview process: ") + std::strerror(errno));
    return false;
  }

  return true;
}

bool PreviewProcessManager::stop_child_locked()
{
  if (_child_pid <= 0) {
    return true;
  }

  if (kill(_child_pid, SIGTERM) != 0 && errno != ESRCH) {
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() + _stop_timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    int wait_status = 0;
    const pid_t result = waitpid(_child_pid, &wait_status, WNOHANG);
    if (result == _child_pid) {
      _child_pid = -1;
      return true;
    }

    if (result < 0) {
      if (errno == ECHILD) {
        _child_pid = -1;
        return true;
      }
      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }

  if (kill(_child_pid, SIGKILL) != 0 && errno != ESRCH) {
    return false;
  }

  int wait_status = 0;
  const pid_t result = waitpid(_child_pid, &wait_status, 0);
  if (result == _child_pid || (result < 0 && errno == ECHILD)) {
    _child_pid = -1;
    return true;
  }

  return false;
}

std::string PreviewProcessManager::wait_status_message(int wait_status) const
{
  if (WIFEXITED(wait_status)) {
    return "preview process exited with code " + std::to_string(WEXITSTATUS(wait_status));
  }

  if (WIFSIGNALED(wait_status)) {
    return "preview process terminated by signal " + std::to_string(WTERMSIG(wait_status));
  }

  return "preview process exited unexpectedly";
}
} // namespace robot_diag_control_cpp
