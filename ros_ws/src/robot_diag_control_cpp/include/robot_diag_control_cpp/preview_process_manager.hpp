#pragma once

#include <chrono>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <sys/types.h>

#include "robot_diag_control_cpp/gateway_state.hpp"

namespace robot_diag_control_cpp
{
struct PreviewProcessCommand
{
  std::string executable{};
  std::vector<std::string> arguments{};
};

struct PreviewControlResult
{
  bool accepted{false};
  std::string message{};
  PreviewStatusSnapshot preview{};
};

struct PreviewCommandResolution
{
  bool ok{false};
  std::string message{};
  PreviewProcessCommand command{};
};

using PreviewCommandFactory = std::function<PreviewCommandResolution(PreviewProfile)>;

class PreviewProcessManager
{
public:
  explicit PreviewProcessManager(
    GatewayStateStore & store,
    PreviewCommandFactory command_factory = {},
    std::chrono::milliseconds stop_timeout = std::chrono::milliseconds(1500),
    std::chrono::milliseconds startup_timeout = std::chrono::milliseconds(300));
  ~PreviewProcessManager();

  PreviewControlResult set_enabled(
    bool enabled, std::optional<PreviewProfile> profile = std::nullopt);
  void poll();

private:
  PreviewControlResult enable_locked(PreviewProfile target_profile);
  PreviewControlResult disable_locked(PreviewProfile target_profile);
  void poll_locked();
  bool child_exited_locked(int & wait_status);
  bool stop_child_locked();
  std::string wait_status_message(int wait_status) const;

  GatewayStateStore & _store;
  PreviewCommandFactory _command_factory{};
  std::chrono::milliseconds _stop_timeout{1500};
  std::chrono::milliseconds _startup_timeout{300};
  std::mutex _mutex{};
  pid_t _child_pid{-1};
};
} // namespace robot_diag_control_cpp
