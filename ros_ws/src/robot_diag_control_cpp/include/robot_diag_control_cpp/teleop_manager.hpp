#pragma once

#include <chrono>
#include <functional>
#include <mutex>
#include <string>

#include "robot_diag_control_cpp/gateway_state.hpp"

namespace robot_diag_control_cpp
{
struct TeleopCommand
{
  double linear_x_mps{0.0};
  double linear_y_mps{0.0};
  double angular_z_rad_s{0.0};
};

struct TeleopControlResult
{
  bool                 accepted{false};
  std::string          message{};
  TeleopStatusSnapshot teleop{};
};

struct TeleopManagerConfig
{
  double                    max_linear_mps{0.35};
  double                    max_angular_rad_s{0.8};
  std::chrono::milliseconds deadman_timeout{500};
  std::chrono::milliseconds min_command_interval{50};
};

using TeleopCommandPublisher = std::function<void(const TeleopCommand &)>;

class TeleopManager
{
public:
  using SteadyTimePoint = GatewayStateStore::SteadyTimePoint;
  using TimeSource = GatewayStateStore::TimeSource;

  explicit TeleopManager(
    GatewayStateStore & store, TeleopCommandPublisher publisher,
    TeleopManagerConfig config = {}, TimeSource time_source = std::chrono::steady_clock::now);
  ~TeleopManager();

  TeleopManager(const TeleopManager &) = delete;
  TeleopManager & operator=(const TeleopManager &) = delete;

  TeleopControlResult set_enabled(bool enabled);
  TeleopControlResult send_command(const TeleopCommand & command);
  void poll();
  void shutdown();

private:
  bool command_in_bounds(const TeleopCommand & command) const;
  void publish_stop_locked();
  TeleopStatusSnapshot status_locked(std::string last_error = "") const;
  TeleopControlResult reject_locked(std::string message);

  GatewayStateStore & _store;
  TeleopCommandPublisher    _publisher{};
  TeleopManagerConfig       _config{};
  TimeSource                _time_source{};
  mutable std::mutex        _mutex{};
  bool                      _enabled{false};
  bool                      _timed_out{false};
  bool                      _has_command{false};
  SteadyTimePoint           _last_command_at{};
  SteadyTimePoint           _last_publish_at{};
  std::string               _last_error{};
};
} // namespace robot_diag_control_cpp
