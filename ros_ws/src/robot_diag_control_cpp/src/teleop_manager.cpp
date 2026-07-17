#include "robot_diag_control_cpp/teleop_manager.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace robot_diag_control_cpp
{
namespace
{
uint64_t age_ms_or_zero(
  bool has_command, TeleopManager::SteadyTimePoint now,
  TeleopManager::SteadyTimePoint last_command_at)
{
  if (!has_command) {
    return 0;
  }
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(now - last_command_at).count());
}
} // namespace

TeleopManager::TeleopManager(
  GatewayStateStore & store, TeleopCommandPublisher publisher, TeleopManagerConfig config,
  TimeSource time_source)
: _store(store),
  _publisher(std::move(publisher)),
  _config(config),
  _time_source(std::move(time_source))
{
  if (!_publisher) {
    throw std::runtime_error("teleop publisher must be configured");
  }
  if (_config.max_linear_mps <= 0.0) {
    throw std::runtime_error("teleop max_linear_mps must be positive");
  }
  if (_config.max_angular_rad_s <= 0.0) {
    throw std::runtime_error("teleop max_angular_rad_s must be positive");
  }
  if (_config.deadman_timeout.count() <= 0) {
    throw std::runtime_error("teleop deadman_timeout must be positive");
  }
  if (_config.min_command_interval.count() < 0) {
    throw std::runtime_error("teleop min_command_interval must be non-negative");
  }
  _store.set_teleop_status(status_locked());
}

TeleopManager::~TeleopManager()
{
  shutdown();
}

TeleopControlResult TeleopManager::set_enabled(bool enabled)
{
  std::lock_guard<std::mutex> lock(_mutex);
  if (enabled) {
    _enabled = true;
    _timed_out = false;
    _last_error.clear();
    const auto status = status_locked();
    _store.set_teleop_status(status);
    return TeleopControlResult{true, "teleop enabled", status};
  }

  _enabled = false;
  _timed_out = false;
  _last_error.clear();
  publish_stop_locked();
  const auto status = status_locked();
  _store.set_teleop_status(status);
  return TeleopControlResult{true, "teleop disabled", status};
}

TeleopControlResult TeleopManager::send_command(const TeleopCommand & command)
{
  std::lock_guard<std::mutex> lock(_mutex);
  if (!_enabled) {
    return reject_locked("teleop disabled");
  }
  if (_timed_out) {
    return reject_locked("teleop timed out; re-enable required");
  }
  if (!command_in_bounds(command)) {
    return reject_locked("teleop command exceeds configured bounds");
  }

  const auto now = _time_source();
  if (_has_command && now - _last_publish_at < _config.min_command_interval) {
    return reject_locked("teleop command rate limited");
  }

  _publisher(command);
  _has_command = true;
  _last_command_at = now;
  _last_publish_at = now;
  _last_error.clear();
  const auto status = status_locked();
  _store.set_teleop_status(status);
  return TeleopControlResult{true, "teleop command sent", status};
}

void TeleopManager::poll()
{
  std::lock_guard<std::mutex> lock(_mutex);
  if (!_enabled || _timed_out || !_has_command) {
    _store.set_teleop_status(status_locked());
    return;
  }

  if (_time_source() - _last_command_at <= _config.deadman_timeout) {
    _store.set_teleop_status(status_locked());
    return;
  }

  _enabled = false;
  _timed_out = true;
  _last_error = "teleop deadman timeout";
  publish_stop_locked();
  _store.set_teleop_status(status_locked());
}

void TeleopManager::shutdown()
{
  std::lock_guard<std::mutex> lock(_mutex);
  if (_enabled || _has_command) {
    _enabled = false;
    publish_stop_locked();
  }
  _has_command = false;
  _store.set_teleop_status(status_locked());
}

bool TeleopManager::command_in_bounds(const TeleopCommand & command) const
{
  return std::isfinite(command.linear_x_mps) && std::isfinite(command.linear_y_mps) &&
         std::isfinite(command.angular_z_rad_s) &&
         std::abs(command.linear_x_mps) <= _config.max_linear_mps &&
         std::abs(command.linear_y_mps) <= _config.max_linear_mps &&
         std::abs(command.angular_z_rad_s) <= _config.max_angular_rad_s;
}

void TeleopManager::publish_stop_locked()
{
  _publisher(TeleopCommand{});
  _last_publish_at = _time_source();
}

TeleopStatusSnapshot TeleopManager::status_locked(std::string last_error) const
{
  const auto now = _time_source();
  const auto error = last_error.empty() ? _last_error : std::move(last_error);
  return TeleopStatusSnapshot{
    _timed_out ? TeleopState::TimedOut : (_enabled ? TeleopState::Enabled : TeleopState::Disabled),
    _enabled,
    _timed_out,
    age_ms_or_zero(_has_command, now, _last_command_at),
    _config.max_linear_mps,
    _config.max_angular_rad_s,
    error,
  };
}

TeleopControlResult TeleopManager::reject_locked(std::string message)
{
  _last_error = message;
  const auto status = status_locked();
  _store.set_teleop_status(status);
  return TeleopControlResult{false, std::move(message), status};
}
} // namespace robot_diag_control_cpp
