#include "robot_diag_control_cpp/gateway_state.hpp"

#include <cmath>

namespace robot_diag_control_cpp
{
GatewayStateStore::GatewayStateStore(
  std::string gateway_name, std::string gateway_version,
  std::chrono::milliseconds vision_stale_after, std::chrono::milliseconds odom_stale_after,
  TimeSource time_source)
: _gateway_name(std::move(gateway_name)),
  _gateway_version(std::move(gateway_version)),
  _vision_stale_after(vision_stale_after),
  _odom_stale_after(odom_stale_after),
  _time_source(std::move(time_source))
{
}

SystemStatusSnapshot GatewayStateStore::get_system_status() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  const auto vision = vision_snapshot_locked();
  return SystemStatusSnapshot{
    _gateway_name,
    _gateway_version,
    _preview,
    vision,
    robot_health_snapshot_locked(vision),
  };
}

PreviewStatusSnapshot GatewayStateStore::get_preview_status() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _preview;
}

PreviewStatusSnapshot GatewayStateStore::set_preview_running(PreviewProfile profile)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _preview = PreviewStatusSnapshot{
    PreviewState::Running,
    profile,
    "",
  };
  return _preview;
}

PreviewStatusSnapshot GatewayStateStore::set_preview_disabled(
  PreviewProfile profile, std::string last_error)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _preview = PreviewStatusSnapshot{
    PreviewState::Disabled,
    profile,
    std::move(last_error),
  };
  return _preview;
}

void GatewayStateStore::update_vision_perf(const omniseer_msgs::msg::VisionPerfSummary & msg)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _has_vision_perf = true;
  _vision_perf = StoredVisionPerf{
    static_cast<double>(msg.producer_fps),
    static_cast<double>(msg.consumer_fps),
    static_cast<double>(msg.last_infer_ms),
    msg.infer_error_count,
    msg.capture_fatal_error_count,
    _time_source(),
  };
}

void GatewayStateStore::update_odometry(const nav_msgs::msg::Odometry & msg)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _has_odometry = true;
  _odometry = StoredOdometry{
    std::hypot(
      static_cast<double>(msg.twist.twist.linear.x),
      static_cast<double>(msg.twist.twist.linear.y)),
    static_cast<double>(msg.twist.twist.angular.z),
    _time_source(),
  };
}

VisionStatusSnapshot GatewayStateStore::vision_snapshot_locked() const
{
  if (!_has_vision_perf) {
    return VisionStatusSnapshot{};
  }

  const auto age = _time_source() - _vision_perf.updated_at;
  return VisionStatusSnapshot{
    true,
    age > _vision_stale_after,
    _vision_perf.producer_fps,
    _vision_perf.consumer_fps,
    _vision_perf.last_infer_ms,
    _vision_perf.infer_error_count,
    _vision_perf.capture_fatal_error_count,
  };
}

RobotHealthSnapshot GatewayStateStore::robot_health_snapshot_locked(
  const VisionStatusSnapshot & vision) const
{
  if (!_has_odometry) {
    return RobotHealthSnapshot{
      RobotHealthState::Degraded,
      false,
      "waiting for odometry",
      false,
      false,
      0.0,
      0.0,
    };
  }

  const auto odom_age = _time_source() - _odometry.updated_at;
  const bool odom_stale = odom_age > _odom_stale_after;
  if (odom_stale) {
    return RobotHealthSnapshot{
      RobotHealthState::Degraded,
      false,
      "odometry stale",
      true,
      true,
      _odometry.linear_speed_mps,
      _odometry.angular_speed_rad_s,
    };
  }

  if (!vision.available) {
    return RobotHealthSnapshot{
      RobotHealthState::Degraded,
      false,
      "waiting for vision telemetry",
      true,
      false,
      _odometry.linear_speed_mps,
      _odometry.angular_speed_rad_s,
    };
  }

  if (vision.stale) {
    return RobotHealthSnapshot{
      RobotHealthState::Degraded,
      false,
      "vision telemetry stale",
      true,
      false,
      _odometry.linear_speed_mps,
      _odometry.angular_speed_rad_s,
    };
  }

  return RobotHealthSnapshot{
    RobotHealthState::Ok,
    true,
    "robot healthy",
    true,
    false,
    _odometry.linear_speed_mps,
    _odometry.angular_speed_rad_s,
  };
}
} // namespace robot_diag_control_cpp
