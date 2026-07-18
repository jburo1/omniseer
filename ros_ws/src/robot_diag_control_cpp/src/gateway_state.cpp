#include "robot_diag_control_cpp/gateway_state.hpp"

#include <cmath>

namespace robot_diag_control_cpp
{
GatewayStateStore::GatewayStateStore(
  std::string gateway_name, std::string gateway_version,
  std::chrono::milliseconds vision_stale_after, std::chrono::milliseconds odom_stale_after,
  TimeSource time_source,
  std::chrono::milliseconds detections_stale_after, uint32_t detection_source_width_px,
  uint32_t detection_source_height_px)
: _gateway_name(std::move(gateway_name)),
  _gateway_version(std::move(gateway_version)),
  _vision_stale_after(vision_stale_after),
  _odom_stale_after(odom_stale_after),
  _detections_stale_after(detections_stale_after),
  _detection_source_width_px(detection_source_width_px),
  _detection_source_height_px(detection_source_height_px),
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
    _teleop,
  };
}

DetectionOverlaySnapshot GatewayStateStore::get_detection_overlay() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return detection_overlay_snapshot_locked();
}

PreviewStatusSnapshot GatewayStateStore::get_preview_status() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _preview;
}

TeleopStatusSnapshot GatewayStateStore::get_teleop_status() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _teleop;
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

void GatewayStateStore::set_teleop_status(const TeleopStatusSnapshot & teleop)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _teleop = teleop;
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

void GatewayStateStore::update_detections(const yolo_msgs::msg::DetectionArray & msg)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _has_detections = true;
  _detections.detections.clear();
  _detections.detections.reserve(msg.detections.size());

  for (const auto & detection : msg.detections) {
    _detections.detections.push_back(
      DetectionOverlayItem{
        detection.class_id,
        detection.class_name,
        detection.score,
        detection.id,
        detection.bbox.center.position.x,
        detection.bbox.center.position.y,
        detection.bbox.size.x,
        detection.bbox.size.y,
      });
  }

  _detections.updated_at = _time_source();
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

DetectionOverlaySnapshot GatewayStateStore::detection_overlay_snapshot_locked() const
{
  if (!_has_detections) {
    return DetectionOverlaySnapshot{
      false,
      false,
      0,
      _detection_source_width_px,
      _detection_source_height_px,
      {},
    };
  }

  const auto age = _time_source() - _detections.updated_at;
  return DetectionOverlaySnapshot{
    true,
    age > _detections_stale_after,
    static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(age).count()),
    _detection_source_width_px,
    _detection_source_height_px,
    _detections.detections,
  };
}
} // namespace robot_diag_control_cpp
