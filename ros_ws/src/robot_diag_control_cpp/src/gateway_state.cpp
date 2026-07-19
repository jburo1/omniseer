#include "robot_diag_control_cpp/gateway_state.hpp"

#include <cmath>

namespace robot_diag_control_cpp
{
namespace
{
constexpr std::size_t kMaxOperatorEvents = 8;
} // namespace

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
  const auto health = robot_health_snapshot_locked(vision);
  update_operator_events_locked(vision, health, _preview, _teleop);
  return SystemStatusSnapshot{
    _gateway_name,
    _gateway_version,
    _preview,
    vision,
    health,
    _teleop,
  };
}

DetectionOverlaySnapshot GatewayStateStore::get_detection_overlay() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return detection_overlay_snapshot_locked();
}

std::vector<OperatorEventSnapshot> GatewayStateStore::get_operator_events() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return operator_events_snapshot_locked();
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
  const auto error = std::move(last_error);
  _preview = PreviewStatusSnapshot{
    PreviewState::Disabled,
    profile,
    error,
  };
  if (!error.empty()) {
    append_operator_event_locked("preview error: " + error);
    _last_preview_error_active = true;
  }
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
    static_cast<double>(msg.twist.twist.linear.x),
    static_cast<double>(msg.twist.twist.linear.y),
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
      0,
      0.0,
      0.0,
      0.0,
    };
  }

  const auto odom_age = _time_source() - _odometry.updated_at;
  const auto odom_age_ms = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(odom_age).count());
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
      odom_age_ms,
      _odometry.measured_vx_mps,
      _odometry.measured_vy_mps,
      _odometry.measured_wz_rad_s,
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
      odom_age_ms,
      _odometry.measured_vx_mps,
      _odometry.measured_vy_mps,
      _odometry.measured_wz_rad_s,
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
      odom_age_ms,
      _odometry.measured_vx_mps,
      _odometry.measured_vy_mps,
      _odometry.measured_wz_rad_s,
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
    odom_age_ms,
    _odometry.measured_vx_mps,
    _odometry.measured_vy_mps,
    _odometry.measured_wz_rad_s,
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

std::vector<OperatorEventSnapshot> GatewayStateStore::operator_events_snapshot_locked() const
{
  std::vector<OperatorEventSnapshot> events;
  events.reserve(_operator_events.size());
  const auto now = _time_source();
  for (const auto & event : _operator_events) {
    events.push_back(
      OperatorEventSnapshot{
        event.sequence,
        static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::milliseconds>(now - event.created_at).count()),
        event.message,
      });
  }
  return events;
}

void GatewayStateStore::update_operator_events_locked(
  const VisionStatusSnapshot & vision, const RobotHealthSnapshot & health,
  const PreviewStatusSnapshot & preview, const TeleopStatusSnapshot & teleop) const
{
  const bool odom_fault_active = !health.odom_available || health.odom_stale;
  if (_last_odom_fault_active.has_value() && *_last_odom_fault_active != odom_fault_active) {
    append_operator_event_locked(odom_fault_active ? "odometry lost" : "odometry recovered");
  }
  _last_odom_fault_active = odom_fault_active;

  const bool vision_fault_active = !vision.available || vision.stale;
  if (_last_vision_fault_active.has_value() && *_last_vision_fault_active != vision_fault_active) {
    append_operator_event_locked(vision_fault_active ? "vision stale" : "vision recovered");
  }
  _last_vision_fault_active = vision_fault_active;

  const bool preview_error_active = !preview.last_error.empty();
  if (_last_preview_error_active.has_value() &&
    *_last_preview_error_active != preview_error_active)
  {
    append_operator_event_locked(
      preview_error_active ? "preview error: " + preview.last_error : "preview recovered");
  }
  _last_preview_error_active = preview_error_active;

  const bool teleop_error_active = !teleop.last_error.empty();
  if (_last_teleop_error_active.has_value() && *_last_teleop_error_active != teleop_error_active) {
    append_operator_event_locked(
      teleop_error_active ? "teleop error: " + teleop.last_error : "teleop command recovered");
  }
  _last_teleop_error_active = teleop_error_active;
}

void GatewayStateStore::append_operator_event_locked(std::string message) const
{
  if (_operator_events.size() == kMaxOperatorEvents) {
    _operator_events.erase(_operator_events.begin());
  }
  _operator_events.push_back(
    StoredOperatorEvent{
      _next_operator_event_sequence++,
      _time_source(),
      std::move(message),
    });
}
} // namespace robot_diag_control_cpp
