#include "omniseer_autonomy/target_centering_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace omniseer_autonomy
{
  namespace
  {
    constexpr std::size_t kConsistencyWindow   = 5;
    constexpr int         kConsistencyRequired = 3;
    constexpr double      kTwoPi               = 6.28318530717958647692;

    double distance_sq_or_zero(std::optional<TargetDetection> previous,
                               const TargetDetection&         candidate)
    {
      if (!previous.has_value())
      {
        return 0.0;
      }
      const auto dx = candidate.center_x_px - previous->center_x_px;
      const auto dy = candidate.center_y_px - previous->center_y_px;
      return (dx * dx) + (dy * dy);
    }

    TargetCenteringOutput append_output(TargetCenteringOutput prefix, TargetCenteringOutput suffix)
    {
      prefix.publish_command = suffix.publish_command;
      prefix.linear_x_m_s    = suffix.linear_x_m_s;
      prefix.angular_z_rad_s = suffix.angular_z_rad_s;
      prefix.events.insert(prefix.events.end(), suffix.events.begin(), suffix.events.end());
      return prefix;
    }
  } // namespace

  TargetCenteringController::TargetCenteringController(TargetCenteringConfig config)
      : _config(std::move(config))
  {
    validate_config();
  }

  TargetCenteringOutput TargetCenteringController::update_detections(
      const std::vector<TargetDetection>& detections, double now_sec)
  {
    TargetCenteringOutput output{};
    ensure_started(now_sec, output);
    if (terminal())
    {
      output.publish_command = true;
      output.linear_x_m_s    = 0.0;
      output.angular_z_rad_s = 0.0;
      return output;
    }

    _last_detection_msg_at_sec = now_sec;
    auto target                = select_target(detections);
    push_target_seen(target.has_value());

    if (target.has_value())
    {
      if (!_first_detection_at_sec.has_value())
      {
        _first_detection_at_sec = now_sec;
        output.events.push_back(make_event(now_sec, "first_detection", "", target));
      }
      _last_target_seen_at_sec = now_sec;
      _last_target             = target;
      _target_missing          = false;
    }
    else if (_state != CenteringState::Scan && !_target_missing)
    {
      _target_missing = true;
      ++_target_loss_count;
      output.events.push_back(make_event(now_sec, "target_lost"));
    }

    if (_state == CenteringState::Scan)
    {
      if (target.has_value() && target_consistent())
      {
        transition(CenteringState::Lock, now_sec, output, "target_locked", "", target);
        transition(CenteringState::Center, now_sec, output, "centering_started", "", target);
        return append_output(output, command_for_target(*target, now_sec));
      }
      if (scan_complete())
      {
        return append_output(output, fail(now_sec, "scan_complete_no_target"));
      }
      return append_output(output, command_scan(now_sec));
    }

    if (_state == CenteringState::Lock || _state == CenteringState::Center ||
        _state == CenteringState::Frame)
    {
      if (target.has_value())
      {
        return append_output(output, command_for_target(*target, now_sec));
      }
      return append_output(output, command_zero(now_sec));
    }

    return append_output(output, command_zero(now_sec));
  }

  void TargetCenteringController::update_heading(double heading_rad)
  {
    if (terminal() || _state != CenteringState::Scan || !_last_detection_msg_at_sec.has_value())
    {
      _last_scan_heading_rad = std::nullopt;
      return;
    }

    if (!_last_scan_heading_rad.has_value())
    {
      _last_scan_heading_rad = heading_rad;
      return;
    }

    const auto delta = std::atan2(std::sin(heading_rad - *_last_scan_heading_rad),
                                  std::cos(heading_rad - *_last_scan_heading_rad));
    _scan_yaw_travel_rad += std::abs(delta);
    _last_scan_heading_rad = heading_rad;
  }

  TargetCenteringOutput TargetCenteringController::tick(double now_sec)
  {
    TargetCenteringOutput output{};
    ensure_started(now_sec, output);
    if (terminal())
    {
      output.publish_command = true;
      output.linear_x_m_s    = 0.0;
      output.angular_z_rad_s = 0.0;
      return output;
    }

    if (!_last_detection_msg_at_sec.has_value())
    {
      return append_output(output, command_zero(now_sec));
    }

    if ((now_sec - *_last_detection_msg_at_sec) > _config.detection_stale_sec)
    {
      return append_output(output, fail(now_sec, "stale_detections"));
    }

    if (_state == CenteringState::Scan)
    {
      if (scan_complete())
      {
        return append_output(output, fail(now_sec, "scan_complete_no_target"));
      }
      return append_output(output, command_scan(now_sec));
    }

    if (_state == CenteringState::Lock || _state == CenteringState::Center ||
        _state == CenteringState::Frame)
    {
      if (_last_target_seen_at_sec.has_value() &&
          (now_sec - *_last_target_seen_at_sec) > _config.target_lost_timeout_sec)
      {
        return append_output(output, begin_reacquire(now_sec, "target_lost_timeout"));
      }
      if (_last_target.has_value())
      {
        return append_output(output, command_for_target(*_last_target, now_sec));
      }
    }

    return append_output(output, command_zero(now_sec));
  }

  CenteringState TargetCenteringController::state() const noexcept
  {
    return _state;
  }

  std::string TargetCenteringController::terminal_reason() const
  {
    return _terminal_reason;
  }

  int TargetCenteringController::target_loss_count() const noexcept
  {
    return _target_loss_count;
  }

  std::optional<double> TargetCenteringController::time_to_first_detection_sec() const noexcept
  {
    if (!_first_detection_at_sec.has_value())
    {
      return std::nullopt;
    }
    return *_first_detection_at_sec - _started_at_sec;
  }

  std::optional<double> TargetCenteringController::time_to_centered_sec() const noexcept
  {
    if (!_centered_at_sec.has_value())
    {
      return std::nullopt;
    }
    return *_centered_at_sec - _started_at_sec;
  }

  std::optional<double> TargetCenteringController::final_error() const noexcept
  {
    return _final_error;
  }

  std::optional<double> TargetCenteringController::final_confidence() const noexcept
  {
    return _final_confidence;
  }

  void TargetCenteringController::update_proximity_range(std::optional<double> range_m)
  {
    _last_proximity_range_m = range_m;
  }

  void TargetCenteringController::validate_config() const
  {
    if (_config.target_class.empty())
    {
      throw std::invalid_argument("target_class must not be empty");
    }
    if (_config.image_width_px <= 0.0)
    {
      throw std::invalid_argument("image_width_px must be positive");
    }
    if (_config.image_height_px <= 0.0)
    {
      throw std::invalid_argument("image_height_px must be positive");
    }
    if (_config.scan_yaw_rate_rad_s <= 0.0)
    {
      throw std::invalid_argument("scan_yaw_rate_rad_s must be positive");
    }
    if (_config.max_yaw_rate_rad_s <= 0.0)
    {
      throw std::invalid_argument("max_yaw_rate_rad_s must be positive");
    }
    if (_config.min_yaw_rate_rad_s < 0.0 || _config.min_yaw_rate_rad_s > _config.max_yaw_rate_rad_s)
    {
      throw std::invalid_argument("min_yaw_rate_rad_s must be in [0, max_yaw_rate_rad_s]");
    }
    if (_config.kp <= 0.0)
    {
      throw std::invalid_argument("kp must be positive");
    }
    if (_config.center_deadband < 0.0)
    {
      throw std::invalid_argument("center_deadband must be non-negative");
    }
    if (_config.bbox_area_min_ratio < 0.0)
    {
      throw std::invalid_argument("bbox_area_min_ratio must be non-negative");
    }
    if (_config.bbox_area_max_ratio <= _config.bbox_area_min_ratio)
    {
      throw std::invalid_argument("bbox_area_max_ratio must be greater than bbox_area_min_ratio");
    }
    if (_config.forward_speed_m_s <= 0.0)
    {
      throw std::invalid_argument("forward_speed_m_s must be positive");
    }
    if (_config.reverse_speed_m_s <= 0.0)
    {
      throw std::invalid_argument("reverse_speed_m_s must be positive");
    }
    if (_config.stable_framed_frames <= 0)
    {
      throw std::invalid_argument("stable_framed_frames must be positive");
    }
    if (_config.proximity_stop_m <= 0.0)
    {
      throw std::invalid_argument("proximity_stop_m must be positive");
    }
    if (_config.detection_stale_sec <= 0.0)
    {
      throw std::invalid_argument("detection_stale_sec must be positive");
    }
    if (_config.scan_limit_revolutions <= 0.0)
    {
      throw std::invalid_argument("scan_limit_revolutions must be positive");
    }
    if (_config.target_lost_timeout_sec <= 0.0)
    {
      throw std::invalid_argument("target_lost_timeout_sec must be positive");
    }
  }

  void TargetCenteringController::ensure_started(double now_sec, TargetCenteringOutput& output)
  {
    if (_started)
    {
      return;
    }
    _started              = true;
    _started_at_sec       = now_sec;
    _state_started_at_sec = now_sec;
    output.events.push_back(make_event(now_sec, "started"));
  }

  void TargetCenteringController::transition(CenteringState state, double now_sec,
                                             TargetCenteringOutput& output, std::string event,
                                             std::string                    reason,
                                             std::optional<TargetDetection> target)
  {
    _state                = state;
    _state_started_at_sec = now_sec;
    output.events.push_back(make_event(now_sec, std::move(event), std::move(reason),
                                       target.has_value() ? std::move(target) : _last_target));
  }

  TargetCenteringOutput TargetCenteringController::fail(double now_sec, std::string reason)
  {
    TargetCenteringOutput output{};
    _terminal_reason = reason;
    transition(CenteringState::Failed, now_sec, output, "failed", std::move(reason));
    output.publish_command = true;
    output.linear_x_m_s    = 0.0;
    output.angular_z_rad_s = 0.0;
    return output;
  }

  TargetCenteringOutput TargetCenteringController::begin_reacquire(double      now_sec,
                                                                   std::string reason)
  {
    TargetCenteringOutput output{};
    if (scan_complete())
    {
      return fail(now_sec, "scan_complete_no_target");
    }

    _stable_frames = 0;
    _last_target.reset();
    _target_missing = false;
    _recent_target_seen.clear();
    transition(CenteringState::Scan, now_sec, output, "reacquire_started", std::move(reason));
    return append_output(output, command_scan(now_sec));
  }

  TargetCenteringOutput TargetCenteringController::command_scan(double now_sec)
  {
    TargetCenteringOutput output{};
    const auto            yaw = std::min(_config.scan_yaw_rate_rad_s, _config.max_yaw_rate_rad_s);
    output.publish_command    = true;
    output.linear_x_m_s       = 0.0;
    output.angular_z_rad_s    = yaw;
    output.events.push_back(
        make_event(now_sec, "command", "", std::nullopt, std::nullopt, std::nullopt, 0.0, yaw));
    return output;
  }

  TargetCenteringOutput TargetCenteringController::command_zero(double now_sec)
  {
    TargetCenteringOutput output{};
    output.publish_command = true;
    output.linear_x_m_s    = 0.0;
    output.angular_z_rad_s = 0.0;
    output.events.push_back(
        make_event(now_sec, "command", "", _last_target, std::nullopt, std::nullopt, 0.0, 0.0));
    return output;
  }

  TargetCenteringOutput TargetCenteringController::command_for_target(const TargetDetection& target,
                                                                      double now_sec)
  {
    TargetCenteringOutput output{};
    const auto            error      = normalized_error(target.center_x_px);
    const auto            area_ratio = bbox_area_ratio(target);
    const auto            centered   = std::abs(error) <= _config.center_deadband;
    const auto            framed =
        area_ratio >= _config.bbox_area_min_ratio && area_ratio <= _config.bbox_area_max_ratio;

    double yaw = 0.0;
    if (centered)
    {
      if (!_centered_at_sec.has_value())
      {
        _centered_at_sec = now_sec;
        output.events.push_back(
            make_event(now_sec, "centered_first_frame", "", target, error, area_ratio));
      }
    }
    else
    {
      yaw = clamp_yaw(-_config.kp * error);
    }

    double linear_x = 0.0;
    if (area_ratio < _config.bbox_area_min_ratio)
    {
      if (proximity_blocks_forward())
      {
        return fail(now_sec, "proximity_blocked");
      }
      linear_x = _config.forward_speed_m_s;
    }
    else if (area_ratio > _config.bbox_area_max_ratio)
    {
      linear_x = -_config.reverse_speed_m_s;
    }

    if (_state == CenteringState::Center && centered)
    {
      transition(CenteringState::Frame, now_sec, output, "framing_started", "", target);
    }
    else if (_state == CenteringState::Frame && !centered)
    {
      transition(CenteringState::Center, now_sec, output, "centering_resumed", "", target);
    }

    if (centered && framed)
    {
      ++_stable_frames;
    }
    else
    {
      _stable_frames = 0;
    }

    _last_target      = target;
    _final_error      = error;
    _final_confidence = target.confidence;

    if (_stable_frames >= _config.stable_framed_frames)
    {
      _terminal_reason = "framed";
      transition(CenteringState::Success, now_sec, output, "succeeded", _terminal_reason);
      linear_x = 0.0;
      yaw      = 0.0;
    }

    output.publish_command = true;
    output.linear_x_m_s    = linear_x;
    output.angular_z_rad_s = yaw;
    output.events.push_back(
        make_event(now_sec, "command", "", target, error, area_ratio, linear_x, yaw));
    return output;
  }

  std::optional<TargetDetection> TargetCenteringController::select_target(
      const std::vector<TargetDetection>& detections) const
  {
    std::optional<TargetDetection> selected{};
    for (const auto& detection : detections)
    {
      if (detection.class_name != _config.target_class)
      {
        continue;
      }
      if (!selected.has_value())
      {
        selected = detection;
        continue;
      }
      if (_state == CenteringState::Scan)
      {
        if (detection.confidence > selected->confidence)
        {
          selected = detection;
        }
        continue;
      }
      const auto selected_distance  = distance_sq_or_zero(_last_target, *selected);
      const auto candidate_distance = distance_sq_or_zero(_last_target, detection);
      if (candidate_distance < selected_distance)
      {
        selected = detection;
      }
    }
    return selected;
  }

  void TargetCenteringController::push_target_seen(bool seen)
  {
    _recent_target_seen.push_back(seen);
    while (_recent_target_seen.size() > kConsistencyWindow)
    {
      _recent_target_seen.pop_front();
    }
  }

  bool TargetCenteringController::target_consistent() const
  {
    return std::count(_recent_target_seen.begin(), _recent_target_seen.end(), true) >=
           kConsistencyRequired;
  }

  bool TargetCenteringController::scan_complete() const
  {
    return _scan_yaw_travel_rad >= (_config.scan_limit_revolutions * kTwoPi);
  }

  double TargetCenteringController::normalized_error(double center_x_px) const
  {
    const auto half_width = _config.image_width_px / 2.0;
    return (center_x_px - half_width) / half_width;
  }

  double TargetCenteringController::bbox_area_ratio(const TargetDetection& target) const
  {
    const auto width_ratio  = std::max(0.0, target.size_x_px) / _config.image_width_px;
    const auto height_ratio = std::max(0.0, target.size_y_px) / _config.image_height_px;
    return width_ratio * height_ratio;
  }

  double TargetCenteringController::clamp_yaw(double yaw_rad_s) const
  {
    auto yaw =
        std::max(-_config.max_yaw_rate_rad_s, std::min(_config.max_yaw_rate_rad_s, yaw_rad_s));
    if (std::abs(yaw) > 0.0 && std::abs(yaw) < _config.min_yaw_rate_rad_s)
    {
      yaw = std::copysign(_config.min_yaw_rate_rad_s, yaw);
    }
    return yaw;
  }

  bool TargetCenteringController::proximity_blocks_forward() const noexcept
  {
    return _last_proximity_range_m.has_value() &&
           *_last_proximity_range_m <= _config.proximity_stop_m;
  }

  bool TargetCenteringController::terminal() const noexcept
  {
    return _state == CenteringState::Success || _state == CenteringState::Failed;
  }

  TargetCenteringEvent TargetCenteringController::make_event(
      double now_sec, std::string event, std::string reason, std::optional<TargetDetection> target,
      std::optional<double> error, std::optional<double> area_ratio, double linear_x_m_s,
      double angular_z_rad_s) const
  {
    return TargetCenteringEvent{
        now_sec,           _state,          std::move(event), std::move(reason),
        std::move(target), error,           area_ratio,       _last_proximity_range_m,
        linear_x_m_s,      angular_z_rad_s, _stable_frames,   _target_loss_count,
    };
  }

  std::string state_name(CenteringState state)
  {
    switch (state)
    {
    case CenteringState::Scan:
      return "scan";
    case CenteringState::Lock:
      return "lock";
    case CenteringState::Center:
      return "center";
    case CenteringState::Frame:
      return "frame";
    case CenteringState::Success:
      return "success";
    case CenteringState::Failed:
      return "failed";
    }
    return "unknown";
  }
} // namespace omniseer_autonomy
