#include "omniseer_autonomy/target_centering_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace omniseer_autonomy
{
  namespace
  {
    constexpr int    kConsistencyRequired = 3;
    constexpr double kTwoPi               = 6.28318530717958647692;

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
    auto        target         = select_target(detections);
    std::string target_rejection_reason{};
    if (_state == CenteringState::Scan)
    {
      if (target.has_value() && (!_acquisition_candidate.has_value() ||
                                 center_jump_within_limit(*_acquisition_candidate, *target)))
      {
        _acquisition_candidate = target;
        ++_acquisition_frames;
      }
      else if (target.has_value())
      {
        _acquisition_candidate = target;
        _acquisition_frames    = 1;
      }
      else
      {
        _acquisition_candidate.reset();
        _acquisition_frames = 0;
      }
    }
    else if (target.has_value() && _last_target.has_value() &&
             !center_jump_within_limit(*_last_target, *target))
    {
      target.reset();
      target_rejection_reason = "max_target_center_jump_rejected";
    }

    if (target.has_value())
    {
      if (!_first_detection_at_sec.has_value())
      {
        _first_detection_at_sec = now_sec;
        output.events.push_back(make_event(now_sec, "first_detection", "", target));
      }
      _last_target_seen_at_sec = now_sec;
      _target_missing          = false;
    }
    else if (_state != CenteringState::Scan && !_target_missing)
    {
      _approach_holding = false;
      _target_missing   = true;
      ++_target_loss_count;
      output.events.push_back(make_event(now_sec, "target_lost",
                                         target_rejection_reason.empty()
                                             ? "no_valid_target_detection"
                                             : target_rejection_reason));
    }

    if (!target.has_value())
    {
      record_confirmation_miss(now_sec, output,
                               target_rejection_reason.empty() ? "no_valid_target_detection"
                                                               : target_rejection_reason);
    }

    if (_state == CenteringState::Scan)
    {
      if (target.has_value() && _acquisition_frames >= kConsistencyRequired)
      {
        transition(CenteringState::Lock, now_sec, output, "target_locked", "", target);
        transition(CenteringState::Center, now_sec, output, "centering_started", "", target);
        update_approach_hysteresis(*target);
        update_visual_state(*target, now_sec, output);
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
        update_approach_hysteresis(*target);
        update_visual_state(*target, now_sec, output);
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
    if (_config.approach_stop_area_ratio < _config.bbox_area_min_ratio ||
        _config.approach_stop_area_ratio > _config.bbox_area_max_ratio)
    {
      throw std::invalid_argument(
          "approach_stop_area_ratio must be in [bbox_area_min_ratio, bbox_area_max_ratio]");
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
    if (_config.success_miss_tolerance_updates < 0)
    {
      throw std::invalid_argument("success_miss_tolerance_updates must be non-negative");
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
    if (_config.min_target_confidence < 0.0 || _config.min_target_confidence > 1.0)
    {
      throw std::invalid_argument("min_target_confidence must be in [0, 1]");
    }
    if (_config.max_target_center_jump_ratio <= 0.0 || _config.max_target_center_jump_ratio > 1.0)
    {
      throw std::invalid_argument("max_target_center_jump_ratio must be in (0, 1]");
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
    reset_success_confirmation();
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
    _approach_holding = false;
    _terminal_reason  = reason;
    transition(CenteringState::Failed, now_sec, output, "failed", std::move(reason));
    reset_success_confirmation();
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

    reset_success_confirmation();
    _approach_holding = false;
    _last_target.reset();
    _target_missing = false;
    _acquisition_candidate.reset();
    _acquisition_frames = 0;
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

    double yaw = 0.0;
    if (!centered)
    {
      yaw = clamp_yaw(-_config.kp * error);
    }

    double linear_x = 0.0;
    if (area_ratio > _config.bbox_area_max_ratio)
    {
      linear_x = -_config.reverse_speed_m_s;
    }
    else if (!_approach_holding)
    {
      if (proximity_blocks_forward())
      {
        return fail(now_sec, "proximity_blocked");
      }
      linear_x = _config.forward_speed_m_s;
    }

    if (_state == CenteringState::Success)
    {
      linear_x = 0.0;
      yaw      = 0.0;
    }

    _last_target = target;

    output.publish_command = true;
    output.linear_x_m_s    = linear_x;
    output.angular_z_rad_s = yaw;
    output.events.push_back(
        make_event(now_sec, "command", "", target, error, area_ratio, linear_x, yaw));
    return output;
  }

  void TargetCenteringController::update_approach_hysteresis(const TargetDetection& target)
  {
    const auto area_ratio = bbox_area_ratio(target);
    if (_approach_holding && area_ratio < _config.bbox_area_min_ratio)
    {
      _approach_holding = false;
      reset_success_confirmation();
    }
    else if (!_approach_holding && area_ratio >= _config.approach_stop_area_ratio)
    {
      _approach_holding = true;
    }
  }

  void TargetCenteringController::update_visual_state(const TargetDetection& target, double now_sec,
                                                      TargetCenteringOutput& output)
  {
    const auto error      = normalized_error(target.center_x_px);
    const auto area_ratio = bbox_area_ratio(target);
    const auto centered   = std::abs(error) <= _config.center_deadband;
    const auto framed     = _approach_holding && area_ratio >= _config.bbox_area_min_ratio &&
                        area_ratio <= _config.bbox_area_max_ratio;

    if (centered && !_centered_at_sec.has_value())
    {
      _centered_at_sec = now_sec;
      output.events.push_back(
          make_event(now_sec, "centered_first_frame", "", target, error, area_ratio));
    }

    if (_state == CenteringState::Center && centered)
    {
      transition(CenteringState::Frame, now_sec, output, "framing_started", "", target);
    }
    else if (_state == CenteringState::Frame && !centered)
    {
      transition(CenteringState::Center, now_sec, output, "centering_resumed", "", target);
      reset_success_confirmation();
    }

    if (centered && framed)
    {
      ++_stable_frames;
      _confirmation_miss_count = 0;
    }
    else
    {
      reset_success_confirmation();
    }
    _final_error      = error;
    _final_confidence = target.confidence;

    if (_stable_frames >= _config.stable_framed_frames)
    {
      _approach_holding = false;
      _terminal_reason  = "framed";
      transition(CenteringState::Success, now_sec, output, "succeeded", _terminal_reason);
      reset_success_confirmation();
    }
  }

  void TargetCenteringController::record_confirmation_miss(double                 now_sec,
                                                           TargetCenteringOutput& output,
                                                           const std::string&     reason)
  {
    if (_stable_frames == 0)
    {
      return;
    }

    ++_confirmation_miss_count;
    output.events.push_back(make_event(now_sec, "confirmation_interrupted", reason));
    if (_confirmation_miss_count > _config.success_miss_tolerance_updates)
    {
      reset_success_confirmation();
    }
  }

  void TargetCenteringController::reset_success_confirmation() noexcept
  {
    _stable_frames           = 0;
    _confirmation_miss_count = 0;
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
      if (detection.confidence < _config.min_target_confidence)
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

  bool TargetCenteringController::center_jump_within_limit(const TargetDetection& previous,
                                                           const TargetDetection& candidate) const
  {
    const auto diagonal = std::hypot(_config.image_width_px, _config.image_height_px);
    return std::sqrt(distance_sq_or_zero(previous, candidate)) / diagonal <=
           _config.max_target_center_jump_ratio;
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
        now_sec,
        _state,
        std::move(event),
        std::move(reason),
        std::move(target),
        error,
        area_ratio,
        _last_proximity_range_m,
        linear_x_m_s,
        angular_z_rad_s,
        _stable_frames,
        _confirmation_miss_count,
        _target_loss_count,
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
