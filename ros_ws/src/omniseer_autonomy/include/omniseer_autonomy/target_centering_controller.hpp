#pragma once

#include <deque>
#include <optional>
#include <string>
#include <vector>

namespace omniseer_autonomy
{
enum class CenteringState
{
  Scan,
  Acquire,
  Center,
  Success,
  Failed,
};

struct TargetCenteringConfig
{
  std::string target_class{};
  double      image_width_px{1280.0};
  double      scan_yaw_rate_rad_s{0.20};
  double      max_yaw_rate_rad_s{0.30};
  double      min_yaw_rate_rad_s{0.08};
  double      kp{0.30};
  double      center_deadband{0.05};
  int         stable_center_frames{10};
  double      detection_stale_sec{0.5};
  double      scan_timeout_sec{12.0};
  double      target_lost_timeout_sec{0.5};
};

struct TargetDetection
{
  std::string class_name{};
  double      confidence{0.0};
  double      center_x_px{0.0};
  double      center_y_px{0.0};
  double      size_x_px{0.0};
  double      size_y_px{0.0};
};

struct TargetCenteringEvent
{
  double         time_sec{0.0};
  CenteringState state{CenteringState::Scan};
  std::string    event{};
  std::string    reason{};
  std::optional<TargetDetection> target{};
  std::optional<double> normalized_error{};
  double                angular_z_rad_s{0.0};
  int                   stable_center_frames{0};
  int                   target_loss_count{0};
};

struct TargetCenteringOutput
{
  bool                              publish_command{false};
  double                            angular_z_rad_s{0.0};
  std::vector<TargetCenteringEvent> events{};
};

class TargetCenteringController
{
public:
  explicit TargetCenteringController(TargetCenteringConfig config);

  TargetCenteringOutput update_detections(
    const std::vector<TargetDetection> & detections, double now_sec);
  TargetCenteringOutput tick(double now_sec);

  CenteringState state() const noexcept;
  std::string terminal_reason() const;
  int target_loss_count() const noexcept;
  std::optional<double> time_to_first_detection_sec() const noexcept;
  std::optional<double> time_to_centered_sec() const noexcept;
  std::optional<double> final_error() const noexcept;
  std::optional<double> final_confidence() const noexcept;

private:
  void validate_config() const;
  void ensure_started(double now_sec, TargetCenteringOutput & output);
  void transition(
    CenteringState state, double now_sec, TargetCenteringOutput & output,
    std::string event, std::string reason = "",
    std::optional<TargetDetection> target = std::nullopt);
  TargetCenteringOutput fail(double now_sec, std::string reason);
  TargetCenteringOutput command_scan(double now_sec);
  TargetCenteringOutput command_zero(double now_sec);
  TargetCenteringOutput command_for_target(const TargetDetection & target, double now_sec);
  std::optional<TargetDetection> select_target(
    const std::vector<TargetDetection> & detections) const;
  void push_target_seen(bool seen);
  bool target_consistent() const;
  double normalized_error(double center_x_px) const;
  double clamp_yaw(double yaw_rad_s) const;
  bool terminal() const noexcept;
  TargetCenteringEvent make_event(
    double now_sec, std::string event, std::string reason = "",
    std::optional<TargetDetection> target = std::nullopt,
    std::optional<double> normalized_error = std::nullopt,
    double angular_z_rad_s = 0.0) const;

  TargetCenteringConfig _config{};
  CenteringState        _state{CenteringState::Scan};
  bool                  _started{false};
  double                _started_at_sec{0.0};
  double                _state_started_at_sec{0.0};
  std::optional<double> _last_detection_msg_at_sec{};
  std::optional<double> _first_detection_at_sec{};
  std::optional<double> _centered_at_sec{};
  std::optional<double> _last_target_seen_at_sec{};
  std::optional<TargetDetection> _last_target{};
  std::optional<double> _final_error{};
  std::optional<double> _final_confidence{};
  std::deque<bool> _recent_target_seen{};
  int                   _stable_frames{0};
  int                   _target_loss_count{0};
  bool                  _target_missing{false};
  std::string           _terminal_reason{};
};

std::string state_name(CenteringState state);
} // namespace omniseer_autonomy
