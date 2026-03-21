#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "nav_msgs/msg/odometry.hpp"
#include "omniseer_msgs/msg/vision_perf_summary.hpp"

namespace robot_diag_control_cpp
{
enum class PreviewState
{
  Disabled,
  Running,
};

enum class PreviewProfile
{
  LowBw,
  Balanced,
  HighQuality,
};

enum class RobotHealthState
{
  Ok,
  Degraded,
};

struct PreviewStatusSnapshot
{
  PreviewState   state{PreviewState::Disabled};
  PreviewProfile profile{PreviewProfile::Balanced};
  std::string    last_error{};
};

struct VisionStatusSnapshot
{
  bool     available{false};
  bool     stale{false};
  double   producer_fps{0.0};
  double   consumer_fps{0.0};
  double   last_infer_ms{0.0};
  uint64_t infer_error_count{0};
  uint64_t capture_fatal_error_count{0};
};

struct RobotHealthSnapshot
{
  RobotHealthState state{RobotHealthState::Degraded};
  bool             ready{false};
  std::string      summary{"waiting for odometry"};
  bool             odom_available{false};
  bool             odom_stale{false};
  double           linear_speed_mps{0.0};
  double           angular_speed_rad_s{0.0};
};

struct SystemStatusSnapshot
{
  std::string          gateway_name{};
  std::string          gateway_version{};
  PreviewStatusSnapshot preview{};
  VisionStatusSnapshot  vision{};
  RobotHealthSnapshot   health{};
};

class GatewayStateStore
{
public:
  using SteadyTimePoint = std::chrono::time_point<std::chrono::steady_clock>;
  using TimeSource = std::function<SteadyTimePoint()>;

  explicit GatewayStateStore(
    std::string gateway_name = "robot_diag_control_cpp",
    std::string gateway_version = "0.1.0",
    std::chrono::milliseconds vision_stale_after = std::chrono::milliseconds(2000),
    std::chrono::milliseconds odom_stale_after = std::chrono::milliseconds(1000),
    TimeSource time_source = std::chrono::steady_clock::now);

  SystemStatusSnapshot get_system_status() const;
  PreviewStatusSnapshot get_preview_status() const;
  PreviewStatusSnapshot set_preview_running(PreviewProfile profile);
  PreviewStatusSnapshot set_preview_disabled(
    PreviewProfile profile, std::string last_error = "");
  void update_vision_perf(const omniseer_msgs::msg::VisionPerfSummary & msg);
  void update_odometry(const nav_msgs::msg::Odometry & msg);

private:
  struct StoredVisionPerf
  {
    double          producer_fps{0.0};
    double          consumer_fps{0.0};
    double          last_infer_ms{0.0};
    uint64_t        infer_error_count{0};
    uint64_t        capture_fatal_error_count{0};
    SteadyTimePoint updated_at{};
  };

  struct StoredOdometry
  {
    double          linear_speed_mps{0.0};
    double          angular_speed_rad_s{0.0};
    SteadyTimePoint updated_at{};
  };

  VisionStatusSnapshot vision_snapshot_locked() const;
  RobotHealthSnapshot robot_health_snapshot_locked(const VisionStatusSnapshot & vision) const;

  mutable std::mutex          _mutex{};
  std::string                 _gateway_name{};
  std::string                 _gateway_version{};
  std::chrono::milliseconds   _vision_stale_after{2000};
  std::chrono::milliseconds   _odom_stale_after{1000};
  TimeSource                  _time_source{};
  PreviewStatusSnapshot       _preview{};
  bool                        _has_vision_perf{false};
  StoredVisionPerf            _vision_perf{};
  bool                        _has_odometry{false};
  StoredOdometry              _odometry{};
};
} // namespace robot_diag_control_cpp
