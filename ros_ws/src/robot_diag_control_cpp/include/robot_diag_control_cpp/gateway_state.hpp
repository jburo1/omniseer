#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "omniseer_msgs/msg/vision_perf_summary.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "yolo_msgs/msg/detection_array.hpp"

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
  uint64_t         odom_age_ms{0};
  double           measured_vx_mps{0.0};
  double           measured_vy_mps{0.0};
  double           measured_wz_rad_s{0.0};
};

enum class TeleopState
{
  Disabled,
  Enabled,
  TimedOut,
};

struct TeleopStatusSnapshot
{
  TeleopState state{TeleopState::Disabled};
  bool        enabled{false};
  bool        timed_out{false};
  uint64_t    last_command_age_ms{0};
  double      max_linear_mps{0.35};
  double      max_angular_rad_s{0.8};
  std::string last_error{};
  double      last_command_vx_mps{0.0};
  double      last_command_vy_mps{0.0};
  double      last_command_wz_rad_s{0.0};
};

struct DetectionOverlayItem
{
  int32_t     class_id{0};
  std::string class_name{};
  double      score{0.0};
  std::string track_id{};
  double      bbox_center_x_px{0.0};
  double      bbox_center_y_px{0.0};
  double      bbox_width_px{0.0};
  double      bbox_height_px{0.0};
};

struct DetectionOverlaySnapshot
{
  bool                              available{false};
  bool                              stale{false};
  uint64_t                          age_ms{0};
  uint32_t                          source_width_px{1280};
  uint32_t                          source_height_px{720};
  std::vector<DetectionOverlayItem> detections{};
};

struct ComputeStatusSnapshot
{
  bool     available{false};
  bool     stale{false};
  uint64_t age_ms{0};
  double   cpu_percent{0.0};
  bool     cpu_temperature_available{false};
  double   cpu_temperature_c{0.0};
  bool     thermal_throttled_available{false};
  bool     thermal_throttled{false};
  uint64_t ram_used_bytes{0};
  uint64_t ram_total_bytes{0};
  double   ram_used_percent{0.0};
  bool     disk_available{false};
  double   disk_used_percent{0.0};
};

struct NetworkStatusSnapshot
{
  bool        available{false};
  bool        stale{false};
  uint64_t    age_ms{0};
  bool        connected{false};
  std::string interface_name{};
  bool        wifi_signal_available{false};
  int32_t     wifi_signal_dbm{0};
  bool        link_quality_available{false};
  uint32_t    link_quality_percent{0};
};

struct BatteryStatusSnapshot
{
  bool        available{false};
  bool        stale{false};
  uint64_t    age_ms{0};
  std::string source{};
  bool        present{false};
  bool        voltage_available{false};
  double      voltage{0.0};
  bool        percentage_available{false};
  double      percentage{0.0};
  bool        charging_available{false};
  bool        charging{false};
};

struct PowerStatusSnapshot
{
  BatteryStatusSnapshot lipo_battery{};
  BatteryStatusSnapshot onboard_battery{};
};

struct PlatformStatusSnapshot
{
  ComputeStatusSnapshot compute{};
  NetworkStatusSnapshot network{};
  PowerStatusSnapshot   power{};
};

struct OperatorEventSnapshot
{
  uint64_t    sequence{0};
  uint64_t    age_ms{0};
  std::string message{};
};

struct SystemStatusSnapshot
{
  std::string          gateway_name{};
  std::string          gateway_version{};
  PreviewStatusSnapshot preview{};
  VisionStatusSnapshot  vision{};
  RobotHealthSnapshot   health{};
  TeleopStatusSnapshot  teleop{};
  PlatformStatusSnapshot platform{};
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
    TimeSource time_source = std::chrono::steady_clock::now,
    std::chrono::milliseconds detections_stale_after = std::chrono::milliseconds(500),
    uint32_t detection_source_width_px = 1280,
    uint32_t detection_source_height_px = 720);

  SystemStatusSnapshot get_system_status() const;
  DetectionOverlaySnapshot get_detection_overlay() const;
  std::vector<OperatorEventSnapshot> get_operator_events() const;
  PreviewStatusSnapshot get_preview_status() const;
  PreviewStatusSnapshot set_preview_running(PreviewProfile profile);
  PreviewStatusSnapshot set_preview_disabled(
    PreviewProfile profile, std::string last_error = "");
  TeleopStatusSnapshot get_teleop_status() const;
  void set_teleop_status(const TeleopStatusSnapshot & teleop);
  void update_vision_perf(const omniseer_msgs::msg::VisionPerfSummary & msg);
  void update_odometry(const nav_msgs::msg::Odometry & msg);
  void update_detections(const yolo_msgs::msg::DetectionArray & msg);
  void update_lipo_battery(const sensor_msgs::msg::BatteryState & msg);
  void update_compute_status(const ComputeStatusSnapshot & snapshot);
  void update_network_status(const NetworkStatusSnapshot & snapshot);
  void update_onboard_battery(const BatteryStatusSnapshot & snapshot);

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
    double          measured_vx_mps{0.0};
    double          measured_vy_mps{0.0};
    double          measured_wz_rad_s{0.0};
    SteadyTimePoint updated_at{};
  };

  struct StoredDetectionOverlay
  {
    std::vector<DetectionOverlayItem> detections{};
    SteadyTimePoint                   updated_at{};
  };

  struct StoredComputeStatus
  {
    ComputeStatusSnapshot snapshot{};
    SteadyTimePoint      updated_at{};
  };

  struct StoredNetworkStatus
  {
    NetworkStatusSnapshot snapshot{};
    SteadyTimePoint       updated_at{};
  };

  struct StoredBatteryStatus
  {
    BatteryStatusSnapshot snapshot{};
    SteadyTimePoint       updated_at{};
  };

  struct StoredOperatorEvent
  {
    uint64_t        sequence{0};
    SteadyTimePoint created_at{};
    std::string     message{};
  };

  VisionStatusSnapshot vision_snapshot_locked() const;
  RobotHealthSnapshot robot_health_snapshot_locked(const VisionStatusSnapshot & vision) const;
  DetectionOverlaySnapshot detection_overlay_snapshot_locked() const;
  PlatformStatusSnapshot platform_snapshot_locked() const;
  ComputeStatusSnapshot compute_snapshot_locked() const;
  NetworkStatusSnapshot network_snapshot_locked() const;
  BatteryStatusSnapshot battery_snapshot_locked(
    const StoredBatteryStatus & stored, bool has_battery) const;
  std::vector<OperatorEventSnapshot> operator_events_snapshot_locked() const;
  void update_operator_events_locked(
    const VisionStatusSnapshot & vision, const RobotHealthSnapshot & health,
    const PreviewStatusSnapshot & preview, const TeleopStatusSnapshot & teleop,
    const PlatformStatusSnapshot & platform) const;
  void append_operator_event_locked(std::string message) const;

  mutable std::mutex          _mutex{};
  std::string                 _gateway_name{};
  std::string                 _gateway_version{};
  std::chrono::milliseconds   _vision_stale_after{2000};
  std::chrono::milliseconds   _odom_stale_after{1000};
  std::chrono::milliseconds   _detections_stale_after{500};
  std::chrono::milliseconds   _platform_stale_after{2000};
  uint32_t                    _detection_source_width_px{1280};
  uint32_t                    _detection_source_height_px{720};
  TimeSource                  _time_source{};
  PreviewStatusSnapshot       _preview{};
  TeleopStatusSnapshot        _teleop{};
  bool                        _has_vision_perf{false};
  StoredVisionPerf            _vision_perf{};
  bool                        _has_odometry{false};
  StoredOdometry              _odometry{};
  bool                        _has_detections{false};
  StoredDetectionOverlay      _detections{};
  bool                        _has_compute{false};
  StoredComputeStatus         _compute{};
  bool                        _has_network{false};
  StoredNetworkStatus         _network{};
  bool                        _has_lipo_battery{false};
  StoredBatteryStatus         _lipo_battery{};
  bool                        _has_onboard_battery{false};
  StoredBatteryStatus         _onboard_battery{};
  mutable std::vector<StoredOperatorEvent> _operator_events{};
  mutable uint64_t            _next_operator_event_sequence{1};
  mutable std::optional<bool> _last_odom_fault_active{};
  mutable std::optional<bool> _last_vision_fault_active{};
  mutable std::optional<bool> _last_preview_error_active{};
  mutable std::optional<bool> _last_teleop_error_active{};
  mutable std::optional<bool> _last_wifi_fault_active{};
  mutable std::optional<bool> _last_compute_fault_active{};
  mutable std::optional<bool> _last_lipo_low_active{};
  mutable std::optional<bool> _last_onboard_low_active{};
};
} // namespace robot_diag_control_cpp
