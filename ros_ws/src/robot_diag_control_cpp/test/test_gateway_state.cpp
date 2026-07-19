#include <chrono>
#include <string>

#include <gtest/gtest.h>

#include "nav_msgs/msg/odometry.hpp"
#include "omniseer_msgs/msg/vision_perf_summary.hpp"
#include "robot_diag_control_cpp/gateway_state.hpp"
#include "yolo_msgs/msg/detection_array.hpp"

namespace robot_diag_control_cpp
{
namespace
{
using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

TEST(GatewayStateStoreTest, PreviewStateTransitionsStayPredictable)
{
  GatewayStateStore store;

  const auto initial = store.get_system_status().preview;
  EXPECT_EQ(initial.state, PreviewState::Disabled);
  EXPECT_EQ(initial.profile, PreviewProfile::Balanced);

  const auto enabled = store.set_preview_running(PreviewProfile::HighQuality);
  EXPECT_EQ(enabled.state, PreviewState::Running);
  EXPECT_EQ(enabled.profile, PreviewProfile::HighQuality);
  EXPECT_TRUE(enabled.last_error.empty());

  const auto disabled = store.set_preview_disabled(PreviewProfile::HighQuality, "worker exited");
  EXPECT_EQ(disabled.state, PreviewState::Disabled);
  EXPECT_EQ(disabled.profile, PreviewProfile::HighQuality);
  EXPECT_EQ(disabled.last_error, "worker exited");
}

TEST(GatewayStateStoreTest, VisionPerfSnapshotBecomesStaleAfterTimeout)
{
  TimePoint now{Clock::duration{std::chrono::seconds(100)}};
  GatewayStateStore store(
    "robot_diag_control_cpp", "0.1.0", std::chrono::milliseconds(1500),
    std::chrono::milliseconds(1000),
    [&now]()
    {
      return now;
    });

  omniseer_msgs::msg::VisionPerfSummary msg{};
  msg.producer_fps = 14.0F;
  msg.consumer_fps = 12.5F;
  msg.last_infer_ms = 8.25F;
  msg.infer_error_count = 3;
  msg.capture_fatal_error_count = 1;
  store.update_vision_perf(msg);

  const auto fresh = store.get_system_status();
  EXPECT_TRUE(fresh.vision.available);
  EXPECT_FALSE(fresh.vision.stale);
  EXPECT_DOUBLE_EQ(fresh.vision.last_infer_ms, 8.25);

  now += std::chrono::seconds(2);
  const auto stale = store.get_system_status();
  EXPECT_TRUE(stale.vision.available);
  EXPECT_TRUE(stale.vision.stale);
}

TEST(GatewayStateStoreTest, RobotHealthTracksOdometryAndVisionFreshness)
{
  TimePoint now{Clock::duration{std::chrono::seconds(100)}};
  GatewayStateStore store(
    "robot_diag_control_cpp", "0.1.0", std::chrono::milliseconds(1500),
    std::chrono::milliseconds(1000),
    [&now]()
    {
      return now;
    });

  const auto initial = store.get_system_status();
  EXPECT_FALSE(initial.health.ready);
  EXPECT_EQ(initial.health.state, RobotHealthState::Degraded);
  EXPECT_EQ(initial.health.summary, "waiting for odometry");
  EXPECT_EQ(initial.teleop.state, TeleopState::Disabled);
  EXPECT_FALSE(initial.teleop.enabled);

  nav_msgs::msg::Odometry odom{};
  odom.twist.twist.linear.x = 0.30;
  odom.twist.twist.linear.y = 0.40;
  odom.twist.twist.angular.z = 0.25;
  store.update_odometry(odom);

  const auto no_vision = store.get_system_status();
  EXPECT_FALSE(no_vision.health.ready);
  EXPECT_EQ(no_vision.health.summary, "waiting for vision telemetry");
  EXPECT_TRUE(no_vision.health.odom_available);
  EXPECT_FALSE(no_vision.health.odom_stale);
  EXPECT_EQ(no_vision.health.odom_age_ms, 0U);
  EXPECT_DOUBLE_EQ(no_vision.health.linear_speed_mps, 0.5);
  EXPECT_DOUBLE_EQ(no_vision.health.angular_speed_rad_s, 0.25);
  EXPECT_DOUBLE_EQ(no_vision.health.measured_vx_mps, 0.30);
  EXPECT_DOUBLE_EQ(no_vision.health.measured_vy_mps, 0.40);
  EXPECT_DOUBLE_EQ(no_vision.health.measured_wz_rad_s, 0.25);

  omniseer_msgs::msg::VisionPerfSummary msg{};
  msg.producer_fps = 14.0F;
  msg.consumer_fps = 12.5F;
  msg.last_infer_ms = 8.25F;
  store.update_vision_perf(msg);

  const auto healthy = store.get_system_status();
  EXPECT_TRUE(healthy.health.ready);
  EXPECT_EQ(healthy.health.state, RobotHealthState::Ok);
  EXPECT_EQ(healthy.health.summary, "robot healthy");

  now += std::chrono::milliseconds(1100);
  const auto stale_odom = store.get_system_status();
  EXPECT_FALSE(stale_odom.health.ready);
  EXPECT_EQ(stale_odom.health.state, RobotHealthState::Degraded);
  EXPECT_EQ(stale_odom.health.summary, "odometry stale");
  EXPECT_TRUE(stale_odom.health.odom_stale);
  EXPECT_EQ(stale_odom.health.odom_age_ms, 1100U);
}

TEST(GatewayStateStoreTest, DetectionOverlaySnapshotTracksLatestDetectionsAndFreshness)
{
  TimePoint now{Clock::duration{std::chrono::seconds(100)}};
  GatewayStateStore store(
    "robot_diag_control_cpp", "0.1.0", std::chrono::milliseconds(1500),
    std::chrono::milliseconds(1000),
    [&now]()
    {
      return now;
    },
    std::chrono::milliseconds(400), 1280, 720);

  const auto initial = store.get_detection_overlay();
  EXPECT_FALSE(initial.available);
  EXPECT_FALSE(initial.stale);
  EXPECT_EQ(initial.source_width_px, 1280U);
  EXPECT_EQ(initial.source_height_px, 720U);

  yolo_msgs::msg::DetectionArray msg{};
  yolo_msgs::msg::Detection detection{};
  detection.class_id = 7;
  detection.class_name = "person";
  detection.score = 0.875;
  detection.id = "track-1";
  detection.bbox.center.position.x = 320.0;
  detection.bbox.center.position.y = 180.0;
  detection.bbox.size.x = 100.0;
  detection.bbox.size.y = 80.0;
  msg.detections.push_back(detection);
  store.update_detections(msg);

  const auto fresh = store.get_detection_overlay();
  ASSERT_TRUE(fresh.available);
  EXPECT_FALSE(fresh.stale);
  EXPECT_EQ(fresh.age_ms, 0U);
  ASSERT_EQ(fresh.detections.size(), 1U);
  EXPECT_EQ(fresh.detections.front().class_id, 7);
  EXPECT_EQ(fresh.detections.front().class_name, "person");
  EXPECT_DOUBLE_EQ(fresh.detections.front().score, 0.875);
  EXPECT_EQ(fresh.detections.front().track_id, "track-1");
  EXPECT_DOUBLE_EQ(fresh.detections.front().bbox_center_x_px, 320.0);

  now += std::chrono::milliseconds(450);
  const auto stale = store.get_detection_overlay();
  EXPECT_TRUE(stale.available);
  EXPECT_TRUE(stale.stale);
  EXPECT_EQ(stale.age_ms, 450U);
}

TEST(GatewayStateStoreTest, OperatorEventsTrackTransitionsAndRemainCapped)
{
  TimePoint now{Clock::duration{std::chrono::seconds(100)}};
  GatewayStateStore store(
    "robot_diag_control_cpp", "0.1.0", std::chrono::milliseconds(1500),
    std::chrono::milliseconds(1000),
    [&now]()
    {
      return now;
    });

  store.get_system_status();

  nav_msgs::msg::Odometry odom{};
  store.update_odometry(odom);
  auto status = store.get_system_status();
  auto events = store.get_operator_events();
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events.back().message, "odometry recovered");

  now += std::chrono::milliseconds(1100);
  status = store.get_system_status();
  EXPECT_TRUE(status.health.odom_stale);
  events = store.get_operator_events();
  ASSERT_EQ(events.size(), 2U);
  EXPECT_EQ(events.back().message, "odometry lost");
  EXPECT_EQ(events.back().age_ms, 0U);

  omniseer_msgs::msg::VisionPerfSummary vision{};
  store.update_vision_perf(vision);
  status = store.get_system_status();
  events = store.get_operator_events();
  ASSERT_EQ(events.size(), 3U);
  EXPECT_EQ(events.back().message, "vision recovered");

  now += std::chrono::milliseconds(1600);
  status = store.get_system_status();
  EXPECT_TRUE(status.vision.stale);
  events = store.get_operator_events();
  ASSERT_EQ(events.size(), 4U);
  EXPECT_EQ(events.back().message, "vision stale");

  for (int index = 0; index < 10; ++index) {
    store.set_preview_disabled(PreviewProfile::Balanced, "preview failed " + std::to_string(index));
  }

  events = store.get_operator_events();
  ASSERT_EQ(events.size(), 8U);
  EXPECT_EQ(events.front().message, "preview error: preview failed 2");
  EXPECT_EQ(events.back().message, "preview error: preview failed 9");
  EXPECT_LT(events.front().sequence, events.back().sequence);
}
} // namespace
} // namespace robot_diag_control_cpp
