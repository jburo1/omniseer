#include <chrono>

#include <gtest/gtest.h>

#include "nav_msgs/msg/odometry.hpp"
#include "omniseer_msgs/msg/vision_perf_summary.hpp"
#include "robot_diag_control_cpp/gateway_state.hpp"

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
  EXPECT_DOUBLE_EQ(no_vision.health.linear_speed_mps, 0.5);
  EXPECT_DOUBLE_EQ(no_vision.health.angular_speed_rad_s, 0.25);

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
}
} // namespace
} // namespace robot_diag_control_cpp
