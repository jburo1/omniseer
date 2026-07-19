#include <chrono>
#include <vector>

#include <gtest/gtest.h>

#include "robot_diag_control_cpp/teleop_manager.hpp"

namespace robot_diag_control_cpp
{
namespace
{
using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

TEST(TeleopManagerTest, DefaultsDisabledAndPublishesStopWhenDisabled)
{
  GatewayStateStore store;
  std::vector<TeleopCommand> published;
  TeleopManager manager(
    store,
    [&published](const TeleopCommand & command)
    {
      published.push_back(command);
    });

  const auto initial = store.get_system_status().teleop;
  EXPECT_EQ(initial.state, TeleopState::Disabled);
  EXPECT_FALSE(initial.enabled);

  const auto enabled = manager.set_enabled(true);
  EXPECT_TRUE(enabled.accepted);
  EXPECT_EQ(enabled.teleop.state, TeleopState::Enabled);

  const auto disabled = manager.set_enabled(false);
  EXPECT_TRUE(disabled.accepted);
  ASSERT_FALSE(published.empty());
  EXPECT_DOUBLE_EQ(published.back().linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(published.back().angular_z_rad_s, 0.0);
}

TEST(TeleopManagerTest, RejectsOutOfBoundsCommands)
{
  GatewayStateStore store;
  std::vector<TeleopCommand> published;
  TeleopManager manager(
    store,
    [&published](const TeleopCommand & command)
    {
      published.push_back(command);
    },
    TeleopManagerConfig{0.25, 0.5});

  manager.set_enabled(true);
  const auto result = manager.send_command(TeleopCommand{0.30, 0.0, 0.0});

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.message, "teleop command exceeds configured bounds");
  EXPECT_TRUE(published.empty());
  EXPECT_EQ(store.get_system_status().teleop.last_error, result.message);
}

TEST(TeleopManagerTest, AcceptsBackToBackCommandsWhileEnabled)
{
  TimePoint now{Clock::duration{std::chrono::seconds(100)}};
  GatewayStateStore store(
    "robot_diag_control_cpp", "0.1.0", std::chrono::milliseconds(1500),
    std::chrono::milliseconds(1000),
    [&now]()
    {
      return now;
    });
  std::vector<TeleopCommand> published;
  TeleopManager manager(
    store,
    [&published](const TeleopCommand & command)
    {
      published.push_back(command);
    },
    TeleopManagerConfig{0.35, 0.8},
    [&now]()
    {
      return now;
    });

  manager.set_enabled(true);
  EXPECT_TRUE(manager.send_command(TeleopCommand{0.1, 0.0, 0.0}).accepted);
  EXPECT_TRUE(manager.send_command(TeleopCommand{0.2, 0.0, 0.0}).accepted);
  EXPECT_EQ(published.size(), 2U);

  now += std::chrono::milliseconds(50);
  EXPECT_TRUE(manager.send_command(TeleopCommand{0.3, 0.0, 0.0}).accepted);
  EXPECT_EQ(published.size(), 3U);
}

TEST(TeleopManagerTest, StoresLastAcceptedCommandVector)
{
  TimePoint now{Clock::duration{std::chrono::seconds(100)}};
  GatewayStateStore store(
    "robot_diag_control_cpp", "0.1.0", std::chrono::milliseconds(1500),
    std::chrono::milliseconds(1000),
    [&now]()
    {
      return now;
    });
  std::vector<TeleopCommand> published;
  TeleopManager manager(
    store,
    [&published](const TeleopCommand & command)
    {
      published.push_back(command);
    },
    TeleopManagerConfig{0.35, 0.8},
    [&now]()
    {
      return now;
    });

  manager.set_enabled(true);
  const auto result = manager.send_command(TeleopCommand{0.12, -0.03, 0.25});

  ASSERT_TRUE(result.accepted);
  const auto status = store.get_system_status().teleop;
  EXPECT_DOUBLE_EQ(status.last_command_vx_mps, 0.12);
  EXPECT_DOUBLE_EQ(status.last_command_vy_mps, -0.03);
  EXPECT_DOUBLE_EQ(status.last_command_wz_rad_s, 0.25);
}

TEST(TeleopManagerTest, PollKeepsTeleopEnabledAfterCommandAges)
{
  TimePoint now{Clock::duration{std::chrono::seconds(100)}};
  GatewayStateStore store(
    "robot_diag_control_cpp", "0.1.0", std::chrono::milliseconds(1500),
    std::chrono::milliseconds(1000),
    [&now]()
    {
      return now;
    });
  std::vector<TeleopCommand> published;
  TeleopManager manager(
    store,
    [&published](const TeleopCommand & command)
    {
      published.push_back(command);
    },
    TeleopManagerConfig{0.35, 0.8},
    [&now]()
    {
      return now;
    });

  manager.set_enabled(true);
  ASSERT_TRUE(manager.send_command(TeleopCommand{0.1, 0.0, 0.0}).accepted);
  now += std::chrono::seconds(5);
  manager.poll();

  const auto status = store.get_system_status().teleop;
  EXPECT_EQ(status.state, TeleopState::Enabled);
  EXPECT_TRUE(status.enabled);
  EXPECT_FALSE(status.timed_out);
  EXPECT_EQ(published.size(), 1U);
  EXPECT_EQ(status.last_error, "");
}
} // namespace
} // namespace robot_diag_control_cpp
