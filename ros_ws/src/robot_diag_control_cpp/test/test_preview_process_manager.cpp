#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "robot_diag_control_cpp/preview_command_factory.hpp"
#include "robot_diag_control_cpp/gateway_state.hpp"
#include "robot_diag_control_cpp/preview_process_manager.hpp"

namespace robot_diag_control_cpp
{
namespace
{
TEST(PreviewProcessManagerTest, StartsAndStopsConfiguredProcess)
{
  GatewayStateStore store;
  PreviewProcessManager manager(
    store,
    make_fixed_preview_command_factory(PreviewProcessCommand{"/bin/sleep", {"30"}}),
    std::chrono::milliseconds(250));

  const auto enabled = manager.set_enabled(true, PreviewProfile::LowBw);
  EXPECT_TRUE(enabled.accepted);
  EXPECT_EQ(enabled.message, "preview started");
  EXPECT_EQ(enabled.preview.state, PreviewState::Running);
  EXPECT_EQ(enabled.preview.profile, PreviewProfile::LowBw);

  const auto repeated_enable = manager.set_enabled(true, PreviewProfile::LowBw);
  EXPECT_TRUE(repeated_enable.accepted);
  EXPECT_EQ(repeated_enable.message, "preview already running");

  const auto disabled = manager.set_enabled(false);
  EXPECT_TRUE(disabled.accepted);
  EXPECT_EQ(disabled.message, "preview stopped");
  EXPECT_EQ(disabled.preview.state, PreviewState::Disabled);
  EXPECT_TRUE(disabled.preview.last_error.empty());
}

TEST(PreviewProcessManagerTest, ReportsUnconfiguredCommandAsRejected)
{
  GatewayStateStore store;
  PreviewProcessManager manager(store);

  const auto result = manager.set_enabled(true, PreviewProfile::Balanced);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.message, "preview command not configured");
  EXPECT_EQ(result.preview.state, PreviewState::Disabled);
  EXPECT_EQ(result.preview.profile, PreviewProfile::Balanced);
  EXPECT_EQ(result.preview.last_error, "preview command not configured");
}

TEST(PreviewProcessManagerTest, PollMarksUnexpectedProcessExit)
{
  GatewayStateStore store;
  PreviewProcessManager manager(
    store,
    make_fixed_preview_command_factory(PreviewProcessCommand{"/bin/sh", {"-c", "exit 7"}}),
    std::chrono::milliseconds(250),
    std::chrono::milliseconds(50));

  const auto enabled = manager.set_enabled(true, PreviewProfile::HighQuality);
  ASSERT_FALSE(enabled.accepted);
  ASSERT_EQ(enabled.preview.state, PreviewState::Disabled);
  ASSERT_EQ(enabled.preview.last_error, "preview process exited with code 7");

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  manager.poll();

  const auto preview = store.get_preview_status();
  EXPECT_EQ(preview.state, PreviewState::Disabled);
  EXPECT_EQ(preview.profile, PreviewProfile::HighQuality);
  EXPECT_EQ(preview.last_error, "preview process exited with code 7");
}

TEST(PreviewProcessManagerTest, SupportsBuiltInGstreamerTestPatternFactory)
{
  GatewayStateStore store;
  PreviewProcessManager manager(
    store,
    make_gstreamer_preview_command_factory(
      GstreamerPreviewConfig{
        "gst-launch-1.0",
        "videotest",
        "/dev/video11",
        "127.0.0.1",
        7011,
        125,
      }),
    std::chrono::milliseconds(250),
    std::chrono::milliseconds(150));

  const auto enabled = manager.set_enabled(true, PreviewProfile::LowBw);
  ASSERT_TRUE(enabled.accepted);
  EXPECT_EQ(enabled.message, "preview started");
  EXPECT_EQ(enabled.preview.state, PreviewState::Running);

  const auto disabled = manager.set_enabled(false);
  EXPECT_TRUE(disabled.accepted);
  EXPECT_EQ(disabled.message, "preview stopped");
}
} // namespace
} // namespace robot_diag_control_cpp
