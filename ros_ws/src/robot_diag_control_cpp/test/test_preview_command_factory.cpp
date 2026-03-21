#include <algorithm>

#include <gtest/gtest.h>

#include "robot_diag_control_cpp/preview_command_factory.hpp"

namespace robot_diag_control_cpp
{
namespace
{
TEST(PreviewCommandFactoryTest, BuildsCameraPipelineForBalancedProfile)
{
  const auto factory = make_gstreamer_preview_command_factory(
    GstreamerPreviewConfig{
        "gst-launch-1.0",
        "camera",
        "/dev/video11",
        "0.0.0.0",
        7001,
        125,
    });

  const auto resolution = factory(PreviewProfile::Balanced);
  ASSERT_TRUE(resolution.ok);
  EXPECT_EQ(resolution.command.executable, "gst-launch-1.0");
  EXPECT_FALSE(resolution.command.arguments.empty());
  EXPECT_EQ(resolution.command.arguments[0], "-q");
  EXPECT_EQ(resolution.command.arguments[1], "-e");
  EXPECT_NE(
    std::find(
      resolution.command.arguments.begin(),
      resolution.command.arguments.end(),
      "device=/dev/video11"),
    resolution.command.arguments.end());
  EXPECT_NE(
    std::find(
      resolution.command.arguments.begin(),
      resolution.command.arguments.end(),
      "video/x-raw,format=NV12,width=1280,height=720,framerate=30/1"),
    resolution.command.arguments.end());
  EXPECT_NE(
    std::find(
      resolution.command.arguments.begin(),
      resolution.command.arguments.end(),
      "bitrate=2500"),
    resolution.command.arguments.end());
  EXPECT_NE(
    std::find(
      resolution.command.arguments.begin(),
      resolution.command.arguments.end(),
      "localport=7001"),
    resolution.command.arguments.end());
}

TEST(PreviewCommandFactoryTest, RejectsUnsupportedSourceKind)
{
  const auto factory = make_gstreamer_preview_command_factory(
    GstreamerPreviewConfig{
        "gst-launch-1.0",
        "bogus",
        "/dev/video11",
        "0.0.0.0",
        7001,
        125,
    });

  const auto resolution = factory(PreviewProfile::LowBw);
  EXPECT_FALSE(resolution.ok);
  EXPECT_EQ(resolution.message, "unsupported preview source kind: bogus");
}
} // namespace
} // namespace robot_diag_control_cpp
