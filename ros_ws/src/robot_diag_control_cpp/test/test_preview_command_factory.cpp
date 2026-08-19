#include <algorithm>
#include <gtest/gtest.h>

#include "robot_diag_control_cpp/preview_command_factory.hpp"

namespace robot_diag_control_cpp
{
  namespace
  {
    bool has_argument(const PreviewCommandResolution& resolution, const std::string& argument)
    {
      return std::find(resolution.command.arguments.begin(), resolution.command.arguments.end(),
                       argument) != resolution.command.arguments.end();
    }

    TEST(PreviewCommandFactoryTest, BuildsCameraPipelineForBalancedProfile)
    {
      const auto factory = make_gstreamer_preview_command_factory(GstreamerPreviewConfig{
          "gst-launch-1.0",
          "camera",
          "/dev/video11",
          "0.0.0.0",
          7100,
          125,
      });

      const auto resolution = factory(PreviewProfile::Balanced);
      ASSERT_TRUE(resolution.ok);
      EXPECT_EQ(resolution.command.executable, "gst-launch-1.0");
      EXPECT_FALSE(resolution.command.arguments.empty());
      EXPECT_EQ(resolution.command.arguments[0], "-q");
      EXPECT_EQ(resolution.command.arguments[1], "-e");
      EXPECT_TRUE(has_argument(resolution, "device=/dev/video11"));
      EXPECT_TRUE(
          has_argument(resolution, "video/x-raw,format=NV12,width=1280,height=720,framerate=30/1"));
      EXPECT_TRUE(has_argument(resolution, "x264enc"));
      EXPECT_TRUE(has_argument(resolution, "tune=zerolatency"));
      EXPECT_TRUE(has_argument(resolution, "speed-preset=veryfast"));
      EXPECT_TRUE(has_argument(resolution, "bitrate=2500"));
      EXPECT_TRUE(has_argument(resolution, "key-int-max=30"));
      EXPECT_TRUE(has_argument(resolution, "bframes=0"));
      EXPECT_TRUE(has_argument(resolution, "byte-stream=true"));
      EXPECT_TRUE(has_argument(resolution, "h264parse"));
      EXPECT_TRUE(has_argument(resolution, "config-interval=-1"));
      EXPECT_TRUE(has_argument(resolution, "mpegtsmux"));
      EXPECT_TRUE(has_argument(resolution, "srtsink"));
      EXPECT_TRUE(has_argument(resolution, "localport=7100"));
    }

    TEST(PreviewCommandFactoryTest, RejectsUnsupportedSourceKind)
    {
      const auto factory = make_gstreamer_preview_command_factory(GstreamerPreviewConfig{
          "gst-launch-1.0",
          "bogus",
          "/dev/video11",
          "0.0.0.0",
          7100,
          125,
      });

      const auto resolution = factory(PreviewProfile::LowBw);
      EXPECT_FALSE(resolution.ok);
      EXPECT_EQ(resolution.message, "unsupported preview source kind: bogus");
    }

    TEST(PreviewCommandFactoryTest, AddsFileSinkToTheEncodedStreamWhenRecording)
    {
      auto config        = GstreamerPreviewConfig{};
      config.record_path = "/runs/demo/video/source.ts";
      const auto resolution =
          make_gstreamer_preview_command_factory(config)(PreviewProfile::Balanced);

      ASSERT_TRUE(resolution.ok);
      EXPECT_EQ(resolution.command.executable, "robot_diag_preview_recorder");
      EXPECT_TRUE(resolution.command.writes_first_buffer_timing);
      EXPECT_TRUE(has_argument(resolution, "--pipeline"));
      ASSERT_EQ(resolution.command.arguments.size(), 2U);
      EXPECT_NE(resolution.command.arguments[1].find("h264parse"), std::string::npos);
      EXPECT_NE(resolution.command.arguments[1].find("identity name=timing_probe"),
                std::string::npos);
      EXPECT_NE(resolution.command.arguments[1].find("tee name=encoded"), std::string::npos);
      EXPECT_NE(resolution.command.arguments[1].find(
                    "filesink location=/runs/demo/video/source.ts"),
                std::string::npos);
      EXPECT_NE(resolution.command.arguments[1].find("srtsink"), std::string::npos);
    }

    TEST(PreviewCommandFactoryTest, BuildsRockchipPipelineForBalancedProfile)
    {
      auto config    = GstreamerPreviewConfig{};
      config.encoder = "rockchip";
      const auto resolution =
          make_gstreamer_preview_command_factory(config)(PreviewProfile::Balanced);

      ASSERT_TRUE(resolution.ok);
      EXPECT_TRUE(
          has_argument(resolution, "video/x-raw,format=NV12,width=1280,height=720,framerate=30/1"));
      EXPECT_TRUE(has_argument(resolution, "mpph264enc"));
      EXPECT_TRUE(has_argument(resolution, "bps=2500000"));
      EXPECT_TRUE(has_argument(resolution, "gop=30"));
      EXPECT_TRUE(has_argument(resolution, "header-mode=each-idr"));
      EXPECT_FALSE(has_argument(resolution, "x264enc"));
      EXPECT_TRUE(has_argument(resolution, "h264parse"));
      EXPECT_TRUE(has_argument(resolution, "mpegtsmux"));
      EXPECT_TRUE(has_argument(resolution, "srtsink"));
    }

    TEST(PreviewCommandFactoryTest, RejectsUnsupportedEncoderBackend)
    {
      auto config           = GstreamerPreviewConfig{};
      config.encoder        = "bogus";
      const auto resolution = make_gstreamer_preview_command_factory(config)(PreviewProfile::LowBw);

      EXPECT_FALSE(resolution.ok);
      EXPECT_EQ(resolution.message, "unsupported preview encoder backend: bogus");
    }
  } // namespace
} // namespace robot_diag_control_cpp
