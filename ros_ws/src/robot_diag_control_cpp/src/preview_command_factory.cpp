#include "robot_diag_control_cpp/preview_command_factory.hpp"

#include <utility>
#include <vector>

namespace robot_diag_control_cpp
{
namespace
{
struct ProfileSettings
{
  int fps{30};
  int bitrate_kbps{2000};
};

ProfileSettings profile_settings_for(PreviewProfile profile)
{
  switch (profile) {
    case PreviewProfile::LowBw:
      return ProfileSettings{15, 1000};
    case PreviewProfile::Balanced:
      return ProfileSettings{30, 2500};
    case PreviewProfile::HighQuality:
      return ProfileSettings{60, 4500};
  }

  return ProfileSettings{};
}
} // namespace

PreviewCommandFactory make_fixed_preview_command_factory(PreviewProcessCommand command)
{
  return [command = std::move(command)](PreviewProfile)
         {
           if (command.executable.empty()) {
             return PreviewCommandResolution{false, "preview command not configured", {}};
           }

           return PreviewCommandResolution{true, "", command};
         };
}

PreviewCommandFactory make_gstreamer_preview_command_factory(GstreamerPreviewConfig config)
{
  return [config = std::move(config)](PreviewProfile profile)
         {
           if (config.gst_launch_path.empty()) {
             return PreviewCommandResolution{false, "preview gst-launch path is empty", {}};
           }

           if (config.srt_port < 0 || config.srt_port > 65535) {
             return PreviewCommandResolution{
               false,
               "preview SRT port must be between 0 and 65535",
               {},
             };
           }

           if (config.srt_latency_ms < 0) {
             return PreviewCommandResolution{false, "preview SRT latency must be non-negative", {}};
           }

           const auto profile_settings = profile_settings_for(profile);
           const std::string caps =
             "video/x-raw,format=NV12,width=1280,height=720,framerate=" +
             std::to_string(profile_settings.fps) + "/1";

           std::vector<std::string> arguments{
             "-q",
             "-e",
           };

           if (config.source_kind == "camera") {
             if (config.device.empty()) {
               return PreviewCommandResolution{false, "preview device path is empty", {}};
             }

             arguments.insert(
               arguments.end(),
               {"v4l2src", "device=" + config.device, "do-timestamp=true"});
           } else if (config.source_kind == "videotest") {
             arguments.insert(
               arguments.end(),
               {"videotestsrc", "is-live=true", "pattern=smpte"});
           } else {
             return PreviewCommandResolution{
               false,
               "unsupported preview source kind: " + config.source_kind,
               {},
             };
           }

           arguments.insert(
             arguments.end(),
      {
        "!",
        caps,
        "!",
        "queue",
        "leaky=downstream",
        "max-size-buffers=2",
        "max-size-bytes=0",
        "max-size-time=0",
        "!",
        "x264enc",
        "tune=zerolatency",
        "speed-preset=veryfast",
        "bitrate=" + std::to_string(profile_settings.bitrate_kbps),
        "key-int-max=" + std::to_string(profile_settings.fps),
        "bframes=0",
        "byte-stream=true",
        "!",
        "h264parse",
        "config-interval=-1",
        "!",
        "mpegtsmux",
        "alignment=7",
        "!",
        "srtsink",
        "mode=listener",
        "localaddress=" + config.srt_bind_address,
        "localport=" + std::to_string(config.srt_port),
        "wait-for-connection=false",
        "latency=" + std::to_string(config.srt_latency_ms),
      });

           return PreviewCommandResolution{
             true,
             "",
             PreviewProcessCommand{
               config.gst_launch_path,
               std::move(arguments),
             },
           };
         };
}
} // namespace robot_diag_control_cpp
