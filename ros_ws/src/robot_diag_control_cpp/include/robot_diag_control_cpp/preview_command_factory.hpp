#pragma once

#include <string>

#include "robot_diag_control_cpp/gateway_state.hpp"
#include "robot_diag_control_cpp/preview_process_manager.hpp"

namespace robot_diag_control_cpp
{
struct GstreamerPreviewConfig
{
  std::string gst_launch_path{"gst-launch-1.0"};
  std::string source_kind{"camera"};
  std::string device{"/dev/video11"};
  std::string srt_bind_address{"0.0.0.0"};
  int srt_port{7001};
  int srt_latency_ms{125};
};

PreviewCommandFactory make_fixed_preview_command_factory(PreviewProcessCommand command);
PreviewCommandFactory make_gstreamer_preview_command_factory(
  GstreamerPreviewConfig config = {});
} // namespace robot_diag_control_cpp
