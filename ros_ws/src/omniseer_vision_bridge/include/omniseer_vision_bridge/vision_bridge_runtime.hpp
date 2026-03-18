#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "omniseer/vision/rolling_telemetry.hpp"

namespace omniseer::vision
{
class IDetectionsSink;
}

namespace omniseer_vision_bridge
{
struct VisionBridgeRuntimeConfig
{
  std::string camera_device{"/dev/video12"};
  int64_t     camera_width{1280};
  int64_t     camera_height{720};
  int64_t     camera_buffer_count{4};

  int64_t pipeline_dst_width{640};
  int64_t pipeline_dst_height{640};

  std::string detector_model_path{};
  std::string clip_model_path{};
  std::string clip_vocab_path{};

  std::string class_list_path{};
  std::string pad_token{"nothing"};

  int64_t producer_preflight_capture_wait_ms{200};
  int64_t runner_warmup_runs{0};

  double  score_threshold{0.25};
  double  nms_iou_threshold{0.45};
  int64_t max_detections{100};

  std::string camera_frame_id{"camera_optical_frame"};
  omniseer::vision::IDetectionsSink * detections_sink{nullptr};
};

class VisionBridgeRuntime
{
public:
  explicit VisionBridgeRuntime(VisionBridgeRuntimeConfig cfg);
  ~VisionBridgeRuntime();

  VisionBridgeRuntime(const VisionBridgeRuntime &)            = delete;
  VisionBridgeRuntime & operator=(const VisionBridgeRuntime &) = delete;
  VisionBridgeRuntime(VisionBridgeRuntime &&)                 = delete;
  VisionBridgeRuntime & operator=(VisionBridgeRuntime &&)      = delete;

  void start();
  void stop() noexcept;

  bool is_running() const noexcept;
  bool has_fatal_error() const noexcept;
  std::string fatal_error_message() const;
  omniseer::vision::RollingTelemetrySnapshot telemetry_snapshot() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> _impl;
};
} // namespace omniseer_vision_bridge
