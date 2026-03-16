#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "omniseer/vision/config.hpp"
#include "omniseer/vision/detections.hpp"
#include "omniseer/vision/rknn_runner.hpp"

namespace omniseer::vision
{
  /**
   * @brief Resolved static YOLO-World RKNN output layout.
   */
  struct YoloWorldOutputLayout
  {
    uint32_t               class_capacity{0};
    std::array<uint32_t, 3> class_output_indices{};
    std::array<uint32_t, 3> box_output_indices{};
    std::array<uint32_t, 3> grid_sizes{};
  };

  /**
   * @brief Validate and resolve the fixed YOLO-World RKNN output layout.
   *
   * Expected layout:
   * - 3 class tensors with shapes `1 x C x {80,40,20} x {80,40,20}`
   * - 3 box tensors with shapes `1 x 4 x {80,40,20} x {80,40,20}`
   *
   * @throws std::runtime_error if the output contract does not match.
   */
  YoloWorldOutputLayout resolve_yolo_world_output_layout(const std::vector<RknnOutputDesc>& descs);

  /**
   * @brief Decode YOLO-World RKNN outputs into canonical source-space detections.
   *
   * This is a non-throwing hot-path helper. On any unsupported or inconsistent
   * runtime condition, it fails closed by leaving `frame.count == 0`.
   */
  void decode_yolo_world_detections(const std::vector<RknnOutputView>& outputs,
                                    const std::vector<RknnOutputDesc>& output_descs,
                                    const YoloWorldOutputLayout& layout,
                                    const ConsumerPipelineConfig& cfg,
                                    const PipelineRemapConfig& remap,
                                    uint32_t active_class_count,
                                    DetectionsFrame& frame) noexcept;
} // namespace omniseer::vision
