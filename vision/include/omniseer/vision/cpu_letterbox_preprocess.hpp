#pragma once

#include <cstdint>
#include <opencv2/core.hpp>

#include "omniseer/vision/config.hpp"
#include "omniseer/vision/types.hpp"

namespace omniseer::vision
{
  /**
   * @brief Copy a BGR OpenCV frame into a DMA-backed RGB model-input buffer.
   *
   * The destination is filled with pad value 114, then a centered, uniformly resized
   * source image is copied using the supplied shared letterbox remap geometry.
   *
   * @throws std::invalid_argument or std::runtime_error on invalid inputs or DMA-BUF I/O failure.
   */
  void cpu_letterbox_bgr_to_rgb(const cv::Mat& source_bgr, ImageBuffer& destination,
                                const PipelineRemapConfig& remap, uint8_t pad_value = 114);
} // namespace omniseer::vision
