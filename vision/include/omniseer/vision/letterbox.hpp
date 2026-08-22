#pragma once

#include <cstdint>

#include "omniseer/vision/config.hpp"

namespace omniseer::vision
{
  /**
   * @brief Runtime letterbox parameters shared by preprocessing implementations.
   */
  struct LetterboxMeta
  {
    float scale{1.0F};
    int   pad_x{0};
    int   pad_y{0};
    int   resized_w{0};
    int   resized_h{0};
  };

  /**
   * @brief Compute centered, uniform letterbox geometry for one source/destination pair.
   *
   * @throws std::invalid_argument when either size is invalid.
   */
  LetterboxMeta compute_letterbox_meta(Size source_size, Size model_input_size);

  /**
   * @brief Build the source-to-model remap contract used by YOLO-World postprocess.
   */
  PipelineRemapConfig make_letterbox_remap(Size source_size, Size model_input_size);
} // namespace omniseer::vision
