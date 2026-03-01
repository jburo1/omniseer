#pragma once

#include "omniseer/vision/types.hpp"

namespace omniseer::vision
{
  /**
   * @brief Immutable geometry/remap contract for the fixed pipeline setup.
   */
  struct PipelineRemapConfig
  {
    /// @brief Source capture frame dimensions in pixels.
    Size source_size{1280, 720};
    /// @brief Model input frame dimensions in pixels.
    Size model_input_size{640, 640};

    /// @brief Uniform resize scale from source to model input.
    float scale{0.5F};
    /// @brief Horizontal letterbox padding in destination pixels.
    int pad_x{0};
    /// @brief Vertical letterbox padding in destination pixels.
    int pad_y{140};
    /// @brief Resized content width before padding.
    int resized_w{640};
    /// @brief Resized content height before padding.
    int resized_h{360};
  };

} // namespace omniseer::vision
