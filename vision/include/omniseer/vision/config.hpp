#pragma once

#include "omniseer/vision/types.hpp"

namespace omniseer::vision
{
  // Immutable geometry/remap contract for the current fixed pipeline setup.
  // If capture or model input sizes change, update these values in one place.
  struct PipelineRemapConfig
  {
    Size source_size{1280, 720};
    Size model_input_size{640, 640};

    float scale{0.5F};
    int   pad_x{0};
    int   pad_y{140};
    int   resized_w{640};
    int   resized_h{360};
  };

} // namespace omniseer::vision
