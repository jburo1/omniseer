#include "omniseer/vision/letterbox.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace omniseer::vision
{
  LetterboxMeta compute_letterbox_meta(Size source_size, Size model_input_size)
  {
    if (source_size.w <= 0 || source_size.h <= 0 || model_input_size.w <= 0 ||
        model_input_size.h <= 0)
    {
      throw std::invalid_argument("letterbox sizes must be positive");
    }

    const float sx    = static_cast<float>(model_input_size.w) / static_cast<float>(source_size.w);
    const float sy    = static_cast<float>(model_input_size.h) / static_cast<float>(source_size.h);
    const float scale = std::min(sx, sy);

    LetterboxMeta out{};
    out.scale = scale;
    out.resized_w =
        std::clamp(static_cast<int>(std::lround(source_size.w * scale)), 1, model_input_size.w);
    out.resized_h =
        std::clamp(static_cast<int>(std::lround(source_size.h * scale)), 1, model_input_size.h);
    out.pad_x = (model_input_size.w - out.resized_w) / 2;
    out.pad_y = (model_input_size.h - out.resized_h) / 2;
    return out;
  }

  PipelineRemapConfig make_letterbox_remap(Size source_size, Size model_input_size)
  {
    const LetterboxMeta meta = compute_letterbox_meta(source_size, model_input_size);
    PipelineRemapConfig out{};
    out.source_size      = source_size;
    out.model_input_size = model_input_size;
    out.scale            = meta.scale;
    out.pad_x            = meta.pad_x;
    out.pad_y            = meta.pad_y;
    out.resized_w        = meta.resized_w;
    out.resized_h        = meta.resized_h;
    return out;
  }
} // namespace omniseer::vision
