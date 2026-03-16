#pragma once

#include "omniseer/vision/config.hpp"
#include "omniseer/vision/detections.hpp"
#include "omniseer/vision/types.hpp"

namespace omniseer::vision
{
  /**
   * @brief Synchronous preview callback for one consumed model-input frame.
   *
   * Contract:
   * - Called on the consumer hot path before the read lease is released.
   * - `image` remains valid only for the duration of the callback.
   * - Implementations must not retain references or pointers into `image`.
   */
  class IFramePreviewSink
  {
  public:
    virtual ~IFramePreviewSink() = default;

    virtual void publish(const ImageBuffer& image, const DetectionsFrame& detections,
                         const PipelineRemapConfig& remap) noexcept = 0;
  };
} // namespace omniseer::vision
