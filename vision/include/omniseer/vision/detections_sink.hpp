#pragma once

#include "omniseer/vision/detections.hpp"

namespace omniseer::vision
{
  /**
   * @brief Single-boundary sink for canonical consumer detections.
   */
  class IDetectionsSink
  {
  public:
    virtual ~IDetectionsSink() = default;

    /// @brief Publish one detections frame. Implementations must not throw.
    virtual void publish(const DetectionsFrame& frame) noexcept = 0;
  };
} // namespace omniseer::vision
