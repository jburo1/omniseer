#pragma once

#include <array>
#include <cstdint>

#include "omniseer/vision/types.hpp"

namespace omniseer::vision
{
  /**
   * @brief One canonical detection in source-image pixel coordinates.
   */
  struct Detection
  {
    /// @brief Zero-based class index within the active class list.
    uint16_t class_id{0};
    /// @brief Detection confidence in [0, 1].
    float score{0.0F};
    /// @brief Left pixel coordinate in source image space.
    float x1{0.0F};
    /// @brief Top pixel coordinate in source image space.
    float y1{0.0F};
    /// @brief Right pixel coordinate in source image space.
    float x2{0.0F};
    /// @brief Bottom pixel coordinate in source image space.
    float y2{0.0F};
  };

  /**
   * @brief Fixed-capacity detections packet published by the consumer stage.
   */
  struct DetectionsFrame
  {
    /// @brief Maximum number of detections carried in one frame.
    static constexpr uint32_t capacity{100};

    /// @brief Producer-assigned logical frame identifier.
    uint64_t frame_id{0};
    /// @brief Source capture sequence, when available.
    uint64_t sequence{0};
    /// @brief Source capture timestamp mapped to realtime nanoseconds.
    uint64_t capture_ts_real_ns{0};
    /// @brief Number of active classes configured for this consumer run.
    uint32_t active_class_count{0};
    /// @brief Source image dimensions used for bbox coordinates.
    Size source_size{};
    /// @brief Number of valid entries in @ref detections.
    uint32_t count{0};
    /// @brief Canonical detections in source-image coordinates.
    std::array<Detection, capacity> detections{};
  };
} // namespace omniseer::vision
