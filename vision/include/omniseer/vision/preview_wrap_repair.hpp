#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace omniseer::vision
{
  /**
   * @brief Validated Rockchip preview recording geometry and circular-wrap repair.
   *
   * Raw RunBundle preview evidence is recorded as
   * [original x=1272..1279][original x=0..1271].  This repair rotates each decoded
   * BGR row left by eight pixels.  It is intentionally limited to the validated
   * 1280x720 recording geometry so arbitrary video is never silently transformed.
   */
  constexpr int kRockchipPreviewWrapRepairPixels{8};
  constexpr int kRockchipPreviewWrapRepairWidth{1280};
  constexpr int kRockchipPreviewWrapRepairHeight{720};
  constexpr int kRockchipPreviewWrapRepairChannels{3};

  /**
   * @brief Reverse the known Rockchip preview wrap in-place on a decoded BGR frame.
   *
   * @return true on success.  On false, @p error identifies the unsupported input.
   */
  bool repair_rockchip_preview_wrap_bgr(uint8_t* data, size_t row_stride, int width, int height,
                                        std::string& error) noexcept;
} // namespace omniseer::vision
