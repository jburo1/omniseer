#include "omniseer/vision/preview_wrap_repair.hpp"

#include <array>
#include <cstring>

namespace omniseer::vision
{
  bool repair_rockchip_preview_wrap_bgr(uint8_t* data, size_t row_stride, int width, int height,
                                        std::string& error) noexcept
  {
    error.clear();
    if (data == nullptr)
    {
      error = "Rockchip preview-wrap repair requires non-null frame data";
      return false;
    }
    if (width != kRockchipPreviewWrapRepairWidth || height != kRockchipPreviewWrapRepairHeight)
    {
      error = "Rockchip preview-wrap repair is validated only for 1280x720 BGR frames";
      return false;
    }

    constexpr size_t pixel_bytes = kRockchipPreviewWrapRepairChannels;
    constexpr size_t wrap_bytes = kRockchipPreviewWrapRepairPixels * pixel_bytes;
    constexpr size_t row_bytes = kRockchipPreviewWrapRepairWidth * pixel_bytes;
    if (row_stride < row_bytes)
    {
      error = "Rockchip preview-wrap repair frame stride is smaller than one BGR row";
      return false;
    }

    for (int y = 0; y < height; ++y)
    {
      uint8_t* row = data + static_cast<size_t>(y) * row_stride;
      std::array<uint8_t, wrap_bytes> wrapped_right{};
      std::memcpy(wrapped_right.data(), row, wrap_bytes);
      std::memmove(row, row + wrap_bytes, row_bytes - wrap_bytes);
      std::memcpy(row + row_bytes - wrap_bytes, wrapped_right.data(), wrap_bytes);
    }
    return true;
  }
} // namespace omniseer::vision
