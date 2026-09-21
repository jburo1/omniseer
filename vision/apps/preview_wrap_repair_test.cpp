#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <gtest/gtest.h>
#include <string>
#include <vector>

#include "omniseer/vision/preview_wrap_repair.hpp"

namespace omniseer::vision
{
  TEST(PreviewWrapRepair, RestoresEverySyntheticBgrPixelWithoutChangingGeometry)
  {
    constexpr size_t row_bytes =
        kRockchipPreviewWrapRepairWidth * kRockchipPreviewWrapRepairChannels;
    std::vector<uint8_t> original(row_bytes * kRockchipPreviewWrapRepairHeight);
    for (int y = 0; y < kRockchipPreviewWrapRepairHeight; ++y)
    {
      for (int x = 0; x < kRockchipPreviewWrapRepairWidth; ++x)
      {
        const size_t offset  = static_cast<size_t>(y) * row_bytes +
                               static_cast<size_t>(x) * kRockchipPreviewWrapRepairChannels;
        original[offset + 0] = static_cast<uint8_t>(x & 0xff);
        original[offset + 1] = static_cast<uint8_t>((x >> 8) + y);
        original[offset + 2] = static_cast<uint8_t>(y & 0xff);
      }
    }

    std::vector<uint8_t> recorded(original.size());
    for (int y = 0; y < kRockchipPreviewWrapRepairHeight; ++y)
    {
      const size_t offset = static_cast<size_t>(y) * row_bytes;
      std::copy_n(original.begin() +
                      static_cast<std::ptrdiff_t>(offset + (kRockchipPreviewWrapRepairWidth -
                                                            kRockchipPreviewWrapRepairPixels) *
                                                               kRockchipPreviewWrapRepairChannels),
                  kRockchipPreviewWrapRepairPixels * kRockchipPreviewWrapRepairChannels,
                  recorded.begin() + static_cast<std::ptrdiff_t>(offset));
      std::copy_n(original.begin() + static_cast<std::ptrdiff_t>(offset),
                  (kRockchipPreviewWrapRepairWidth - kRockchipPreviewWrapRepairPixels) *
                      kRockchipPreviewWrapRepairChannels,
                  recorded.begin() +
                      static_cast<std::ptrdiff_t>(offset + kRockchipPreviewWrapRepairPixels *
                                                               kRockchipPreviewWrapRepairChannels));
    }

    std::string error{};
    ASSERT_TRUE(repair_rockchip_preview_wrap_bgr(recorded.data(), row_bytes,
                                                 kRockchipPreviewWrapRepairWidth,
                                                 kRockchipPreviewWrapRepairHeight, error))
        << error;
    EXPECT_EQ(recorded, original);
  }

  TEST(PreviewWrapRepair, RejectsUnsupportedGeometryAndInvalidStride)
  {
    std::vector<uint8_t> pixels(1280U * 720U * 3U);
    std::string          error{};
    EXPECT_FALSE(repair_rockchip_preview_wrap_bgr(pixels.data(), 1280U * 3U, 640, 720, error));
    EXPECT_NE(error.find("1280x720"), std::string::npos);
    EXPECT_FALSE(repair_rockchip_preview_wrap_bgr(pixels.data(), 1, 1280, 720, error));
    EXPECT_NE(error.find("stride"), std::string::npos);
  }
} // namespace omniseer::vision
