#pragma once

#include <cstdint>
#include <im2d_type.h>

#include "omniseer/vision/types.hpp"

namespace omniseer::vision
{

  struct LetterboxMeta
  {
    float scale{1.0f};
    int   pad_x{0};
    int   pad_y{0};
    int   resized_w{0};
    int   resized_h{0};
  };

  struct RgaPreprocessConfig
  {
    // Fixed source stream size (NV12).
    int src_w{1280};
    int src_h{720};

    int     dst_w{640};
    int     dst_h{640};
    uint8_t pad_value{114};
  };

  // NV12 (DMA-BUF) -> RGB888 (DMA-BUF) via librga (im2d).
  class RgaPreprocess
  {
  public:
    // Throws std::invalid_argument when src/dst geometry is invalid.
    explicit RgaPreprocess(RgaPreprocessConfig cfg = {});

    // Prefill the destination buffer once with the padding value so per-frame
    // work only overwrites the resized image rectangle.
    // Throws on invalid destination descriptors or DMA-BUF CPU mapping failures.
    void prefill(ImageBuffer& dst_rgb) const;

    // One-time descriptor/config validation + RGA smoke pass.
    // Expected usage: call once during startup before entering the frame loop.
    PreprocessResult preflight(const FrameDescriptor& src_nv12, ImageBuffer& dst_rgb,
                               LetterboxMeta* meta = nullptr) const noexcept;

    // Run the transformation
    // src_nv12: NV12 dmabuf frame from V4L2
    // dst_rgb : model input buffer
    // Fast path: assumes preflight() has already validated descriptors.
    PreprocessResult run(const FrameDescriptor& src_nv12, ImageBuffer& dst_rgb,
                         LetterboxMeta* meta = nullptr) const noexcept;

  private:
    static LetterboxMeta _compute_letterbox(int src_w, int src_h, int dst_w, int dst_h);
    bool                 _init_letterbox();

    RgaPreprocessConfig _cfg{};
    bool             _initialized{false};
    LetterboxMeta    _lb{};
    im_rect          _srect{0, 0, 0, 0};
    im_rect          _drect{0, 0, 0, 0};
  };

} // namespace omniseer::vision
