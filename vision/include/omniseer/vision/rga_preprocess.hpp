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
    // If your V4L2 capture size differs, pass it explicitly.
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
    explicit RgaPreprocess(RgaPreprocessConfig cfg = {});

    // Prefill the destination buffer once with the padding value so per-frame
    // work only overwrites the resized image rectangle.
    bool prefill(ImageBuffer& dst_rgb) const;

    // Run the transformation
    // src_nv12: NV12 dmabuf frame from V4L2
    // dst_rgb : model input buffer
    bool run(const FrameDescriptor& src_nv12, ImageBuffer& dst_rgb,
             LetterboxMeta* meta = nullptr) const;

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
