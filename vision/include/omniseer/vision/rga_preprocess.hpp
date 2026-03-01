#pragma once

#include <cstdint>
#include <im2d_type.h>

#include "omniseer/vision/types.hpp"

namespace omniseer::vision
{

  /**
   * @brief Runtime letterbox parameters produced by preprocessing.
   */
  struct LetterboxMeta
  {
    /// @brief Uniform resize scale applied to source dimensions.
    float scale{1.0f};
    /// @brief Horizontal padding applied on destination.
    int   pad_x{0};
    /// @brief Vertical padding applied on destination.
    int   pad_y{0};
    /// @brief Resized content width before padding.
    int   resized_w{0};
    /// @brief Resized content height before padding.
    int   resized_h{0};
  };

  /**
   * @brief Configuration for NV12-to-RGB preprocessing with letterboxing.
   */
  struct RgaPreprocessConfig
  {
    /// @brief Source stream width in pixels (NV12).
    int src_w{1280};
    /// @brief Source stream height in pixels (NV12).
    int src_h{720};

    /// @brief Destination width in pixels (model input).
    int dst_w{640};
    /// @brief Destination height in pixels (model input).
    int dst_h{640};
    /// @brief Padding fill value for letterbox borders.
    uint8_t pad_value{114};
  };

  /**
   * @brief NV12 (DMA-BUF) to RGB888 (DMA-BUF) transform via librga (im2d).
   *
   * Typical flow:
   * - Construct with RgaPreprocessConfig.
   * - prefill() destination buffers once with pad color.
   * - preflight() once at startup to validate descriptors and run a smoke pass.
   * - run() on each frame in the hot path.
   */
  class RgaPreprocess
  {
  public:
    /**
     * @brief Construct preprocess pipeline state from configuration.
     *
     * @throws std::invalid_argument when source/destination geometry is invalid.
     */
    explicit RgaPreprocess(RgaPreprocessConfig cfg = {});

    /**
     * @brief Prefill destination buffer with pad color.
     *
     * This is typically done once so per-frame work only overwrites the resized image region.
     *
     * @throws On invalid destination descriptors or DMA-BUF CPU mapping failures.
     */
    void prefill(ImageBuffer& dst_rgb) const;

    /**
     * @brief One-time descriptor/config validation plus RGA smoke pass.
     *
     * Expected usage: call once during startup before entering the frame loop.
     */
    PreprocessResult preflight(const FrameDescriptor& src_nv12, ImageBuffer& dst_rgb,
                               LetterboxMeta* meta = nullptr) const noexcept;

    /**
     * @brief Run one-frame NV12-to-RGB preprocessing transform.
     *
     * Fast path: assumes preflight() has already validated descriptors.
     *
     * @param src_nv12 NV12 DMA-BUF frame descriptor from V4L2.
     * @param dst_rgb Destination model-input buffer descriptor.
     * @param meta Optional output letterbox metadata for post-processing.
     */
    PreprocessResult run(const FrameDescriptor& src_nv12, ImageBuffer& dst_rgb,
                         LetterboxMeta* meta = nullptr) const noexcept;

  private:
    /// @brief Compute centered letterbox scale and padding from source/destination geometry.
    static LetterboxMeta _compute_letterbox(int src_w, int src_h, int dst_w, int dst_h);
    /// @brief Initialize cached RGA rectangles from current configuration.
    bool                 _init_letterbox();

    /// @brief Immutable preprocessing configuration.
    RgaPreprocessConfig _cfg{};
    /// @brief True once letterbox-derived rectangles are initialized.
    bool _initialized{false};
    /// @brief Cached letterbox metadata.
    LetterboxMeta _lb{};
    /// @brief Cached source rectangle for im2d calls.
    im_rect _srect{0, 0, 0, 0};
    /// @brief Cached destination rectangle for im2d calls.
    im_rect _drect{0, 0, 0, 0};
  };

} // namespace omniseer::vision
