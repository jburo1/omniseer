#pragma once

#include <cstddef>
#include <cstdint>

#include "omniseer/vision/types.hpp"

namespace omniseer::vision
{
  /**
   * @brief Immutable view over prepared, quantized text embeddings for YOLO-World startup.
   */
  struct PreparedTextEmbeddingsView
  {
    /// @brief Pointer to contiguous int8 embedding bytes.
    const int8_t* data{nullptr};
    /// @brief Byte count available at @ref data.
    size_t bytes{0};
  };

  /**
   * @brief Immutable geometry/remap contract for the fixed pipeline setup.
   */
  struct PipelineRemapConfig
  {
    /// @brief Source capture frame dimensions in pixels.
    Size source_size{1280, 720};
    /// @brief Model input frame dimensions in pixels.
    Size model_input_size{640, 640};

    /// @brief Uniform resize scale from source to model input.
    float scale{0.5F};
    /// @brief Horizontal letterbox padding in destination pixels.
    int pad_x{0};
    /// @brief Vertical letterbox padding in destination pixels.
    int pad_y{140};
    /// @brief Resized content width before padding.
    int resized_w{640};
    /// @brief Resized content height before padding.
    int resized_h{360};
  };

  /**
   * @brief Producer-stage policy configuration.
   */
  struct ProducerPipelineConfig
  {
    /// @brief Maximum time to wait for one captured frame during producer preflight.
    uint32_t preflight_capture_wait_ms{200};
  };

  /**
   * @brief Consumer-stage policy configuration.
   *
   * v1 intentionally keeps this empty until consumer-owned policy knobs exist.
   */
  struct ConsumerPipelineConfig
  {
  };

  /**
   * @brief Startup handoff required to arm the consumer pipeline.
   */
  struct ConsumerPipelineStartup
  {
    /// @brief Immutable letterbox/remap geometry discovered during producer preflight.
    PipelineRemapConfig remap{};
    /// @brief Prepared quantized text embeddings for the RKNN text input.
    PreparedTextEmbeddingsView text_embeddings{};
  };

} // namespace omniseer::vision
