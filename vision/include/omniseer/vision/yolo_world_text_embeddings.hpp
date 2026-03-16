#pragma once

#include <string>
#include <vector>

#include "omniseer/vision/config.hpp"

namespace omniseer::vision
{
  /**
   * @brief Owned prepared YOLO-World text embeddings plus source class names.
   */
  struct PreparedTextEmbeddings
  {
    std::vector<std::string> class_names{};
    std::vector<int8_t>      text_i8{};

    PreparedTextEmbeddingsView view() const noexcept
    {
      return {
          .data               = text_i8.data(),
          .bytes              = text_i8.size(),
          .active_class_count = static_cast<uint32_t>(class_names.size()),
      };
    }
  };

  /**
   * @brief Startup configuration for CLIP-text -> YOLO-World text embedding preparation.
   */
  struct YoloWorldTextEmbeddingsBuilderConfig
  {
    /// @brief RKNN CLIP text encoder model path.
    std::string text_encoder_model_path{};
    /// @brief RKNN YOLO-World detector model path used to query `texts` input quantization.
    std::string detector_model_path{};
    /// @brief Raw CLIP BPE merges/vocab asset path.
    std::string clip_vocab_path{};
    /// @brief Pad phrase used to fill unused detector text slots.
    std::string pad_token{"nothing"};
  };

  /**
   * @brief Build quantized YOLO-World text input tensors from user class names.
   *
   * This is a startup-only helper. It owns tokenization, CLIP text inference, and
   * quantization into the detector model's `texts` input format.
   */
  class YoloWorldTextEmbeddingsBuilder
  {
  public:
    explicit YoloWorldTextEmbeddingsBuilder(YoloWorldTextEmbeddingsBuilderConfig cfg = {});

    PreparedTextEmbeddings build(const std::vector<std::string>& class_names) const;

  private:
    YoloWorldTextEmbeddingsBuilderConfig _cfg{};
  };
} // namespace omniseer::vision
