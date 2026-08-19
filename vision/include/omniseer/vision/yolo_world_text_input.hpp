#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <rknn_api.h>

#include <stdexcept>
#include <vector>

namespace omniseer::vision
{
  /** @brief Supported RKNN storage formats for the YOLO-World `texts` input. */
  enum class YoloWorldTextInputFormat : uint8_t
  {
    AffineInt8,
    Float32,
  };

  /** @brief Validate and classify the detector `texts` input storage contract. */
  inline YoloWorldTextInputFormat resolve_yolo_world_text_input_format(
      const rknn_tensor_attr& attr)
  {
    if (attr.type == RKNN_TENSOR_INT8)
    {
      if (attr.scale <= 0.0F)
        throw std::runtime_error(
            "YoloWorldTextEmbeddingsBuilder::build: detector INT8 texts tensor has invalid "
            "affine scale");
      return YoloWorldTextInputFormat::AffineInt8;
    }
    if (attr.type == RKNN_TENSOR_FLOAT32)
      return YoloWorldTextInputFormat::Float32;

    throw std::runtime_error(
        "YoloWorldTextEmbeddingsBuilder::build: unsupported detector texts tensor type; "
        "supported types are affine INT8 and FLOAT32");
  }

  /** @brief Copy one FP32 CLIP embedding into its detector text-input byte slot. */
  inline void copy_float32_yolo_world_text_embedding(std::vector<uint8_t>& storage,
                                                      size_t element_offset,
                                                      const float* values,
                                                      size_t value_count)
  {
    if (values == nullptr)
      throw std::invalid_argument("YOLO-World FLOAT32 text embedding is null");

    const size_t byte_offset = element_offset * sizeof(float);
    const size_t byte_count  = value_count * sizeof(float);
    if (byte_offset > storage.size() || byte_count > storage.size() - byte_offset)
      throw std::runtime_error("YOLO-World FLOAT32 text embedding exceeds detector input size");

    std::memcpy(storage.data() + byte_offset, values, byte_count);
  }
} // namespace omniseer::vision
