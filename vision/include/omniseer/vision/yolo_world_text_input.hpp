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
    Float16,
    Float32,
  };

  /** @brief Validate and classify the detector `texts` input storage contract. */
  inline YoloWorldTextInputFormat resolve_yolo_world_text_input_format(const rknn_tensor_attr& attr)
  {
    if (attr.type == RKNN_TENSOR_INT8)
    {
      if (attr.scale <= 0.0F)
        throw std::runtime_error(
            "YoloWorldTextEmbeddingsBuilder::build: detector INT8 texts tensor has invalid "
            "affine scale");
      return YoloWorldTextInputFormat::AffineInt8;
    }
    if (attr.type == RKNN_TENSOR_FLOAT16)
      return YoloWorldTextInputFormat::Float16;
    if (attr.type == RKNN_TENSOR_FLOAT32)
      return YoloWorldTextInputFormat::Float32;

    throw std::runtime_error(
        "YoloWorldTextEmbeddingsBuilder::build: unsupported detector texts tensor type; "
        "supported types are affine INT8, FLOAT16, and FLOAT32");
  }

  /** @brief Convert an FP32 value to its IEEE 754 binary16 representation. */
  inline uint16_t float32_to_float16_bits(float value) noexcept
  {
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));

    const uint32_t sign     = (bits >> 16) & 0x8000U;
    const uint32_t exponent = (bits >> 23) & 0xffU;
    uint32_t       mantissa = bits & 0x7fffffU;

    if (exponent == 0xffU)
      return static_cast<uint16_t>(sign | (mantissa == 0 ? 0x7c00U : 0x7e00U));

    const int32_t half_exponent = static_cast<int32_t>(exponent) - 127 + 15;
    if (half_exponent >= 31)
      return static_cast<uint16_t>(sign | 0x7c00U);
    if (half_exponent <= 0)
    {
      if (half_exponent < -10)
        return static_cast<uint16_t>(sign);

      mantissa |= 0x800000U;
      const uint32_t shift     = static_cast<uint32_t>(14 - half_exponent);
      uint32_t       half      = mantissa >> shift;
      const uint32_t remainder = mantissa & ((1U << shift) - 1U);
      const uint32_t halfway   = 1U << (shift - 1);
      if (remainder > halfway || (remainder == halfway && (half & 1U)))
        ++half;
      return static_cast<uint16_t>(sign | half);
    }

    uint32_t       half      = (static_cast<uint32_t>(half_exponent) << 10) | (mantissa >> 13);
    const uint32_t remainder = mantissa & 0x1fffU;
    if (remainder > 0x1000U || (remainder == 0x1000U && (half & 1U)))
      ++half;
    return static_cast<uint16_t>(sign | half);
  }

  /** @brief Copy one FP32 CLIP embedding into its detector text-input byte slot. */
  inline void copy_float32_yolo_world_text_embedding(std::vector<uint8_t>& storage,
                                                     size_t element_offset, const float* values,
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

  /** @brief Convert one FP32 CLIP embedding into its FP16 detector text-input byte slot. */
  inline void copy_float16_yolo_world_text_embedding(std::vector<uint8_t>& storage,
                                                     size_t element_offset, const float* values,
                                                     size_t value_count)
  {
    if (values == nullptr)
      throw std::invalid_argument("YOLO-World FLOAT16 text embedding is null");

    const size_t byte_offset = element_offset * sizeof(uint16_t);
    const size_t byte_count  = value_count * sizeof(uint16_t);
    if (byte_offset > storage.size() || byte_count > storage.size() - byte_offset)
      throw std::runtime_error("YOLO-World FLOAT16 text embedding exceeds detector input size");

    for (size_t i = 0; i < value_count; ++i)
    {
      const uint16_t packed = float32_to_float16_bits(values[i]);
      std::memcpy(storage.data() + byte_offset + (i * sizeof(packed)), &packed, sizeof(packed));
    }
  }
} // namespace omniseer::vision
