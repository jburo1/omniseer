#include "omniseer/vision/yolo_world_postprocess.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace omniseer::vision
{
  namespace
  {
    struct Candidate
    {
      uint16_t class_id{0};
      float    score{0.0F};
      float    x1{0.0F};
      float    y1{0.0F};
      float    x2{0.0F};
      float    y2{0.0F};
    };

    constexpr std::array<uint32_t, 3> kExpectedGridSizes{80u, 40u, 20u};

    float clamp_float(float value, float lo, float hi) noexcept
    {
      return std::min(std::max(value, lo), hi);
    }

    int8_t quantize_threshold(float value, int32_t zp, float scale) noexcept
    {
      if (scale <= 0.0F)
        return std::numeric_limits<int8_t>::max();
      const float quantized = (value / scale) + static_cast<float>(zp);
      return static_cast<int8_t>(clamp_float(quantized, -128.0F, 127.0F));
    }

    float dequantize_i8(int8_t value, int32_t zp, float scale) noexcept
    {
      return (static_cast<float>(value) - static_cast<float>(zp)) * scale;
    }

    size_t find_grid_scale_index(uint32_t grid_size)
    {
      for (size_t i = 0; i < kExpectedGridSizes.size(); ++i)
      {
        if (kExpectedGridSizes[i] == grid_size)
          return i;
      }
      return kExpectedGridSizes.size();
    }

    bool is_supported_output_desc(const RknnOutputDesc& desc) noexcept
    {
      return desc.n_dims == 4 && desc.dims[0] == 1 && desc.dims[2] == desc.dims[3];
    }

    void collect_int8_candidates(const int8_t* box_tensor, const RknnOutputDesc& box_desc,
                                 const int8_t* class_tensor, const RknnOutputDesc& class_desc,
                                 uint32_t model_input_size, uint32_t active_class_count,
                                 float score_threshold, std::vector<Candidate>& candidates) noexcept
    {
      if (box_tensor == nullptr || class_tensor == nullptr)
        return;
      if (box_desc.scale <= 0.0F || class_desc.scale <= 0.0F)
        return;

      const uint32_t grid_h   = box_desc.dims[2];
      const uint32_t grid_w   = box_desc.dims[3];
      const size_t   grid_len = static_cast<size_t>(grid_h) * static_cast<size_t>(grid_w);
      const float    stride =
          (grid_h == 0) ? 0.0F : static_cast<float>(model_input_size) / static_cast<float>(grid_h);
      if (grid_len == 0 || stride <= 0.0F)
        return;

      const int8_t score_threshold_i8 =
          quantize_threshold(score_threshold, class_desc.zero_point, class_desc.scale);

      for (uint32_t y = 0; y < grid_h; ++y)
      {
        for (uint32_t x = 0; x < grid_w; ++x)
        {
          const size_t cell_offset = static_cast<size_t>(y) * grid_w + x;

          int8_t   max_score_q = score_threshold_i8;
          uint32_t best_class  = 0;
          bool     found       = false;
          for (uint32_t class_id = 0; class_id < active_class_count; ++class_id)
          {
            const int8_t score_q = class_tensor[cell_offset + static_cast<size_t>(class_id) * grid_len];
            if (score_q > score_threshold_i8 && (!found || score_q > max_score_q))
            {
              max_score_q = score_q;
              best_class  = class_id;
              found       = true;
            }
          }
          if (!found)
            continue;

          const float left =
              dequantize_i8(box_tensor[cell_offset + 0u * grid_len], box_desc.zero_point, box_desc.scale);
          const float top =
              dequantize_i8(box_tensor[cell_offset + 1u * grid_len], box_desc.zero_point, box_desc.scale);
          const float right =
              dequantize_i8(box_tensor[cell_offset + 2u * grid_len], box_desc.zero_point, box_desc.scale);
          const float bottom =
              dequantize_i8(box_tensor[cell_offset + 3u * grid_len], box_desc.zero_point, box_desc.scale);

          const float center_x = (static_cast<float>(x) + 0.5F) * stride;
          const float center_y = (static_cast<float>(y) + 0.5F) * stride;

          Candidate candidate{};
          candidate.class_id = static_cast<uint16_t>(best_class);
          candidate.score    = dequantize_i8(max_score_q, class_desc.zero_point, class_desc.scale);
          candidate.x1       = center_x - left * stride;
          candidate.y1       = center_y - top * stride;
          candidate.x2       = center_x + right * stride;
          candidate.y2       = center_y + bottom * stride;
          candidates.push_back(candidate);
        }
      }
    }

    void collect_fp32_candidates(const float* box_tensor, const RknnOutputDesc& box_desc,
                                 const float* class_tensor, const RknnOutputDesc& class_desc,
                                 uint32_t model_input_size, uint32_t active_class_count,
                                 float score_threshold, std::vector<Candidate>& candidates) noexcept
    {
      (void) class_desc;
      if (box_tensor == nullptr || class_tensor == nullptr)
        return;

      const uint32_t grid_h   = box_desc.dims[2];
      const uint32_t grid_w   = box_desc.dims[3];
      const size_t   grid_len = static_cast<size_t>(grid_h) * static_cast<size_t>(grid_w);
      const float    stride =
          (grid_h == 0) ? 0.0F : static_cast<float>(model_input_size) / static_cast<float>(grid_h);
      if (grid_len == 0 || stride <= 0.0F)
        return;

      for (uint32_t y = 0; y < grid_h; ++y)
      {
        for (uint32_t x = 0; x < grid_w; ++x)
        {
          const size_t cell_offset = static_cast<size_t>(y) * grid_w + x;

          float    max_score = score_threshold;
          uint32_t best_class = 0;
          bool     found      = false;
          for (uint32_t class_id = 0; class_id < active_class_count; ++class_id)
          {
            const float score = class_tensor[cell_offset + static_cast<size_t>(class_id) * grid_len];
            if (score > score_threshold && (!found || score > max_score))
            {
              max_score  = score;
              best_class = class_id;
              found      = true;
            }
          }
          if (!found)
            continue;

          const float left   = box_tensor[cell_offset + 0u * grid_len];
          const float top    = box_tensor[cell_offset + 1u * grid_len];
          const float right  = box_tensor[cell_offset + 2u * grid_len];
          const float bottom = box_tensor[cell_offset + 3u * grid_len];

          const float center_x = (static_cast<float>(x) + 0.5F) * stride;
          const float center_y = (static_cast<float>(y) + 0.5F) * stride;

          Candidate candidate{};
          candidate.class_id = static_cast<uint16_t>(best_class);
          candidate.score    = max_score;
          candidate.x1       = center_x - left * stride;
          candidate.y1       = center_y - top * stride;
          candidate.x2       = center_x + right * stride;
          candidate.y2       = center_y + bottom * stride;
          candidates.push_back(candidate);
        }
      }
    }

    float compute_iou(const Candidate& a, const Candidate& b) noexcept
    {
      const float inter_x1 = std::max(a.x1, b.x1);
      const float inter_y1 = std::max(a.y1, b.y1);
      const float inter_x2 = std::min(a.x2, b.x2);
      const float inter_y2 = std::min(a.y2, b.y2);
      const float inter_w  = std::max(0.0F, inter_x2 - inter_x1);
      const float inter_h  = std::max(0.0F, inter_y2 - inter_y1);
      const float inter    = inter_w * inter_h;
      const float area_a   = std::max(0.0F, a.x2 - a.x1) * std::max(0.0F, a.y2 - a.y1);
      const float area_b   = std::max(0.0F, b.x2 - b.x1) * std::max(0.0F, b.y2 - b.y1);
      const float denom    = area_a + area_b - inter;
      return (denom > 0.0F) ? (inter / denom) : 0.0F;
    }

    void remap_to_source(const Candidate& candidate, const PipelineRemapConfig& remap,
                         Detection& out) noexcept
    {
      const float model_w = static_cast<float>(remap.model_input_size.w);
      const float model_h = static_cast<float>(remap.model_input_size.h);
      const float src_w   = static_cast<float>(remap.source_size.w);
      const float src_h   = static_cast<float>(remap.source_size.h);

      const float model_x1 = clamp_float(candidate.x1 - static_cast<float>(remap.pad_x), 0.0F, model_w);
      const float model_y1 = clamp_float(candidate.y1 - static_cast<float>(remap.pad_y), 0.0F, model_h);
      const float model_x2 = clamp_float(candidate.x2 - static_cast<float>(remap.pad_x), 0.0F, model_w);
      const float model_y2 = clamp_float(candidate.y2 - static_cast<float>(remap.pad_y), 0.0F, model_h);

      out.class_id = candidate.class_id;
      out.score    = candidate.score;
      out.x1       = clamp_float(model_x1 / remap.scale, 0.0F, src_w);
      out.y1       = clamp_float(model_y1 / remap.scale, 0.0F, src_h);
      out.x2       = clamp_float(model_x2 / remap.scale, 0.0F, src_w);
      out.y2       = clamp_float(model_y2 / remap.scale, 0.0F, src_h);
    }
  } // namespace

  YoloWorldOutputLayout resolve_yolo_world_output_layout(const std::vector<RknnOutputDesc>& descs)
  {
    YoloWorldOutputLayout layout{};
    layout.class_output_indices.fill(UINT32_MAX);
    layout.box_output_indices.fill(UINT32_MAX);
    layout.grid_sizes = kExpectedGridSizes;

    for (const RknnOutputDesc& desc : descs)
    {
      if (!is_supported_output_desc(desc))
        throw std::runtime_error("resolve_yolo_world_output_layout: unsupported RKNN output shape");

      const size_t scale_index = find_grid_scale_index(desc.dims[2]);
      if (scale_index == kExpectedGridSizes.size())
        throw std::runtime_error("resolve_yolo_world_output_layout: unexpected RKNN output grid size");

      if (desc.dims[1] == 4)
      {
        if (layout.box_output_indices[scale_index] != UINT32_MAX)
          throw std::runtime_error("resolve_yolo_world_output_layout: duplicate RKNN box output");
        layout.box_output_indices[scale_index] = desc.index;
        continue;
      }

      if (layout.class_capacity == 0)
        layout.class_capacity = desc.dims[1];
      else if (layout.class_capacity != desc.dims[1])
        throw std::runtime_error(
            "resolve_yolo_world_output_layout: inconsistent RKNN class output channel count");

      if (layout.class_output_indices[scale_index] != UINT32_MAX)
        throw std::runtime_error("resolve_yolo_world_output_layout: duplicate RKNN class output");
      layout.class_output_indices[scale_index] = desc.index;
    }

    if (layout.class_capacity == 0)
      throw std::runtime_error("resolve_yolo_world_output_layout: missing RKNN class outputs");

    for (size_t i = 0; i < kExpectedGridSizes.size(); ++i)
    {
      if (layout.class_output_indices[i] == UINT32_MAX || layout.box_output_indices[i] == UINT32_MAX)
        throw std::runtime_error("resolve_yolo_world_output_layout: incomplete RKNN output layout");
    }

    return layout;
  }

  void decode_yolo_world_detections(const std::vector<RknnOutputView>& outputs,
                                    const std::vector<RknnOutputDesc>& output_descs,
                                    const YoloWorldOutputLayout& layout,
                                    const ConsumerPipelineConfig& cfg,
                                    const PipelineRemapConfig& remap,
                                    uint32_t active_class_count,
                                    DetectionsFrame& frame) noexcept
  {
    frame.count = 0;

    if (active_class_count == 0 || remap.scale <= 0.0F)
      return;
    if (outputs.size() != output_descs.size())
      return;

    std::vector<Candidate> candidates{};
    candidates.reserve(80u * 80u + 40u * 40u + 20u * 20u);

    for (size_t scale = 0; scale < layout.grid_sizes.size(); ++scale)
    {
      const uint32_t class_index = layout.class_output_indices[scale];
      const uint32_t box_index   = layout.box_output_indices[scale];
      if (class_index >= outputs.size() || box_index >= outputs.size())
        return;

      const RknnOutputDesc& class_desc = output_descs[class_index];
      const RknnOutputDesc& box_desc   = output_descs[box_index];
      if (class_desc.dims[2] != layout.grid_sizes[scale] || box_desc.dims[2] != layout.grid_sizes[scale])
        return;
      if (class_desc.dims[1] < active_class_count || box_desc.dims[1] != 4)
        return;

      if (class_desc.type == RKNN_TENSOR_INT8 && box_desc.type == RKNN_TENSOR_INT8)
      {
        collect_int8_candidates(static_cast<const int8_t*>(outputs[box_index].data), box_desc,
                                static_cast<const int8_t*>(outputs[class_index].data), class_desc,
                                remap.model_input_size.h, active_class_count, cfg.score_threshold, candidates);
        continue;
      }

      if (class_desc.type == RKNN_TENSOR_FLOAT32 && box_desc.type == RKNN_TENSOR_FLOAT32)
      {
        collect_fp32_candidates(static_cast<const float*>(outputs[box_index].data), box_desc,
                                static_cast<const float*>(outputs[class_index].data), class_desc,
                                remap.model_input_size.h, active_class_count, cfg.score_threshold, candidates);
        continue;
      }

      return;
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& lhs, const Candidate& rhs) { return lhs.score > rhs.score; });

    const uint32_t max_detections = std::min<uint32_t>(cfg.max_detections, DetectionsFrame::capacity);
    std::vector<Candidate> accepted{};
    accepted.reserve(max_detections);

    for (const Candidate& candidate : candidates)
    {
      bool suppressed = false;
      for (const Candidate& kept : accepted)
      {
        if (kept.class_id != candidate.class_id)
          continue;
        if (compute_iou(kept, candidate) > cfg.nms_iou_threshold)
        {
          suppressed = true;
          break;
        }
      }
      if (suppressed)
        continue;

      Detection& out = frame.detections[frame.count];
      remap_to_source(candidate, remap, out);
      if (out.x2 <= out.x1 || out.y2 <= out.y1)
        continue;

      accepted.push_back(candidate);
      frame.count += 1;
      if (frame.count >= max_detections)
        break;
    }
  }
} // namespace omniseer::vision
