#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace omniseer::vision
{
  enum class ComparisonQuadrant
  {
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight,
  };

  struct ComparisonModelSpec
  {
    const char*        label;
    const char*        artifact_name;
    ComparisonQuadrant quadrant;
  };

  constexpr std::array<ComparisonModelSpec, 4> kRunbundleComparisonModels{{
      {"v2-S FP", "yolo_world_v2_s_fp.rknn", ComparisonQuadrant::TopLeft},
      {"v2-S INT8", "yolo_world_v2_s_i8.rknn", ComparisonQuadrant::TopRight},
      {"v2-M FP", "yolo_world_v2_m_fp.rknn", ComparisonQuadrant::BottomLeft},
      {"v2-M INT8", "yolo_world_v2_m_i8.rknn", ComparisonQuadrant::BottomRight},
  }};

  struct ComparisonInputPaths
  {
    std::filesystem::path source_path{};
    std::filesystem::path model_dir{};
    std::filesystem::path class_list_path{};
  };

  /**
   * @brief Apply the documented RunBundle comparison defaults without changing path namespaces.
   *
   * Relative paths intentionally remain relative to the shell that invoked the comparison tool.
   */
  ComparisonInputPaths resolve_comparison_input_paths(const std::filesystem::path& run_dir,
                                                      const std::filesystem::path& repo_root,
                                                      const std::filesystem::path& model_dir,
                                                      const std::filesystem::path& class_list_path);

  /**
   * @brief Visit the four configurations in presentation order with one shared frame.
   *
   * This intentionally has no parallelism or ownership transfer: every visitor call
   * observes the exact same immutable corrected source-frame object.
   */
  template <typename Frame, typename Visitor>
  void visit_comparison_models(const Frame& corrected_frame, Visitor&& visitor)
  {
    for (size_t i = 0; i < kRunbundleComparisonModels.size(); ++i)
      visitor(i, kRunbundleComparisonModels[i], corrected_frame);
  }

  /** @brief Preserve input presentation timing, with a stable fallback for missing metadata. */
  double comparison_output_fps(double input_fps) noexcept;

  struct ComparisonProvenanceModel
  {
    std::string label{};
    std::string artifact_name{};
    std::string path{};
    std::string sha256{};
  };

  struct ComparisonProvenance
  {
    std::string                            source_path{};
    std::string                            source_sha256{};
    std::vector<std::string>               classes{};
    float                                  score_threshold{0.25F};
    float                                  nms_iou_threshold{0.45F};
    uint32_t                               max_detections{100};
    std::vector<ComparisonProvenanceModel> models{};
    std::string                            output_path{};
    std::string                            output_sha256{};
    double                                 output_fps{30.0};
    uint64_t                               source_frames_rendered{0};
    std::string                            git_sha{"unknown"};
  };

  /** @brief Serialize comparison provenance without any RunBundle manifest mutation. */
  std::string comparison_provenance_json(const ComparisonProvenance& provenance);
} // namespace omniseer::vision
