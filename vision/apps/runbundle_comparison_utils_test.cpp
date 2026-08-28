#include <cmath>
#include <gtest/gtest.h>

#include "omniseer/vision/runbundle_comparison.hpp"

namespace omniseer::vision
{
  TEST(RunbundleComparison, UsesTheRequiredModelArtifactAndQuadrantMapping)
  {
    ASSERT_EQ(kRunbundleComparisonModels.size(), 4U);
    EXPECT_STREQ(kRunbundleComparisonModels[0].label, "v2-S FP");
    EXPECT_STREQ(kRunbundleComparisonModels[0].artifact_name, "yolo_world_v2_s_fp.rknn");
    EXPECT_STREQ(kRunbundleComparisonModels[0].detections_filename, "v2s_fp.jsonl");
    EXPECT_EQ(kRunbundleComparisonModels[0].quadrant, ComparisonQuadrant::TopLeft);
    EXPECT_STREQ(kRunbundleComparisonModels[1].label, "v2-S INT8");
    EXPECT_STREQ(kRunbundleComparisonModels[1].detections_filename, "v2s_int8.jsonl");
    EXPECT_EQ(kRunbundleComparisonModels[1].quadrant, ComparisonQuadrant::TopRight);
    EXPECT_STREQ(kRunbundleComparisonModels[2].label, "v2-M FP");
    EXPECT_STREQ(kRunbundleComparisonModels[2].detections_filename, "v2m_fp.jsonl");
    EXPECT_EQ(kRunbundleComparisonModels[2].quadrant, ComparisonQuadrant::BottomLeft);
    EXPECT_STREQ(kRunbundleComparisonModels[3].label, "v2-M INT8");
    EXPECT_STREQ(kRunbundleComparisonModels[3].detections_filename, "v2m_int8.jsonl");
    EXPECT_EQ(kRunbundleComparisonModels[3].quadrant, ComparisonQuadrant::BottomRight);
  }

  TEST(RunbundleComparison, ResolvesDocumentedRunbundleInputDefaults)
  {
    const auto paths = resolve_comparison_input_paths("runs/demo_001", "/omniseer", {}, {});

    EXPECT_EQ(paths.source_path, "runs/demo_001/video/source.ts");
    EXPECT_EQ(paths.class_list_path, "runs/demo_001/classes.txt");
    EXPECT_EQ(paths.model_dir, "/omniseer/runs/model_artifacts");
  }

  TEST(RunbundleComparison, PreservesExplicitInputOverrides)
  {
    const auto paths = resolve_comparison_input_paths("runs/demo_001", "/omniseer", "/models",
                                                      "custom/classes.txt");

    EXPECT_EQ(paths.model_dir, "/models");
    EXPECT_EQ(paths.class_list_path, "custom/classes.txt");
  }

  TEST(RunbundleComparison, OutputFpsDoesNotDependOnOfflineProcessingDuration)
  {
    EXPECT_DOUBLE_EQ(comparison_output_fps(29.97), 29.97);
    EXPECT_DOUBLE_EQ(comparison_output_fps(0.0), 30.0);
    EXPECT_DOUBLE_EQ(comparison_output_fps(-1.0), 30.0);
  }

  TEST(RunbundleComparison, AcceptsOnlyConservativeComparisonDirectoryNames)
  {
    EXPECT_TRUE(comparison_name_is_safe("default"));
    EXPECT_TRUE(comparison_name_is_safe("coco80-v2"));
    EXPECT_FALSE(comparison_name_is_safe(""));
    EXPECT_FALSE(comparison_name_is_safe(".."));
    EXPECT_FALSE(comparison_name_is_safe("task/results"));
    EXPECT_FALSE(comparison_name_is_safe("task name"));
  }

  TEST(RunbundleComparison, UsesPresentationTimestampWithFrameRateFallback)
  {
    EXPECT_DOUBLE_EQ(comparison_source_timestamp_sec(1250.0, 99, 30.0), 1.25);
    EXPECT_DOUBLE_EQ(comparison_source_timestamp_sec(-1.0, 3, 20.0), 0.15);
    EXPECT_DOUBLE_EQ(comparison_source_timestamp_sec(NAN, 3, 0.0), 0.0);
  }

  TEST(RunbundleComparison, EveryModelVisitReceivesTheSameCorrectedSourceObject)
  {
    const int                 corrected_frame = 42;
    std::array<const int*, 4> received{};
    std::array<size_t, 4>     order{};
    visit_comparison_models(corrected_frame,
                            [&](size_t index, const ComparisonModelSpec&, const int& frame)
                            {
                              received[index] = &frame;
                              order[index]    = index;
                            });
    for (size_t i = 0; i < received.size(); ++i)
    {
      EXPECT_EQ(received[i], &corrected_frame);
      EXPECT_EQ(order[i], i);
    }
  }

  TEST(RunbundleComparison, ProvenanceCapturesRawSourceRepairAndModelIdentity)
  {
    ComparisonProvenance provenance{};
    provenance.source_path     = "video/source.ts";
    provenance.comparison_name = "task";
    provenance.source_sha256   = "source-hash";
    provenance.classes         = {"chair"};
    provenance.models.push_back(
        {"v2-S FP", "yolo_world_v2_s_fp.rknn", "/models/s.rknn", "model-hash"});
    provenance.output_path           = "video/comparison/comparison.mp4";
    provenance.output_sha256         = "output-hash";
    provenance.detection_jsonl_paths = {"video/comparison/task/v2s_fp.jsonl"};
    const std::string json           = comparison_provenance_json(provenance);
    EXPECT_NE(json.find("video/source.ts"), std::string::npos);
    EXPECT_NE(json.find("rockchip_preview_circular_wrap_v1"), std::string::npos);
    EXPECT_NE(json.find("rotate_left_px"), std::string::npos);
    EXPECT_NE(json.find("yolo_world_v2_s_fp.rknn"), std::string::npos);
    EXPECT_NE(json.find("\"schema_version\": 2"), std::string::npos);
    EXPECT_NE(json.find("\"comparison_name\": \"task\""), std::string::npos);
    EXPECT_NE(json.find("v2s_fp.jsonl"), std::string::npos);
    EXPECT_NE(json.find("offline processing duration does not change"), std::string::npos);
  }
} // namespace omniseer::vision
