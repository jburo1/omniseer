#include <gtest/gtest.h>

#include "omniseer/vision/runbundle_comparison.hpp"

namespace omniseer::vision
{
  TEST(RunbundleComparison, UsesTheRequiredModelArtifactAndQuadrantMapping)
  {
    ASSERT_EQ(kRunbundleComparisonModels.size(), 4U);
    EXPECT_STREQ(kRunbundleComparisonModels[0].label, "v2-S FP");
    EXPECT_STREQ(kRunbundleComparisonModels[0].artifact_name, "yolo_world_v2_s_fp.rknn");
    EXPECT_EQ(kRunbundleComparisonModels[0].quadrant, ComparisonQuadrant::TopLeft);
    EXPECT_STREQ(kRunbundleComparisonModels[1].label, "v2-S INT8");
    EXPECT_EQ(kRunbundleComparisonModels[1].quadrant, ComparisonQuadrant::TopRight);
    EXPECT_STREQ(kRunbundleComparisonModels[2].label, "v2-M FP");
    EXPECT_EQ(kRunbundleComparisonModels[2].quadrant, ComparisonQuadrant::BottomLeft);
    EXPECT_STREQ(kRunbundleComparisonModels[3].label, "v2-M INT8");
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
    provenance.source_path   = "video/source.ts";
    provenance.source_sha256 = "source-hash";
    provenance.classes       = {"chair"};
    provenance.models.push_back(
        {"v2-S FP", "yolo_world_v2_s_fp.rknn", "/models/s.rknn", "model-hash"});
    provenance.output_path   = "video/comparison/comparison.mp4";
    provenance.output_sha256 = "output-hash";
    const std::string json   = comparison_provenance_json(provenance);
    EXPECT_NE(json.find("video/source.ts"), std::string::npos);
    EXPECT_NE(json.find("rockchip_preview_circular_wrap_v1"), std::string::npos);
    EXPECT_NE(json.find("rotate_left_px"), std::string::npos);
    EXPECT_NE(json.find("yolo_world_v2_s_fp.rknn"), std::string::npos);
    EXPECT_NE(json.find("offline processing duration does not change"), std::string::npos);
  }
} // namespace omniseer::vision
