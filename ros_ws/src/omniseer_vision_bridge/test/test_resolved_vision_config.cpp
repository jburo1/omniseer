#include <chrono>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iterator>
#include <string>
#include <unistd.h>

#include "omniseer_vision_bridge/resolved_vision_config.hpp"

namespace
{
  std::filesystem::path make_temp_dir()
  {
    const auto root = std::filesystem::temp_directory_path() /
                      ("omniseer_resolved_config_test_" + std::to_string(::getpid()) + "_" +
                       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(root);
    return root;
  }

  std::string read_text(const std::filesystem::path& path)
  {
    std::ifstream input(path);
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
  }
} // namespace

TEST(ResolvedVisionConfigTest, WritesResolvedComparisonSettings)
{
  const auto root        = make_temp_dir();
  const auto config_path = root / "provenance" / "resolved_vision_config.yaml";

  omniseer_vision_bridge::VisionBridgeRuntimeConfig config{};
  config.camera_device        = "/dev/video11";
  config.camera_width         = 1920;
  config.camera_height        = 1080;
  config.camera_buffer_count  = 6;
  config.pipeline_dst_width   = 960;
  config.pipeline_dst_height  = 960;
  config.detector_model_path  = "/models/yolo-world-v2s-int8.rknn";
  config.clip_model_path      = "/models/clip.rknn";
  config.clip_vocab_path      = "/models/vocab.bpe";
  config.class_list_path      = "/models/classes.txt";
  config.pad_token            = "nothing";
  config.runner_warmup_runs   = 3;
  config.score_threshold      = 0.35;
  config.nms_iou_threshold    = 0.55;
  config.max_detections       = 42;
  config.camera_frame_id      = "front_camera_optical";
  config.resolved_config_path = config_path.string();

  std::string error;
  ASSERT_TRUE(omniseer_vision_bridge::write_resolved_vision_config(config, &error)) << error;
  ASSERT_TRUE(std::filesystem::is_regular_file(config_path));

  const auto content = read_text(config_path);
  EXPECT_NE(content.find("device: \"/dev/video11\""), std::string::npos);
  EXPECT_NE(content.find("width: 1920"), std::string::npos);
  EXPECT_NE(content.find("buffer_count: 6"), std::string::npos);
  EXPECT_NE(content.find("dst_width: 960"), std::string::npos);
  EXPECT_NE(content.find("detector_model_path: \"/models/yolo-world-v2s-int8.rknn\""),
            std::string::npos);
  EXPECT_NE(content.find("clip_model_path: \"/models/clip.rknn\""), std::string::npos);
  EXPECT_NE(content.find("clip_vocab_path: \"/models/vocab.bpe\""), std::string::npos);
  EXPECT_NE(content.find("pad_token: \"nothing\""), std::string::npos);
  EXPECT_NE(content.find("warmup_runs: 3"), std::string::npos);
  EXPECT_NE(content.find("score_threshold: 0.35"), std::string::npos);
  EXPECT_NE(content.find("nms_iou_threshold: 0.55"), std::string::npos);
  EXPECT_NE(content.find("max_detections: 42"), std::string::npos);
  EXPECT_NE(content.find("camera_frame_id: \"front_camera_optical\""), std::string::npos);

  std::filesystem::remove_all(root);
}
