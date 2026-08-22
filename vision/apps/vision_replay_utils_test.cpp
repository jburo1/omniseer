#include <cstdio>
#include <fstream>
#include <gtest/gtest.h>
#include <string>
#include <unistd.h>
#include <vector>

#include "omniseer/vision/letterbox.hpp"
#include "omniseer/vision/replay_jsonl.hpp"
#include "omniseer/vision/vision_replay_config.hpp"

namespace
{
  std::string temporary_path(const char* suffix)
  {
    return "/tmp/omniseer_vision_replay_" + std::to_string(::getpid()) + suffix;
  }

  std::vector<char*> argv_from(std::vector<std::string>& args)
  {
    std::vector<char*> argv{};
    argv.reserve(args.size());
    for (std::string& arg : args)
      argv.push_back(arg.data());
    return argv;
  }
} // namespace

TEST(VisionReplayLetterboxTest, MatchesPipelineRemapConventionForLandscapeSource)
{
  const auto remap = omniseer::vision::make_letterbox_remap({1280, 720}, {640, 640});

  EXPECT_FLOAT_EQ(remap.scale, 0.5F);
  EXPECT_EQ(remap.pad_x, 0);
  EXPECT_EQ(remap.pad_y, 140);
  EXPECT_EQ(remap.resized_w, 640);
  EXPECT_EQ(remap.resized_h, 360);
}

TEST(VisionReplayLetterboxTest, CentersPortraitSourceWithoutChangingRemapContract)
{
  const auto remap = omniseer::vision::make_letterbox_remap({480, 640}, {640, 640});

  EXPECT_FLOAT_EQ(remap.scale, 1.0F);
  EXPECT_EQ(remap.pad_x, 80);
  EXPECT_EQ(remap.pad_y, 0);
  EXPECT_EQ(remap.resized_w, 480);
  EXPECT_EQ(remap.resized_h, 640);
}

TEST(VisionReplayJsonlTest, WritesOneRecordPerFrameIncludingEmptyDetections)
{
  const std::string              output_path = temporary_path(".jsonl");
  const std::vector<std::string> class_names{"person", "bus"};
  {
    omniseer::vision::ReplayJsonlWriter writer(output_path, class_names);
    omniseer::vision::DetectionsFrame   empty{};
    writer.write(0, 0.0, empty);

    omniseer::vision::DetectionsFrame detected{};
    detected.count                  = 1;
    detected.detections[0].class_id = 1;
    detected.detections[0].score    = 0.75F;
    detected.detections[0].x1       = 10.0F;
    detected.detections[0].y1       = 20.0F;
    detected.detections[0].x2       = 30.0F;
    detected.detections[0].y2       = 40.0F;
    writer.write(1, 1.0 / 30.0, detected);
  }

  std::ifstream input(output_path);
  ASSERT_TRUE(input.good());
  std::string first{};
  std::string second{};
  EXPECT_TRUE(std::getline(input, first));
  EXPECT_TRUE(std::getline(input, second));
  EXPECT_FALSE(std::getline(input, first));
  EXPECT_EQ(first, "{\"frame_index\":0,\"timestamp_sec\":0.000000000,\"detections\":[]}");
  EXPECT_EQ(second,
            "{\"frame_index\":1,\"timestamp_sec\":0.033333333,\"detections\":[{\"class_id\":1,"
            "\"class_name\":\"bus\",\"score\":0.750000000,\"bbox\":[10.000000000,20.000000000,"
            "30.000000000,40.000000000]}]}");
  EXPECT_EQ(std::remove(output_path.c_str()), 0);
}

TEST(VisionReplayConfigTest, RejectsMissingReplayInputs)
{
  omniseer::vision::VisionReplayConfig config{};
  std::vector<std::string>             args{"vision_replay", "--video", "input.mp4"};
  std::vector<char*>                   argv = argv_from(args);
  std::string                          error{};
  bool                                 help_requested = false;

  EXPECT_FALSE(omniseer::vision::parse_vision_replay_args(static_cast<int>(argv.size()),
                                                          argv.data(), config, error,
                                                          help_requested));
  EXPECT_EQ(error, "--detector-model is required");
  EXPECT_FALSE(help_requested);
}

TEST(VisionReplayConfigTest, ParsesSupportedDetectorOptions)
{
  omniseer::vision::VisionReplayConfig config{};
  std::vector<std::string>             args{
      "vision_replay", "--video",           "input.mp4",   "--detector-model",
      "detector.rknn", "--classes",         "classes.txt", "--output",
      "output.jsonl",  "--clip-model",      "clip.rknn",   "--clip-vocab",
      "vocab.bpe",     "--score-threshold", "0.3",         "--nms-iou-threshold",
      "0.4",           "--max-detections",  "8",
  };
  std::vector<char*> argv = argv_from(args);
  std::string        error{};
  bool               help_requested = false;

  ASSERT_TRUE(omniseer::vision::parse_vision_replay_args(static_cast<int>(argv.size()), argv.data(),
                                                         config, error, help_requested));
  EXPECT_FALSE(help_requested);
  EXPECT_EQ(config.video_path, "input.mp4");
  EXPECT_FLOAT_EQ(config.score_threshold, 0.3F);
  EXPECT_FLOAT_EQ(config.nms_iou_threshold, 0.4F);
  EXPECT_EQ(config.max_detections, 8U);
}
