#include <cmath>
#include <cstdio>
#include <limits>
#include <opencv2/videoio.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "omniseer/vision/offline_detector.hpp"
#include "omniseer/vision/replay_jsonl.hpp"
#include "omniseer/vision/vision_replay_config.hpp"

namespace
{
  std::string source_path(const char* relpath)
  {
    return std::string(VISION_SOURCE_DIR) + relpath;
  }

  double replay_timestamp_sec(cv::VideoCapture& video, uint64_t frame_index, double fps)
  {
    const double timestamp_sec = video.get(cv::CAP_PROP_POS_MSEC) / 1000.0;
    if (std::isfinite(timestamp_sec) && timestamp_sec >= 0.0)
      return timestamp_sec;
    return (fps > 0.0 && std::isfinite(fps)) ? static_cast<double>(frame_index) / fps : 0.0;
  }
} // namespace

int main(int argc, char** argv)
{
  try
  {
    omniseer::vision::VisionReplayConfig config{};
    config.detector_model_path = source_path("/testdata/rknn_runner/yolo_world_v2s_i8.rknn");
    config.clip_model_path     = source_path("/testdata/text_embeddings/clip_text_fp16.rknn");
    config.clip_vocab_path     = source_path("/testdata/text_embeddings/clip_vocab.bpe");

    std::string parse_error{};
    bool        help_requested = false;
    if (!omniseer::vision::parse_vision_replay_args(argc, argv, config, parse_error,
                                                    help_requested))
    {
      std::fprintf(stderr, "vision_replay: %s\n%s", parse_error.c_str(),
                   omniseer::vision::vision_replay_usage(argv[0]).c_str());
      return 1;
    }
    if (help_requested)
    {
      std::fputs(omniseer::vision::vision_replay_usage(argv[0]).c_str(), stdout);
      return 0;
    }

    cv::VideoCapture video(config.video_path);
    if (!video.isOpened())
      throw std::runtime_error("failed to open input video: " + config.video_path);

    const int source_w = static_cast<int>(video.get(cv::CAP_PROP_FRAME_WIDTH));
    const int source_h = static_cast<int>(video.get(cv::CAP_PROP_FRAME_HEIGHT));
    if (source_w <= 0 || source_h <= 0)
      throw std::runtime_error("input video does not report a valid frame size");
    const double fps = video.get(cv::CAP_PROP_FPS);

    omniseer::vision::OfflineDetector   detector({
          .detector_model_path = config.detector_model_path,
          .class_list_path     = config.class_list_path,
          .clip_model_path     = config.clip_model_path,
          .clip_vocab_path     = config.clip_vocab_path,
          .pad_token           = config.pad_token,
          .warmup_runs         = config.warmup_runs,
          .score_threshold     = config.score_threshold,
          .nms_iou_threshold   = config.nms_iou_threshold,
          .max_detections      = config.max_detections,
          .source_width        = source_w,
          .source_height       = source_h,
          .debug_rknn          = config.debug_rknn,
    });
    omniseer::vision::ReplayJsonlWriter jsonl(config.output_path, detector.class_names());

    cv::Mat  frame{};
    uint64_t frame_index = 0;
    while (video.read(frame))
    {
      if (frame_index > std::numeric_limits<uint32_t>::max())
        throw std::runtime_error("input video has more frames than the replay sequence supports");

      const omniseer::vision::DetectionsFrame detections = detector.infer(frame, frame_index);
      jsonl.write(frame_index, replay_timestamp_sec(video, frame_index, fps), detections);
      ++frame_index;
    }

    std::fprintf(stdout, "vision_replay: wrote %llu frame records to %s\n",
                 static_cast<unsigned long long>(frame_index), config.output_path.c_str());
    return 0;
  }
  catch (const std::exception& e)
  {
    std::fprintf(stderr, "vision_replay: %s\n", e.what());
    return 1;
  }
}
