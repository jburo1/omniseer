#pragma once

#include <cstdint>
#include <string>

namespace omniseer::vision
{
  /** @brief Startup-only configuration for the sequential detector replay executable. */
  struct VisionReplayConfig
  {
    std::string video_path{};
    std::string detector_model_path{};
    std::string class_list_path{};
    std::string output_path{};
    std::string clip_model_path{};
    std::string clip_vocab_path{};
    std::string pad_token{"nothing"};
    uint32_t    warmup_runs{0};
    float       score_threshold{0.25F};
    float       nms_iou_threshold{0.45F};
    uint32_t    max_detections{100};
    bool        debug_rknn{false};
  };

  /**
   * @brief Parse replay CLI options without performing file or hardware I/O.
   *
   * @return true when arguments are valid. On false, @p error explains the problem.
   */
  bool parse_vision_replay_args(int argc, char** argv, VisionReplayConfig& config,
                                std::string& error, bool& help_requested);

  std::string vision_replay_usage(const char* argv0);
} // namespace omniseer::vision
