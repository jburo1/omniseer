#include "omniseer/vision/vision_replay_config.hpp"

#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>

namespace omniseer::vision
{
  namespace
  {
    bool parse_u32(const char* text, uint32_t& out)
    {
      if (text == nullptr || *text == '\0')
        return false;
      char*          end   = nullptr;
      const auto     value = std::strtoul(text, &end, 10);
      constexpr auto max   = std::numeric_limits<uint32_t>::max();
      if (end == nullptr || *end != '\0' || value > max)
        return false;
      out = static_cast<uint32_t>(value);
      return true;
    }

    bool parse_float(const char* text, float& out)
    {
      if (text == nullptr || *text == '\0')
        return false;
      char*       end   = nullptr;
      const float value = std::strtof(text, &end);
      if (end == nullptr || *end != '\0' || !std::isfinite(value))
        return false;
      out = value;
      return true;
    }
  } // namespace

  std::string vision_replay_usage(const char* argv0)
  {
    return "Usage: " + std::string(argv0) +
           " --video <mp4> --classes <path> --output <jsonl> [options]\n"
           "Options:\n"
           "  --detector-model <path>  YOLO-World RKNN path\n"
           "  --clip-model <path>      CLIP text encoder RKNN path\n"
           "  --clip-vocab <path>      CLIP BPE merges/vocab path\n"
           "  --pad-token <text>       Internal pad phrase for unused class slots\n"
           "  --warmup <u32>           RKNN warmup runs (default: 0)\n"
           "  --score-threshold <f>    Minimum detection confidence (default: 0.25)\n"
           "  --nms-iou-threshold <f>  Per-class NMS IoU threshold (default: 0.45)\n"
           "  --max-detections <u32>   Maximum detections per frame (default: 100)\n"
           "  --rknn-debug            Print tensor metadata and first-inference raw output "
           "statistics\n"
           "  --help                   Show this help\n";
  }

  bool parse_vision_replay_args(int argc, char** argv, VisionReplayConfig& config,
                                std::string& error, bool& help_requested)
  {
    help_requested = false;
    error.clear();

    try
    {
      for (int i = 1; i < argc; ++i)
      {
        const std::string arg           = argv[i];
        auto              require_value = [&](const char* name) -> const char*
        {
          if (i + 1 >= argc)
            throw std::runtime_error(std::string("missing value for ") + name);
          return argv[++i];
        };

        if (arg == "--help")
        {
          help_requested = true;
          return true;
        }
        if (arg == "--video")
          config.video_path = require_value("--video");
        else if (arg == "--detector-model")
          config.detector_model_path = require_value("--detector-model");
        else if (arg == "--classes")
          config.class_list_path = require_value("--classes");
        else if (arg == "--output")
          config.output_path = require_value("--output");
        else if (arg == "--clip-model")
          config.clip_model_path = require_value("--clip-model");
        else if (arg == "--clip-vocab")
          config.clip_vocab_path = require_value("--clip-vocab");
        else if (arg == "--pad-token")
          config.pad_token = require_value("--pad-token");
        else if (arg == "--warmup")
        {
          const char* value = require_value("--warmup");
          if (!parse_u32(value, config.warmup_runs))
            throw std::runtime_error("invalid integer for --warmup: " + std::string(value));
        }
        else if (arg == "--max-detections")
        {
          const char* value = require_value("--max-detections");
          if (!parse_u32(value, config.max_detections))
            throw std::runtime_error("invalid integer for --max-detections: " + std::string(value));
        }
        else if (arg == "--score-threshold")
        {
          const char* value = require_value("--score-threshold");
          if (!parse_float(value, config.score_threshold))
            throw std::runtime_error("invalid float for --score-threshold: " + std::string(value));
        }
        else if (arg == "--nms-iou-threshold")
        {
          const char* value = require_value("--nms-iou-threshold");
          if (!parse_float(value, config.nms_iou_threshold))
          {
            throw std::runtime_error("invalid float for --nms-iou-threshold: " +
                                     std::string(value));
          }
        }
        else if (arg == "--rknn-debug")
          config.debug_rknn = true;
        else
        {
          throw std::runtime_error("unknown argument: " + arg);
        }
      }

      if (config.video_path.empty())
        throw std::runtime_error("--video is required");
      if (config.detector_model_path.empty())
        throw std::runtime_error("--detector-model is required");
      if (config.class_list_path.empty())
        throw std::runtime_error("--classes is required");
      if (config.output_path.empty())
        throw std::runtime_error("--output is required");
      if (config.clip_model_path.empty())
        throw std::runtime_error("--clip-model is required");
      if (config.clip_vocab_path.empty())
        throw std::runtime_error("--clip-vocab is required");
      if (config.score_threshold < 0.0F || config.score_threshold > 1.0F)
        throw std::runtime_error("--score-threshold must be in [0, 1]");
      if (config.nms_iou_threshold < 0.0F || config.nms_iou_threshold > 1.0F)
        throw std::runtime_error("--nms-iou-threshold must be in [0, 1]");
      if (config.max_detections == 0)
        throw std::runtime_error("--max-detections must be > 0");
    }
    catch (const std::exception& e)
    {
      error = e.what();
      return false;
    }

    return true;
  }
} // namespace omniseer::vision
