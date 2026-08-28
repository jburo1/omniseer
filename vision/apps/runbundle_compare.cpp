#include <array>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "omniseer/vision/offline_detector.hpp"
#include "omniseer/vision/preview_wrap_repair.hpp"
#include "omniseer/vision/runbundle_comparison.hpp"

namespace
{
  namespace fs = std::filesystem;

  struct Config
  {
    fs::path    run_dir{};
    fs::path    model_dir{};
    fs::path    class_list_path{};
    fs::path    clip_model_path{};
    fs::path    clip_vocab_path{};
    std::string pad_token{"nothing"};
    uint32_t    warmup_runs{0};
    float       score_threshold{0.25F};
    float       nms_iou_threshold{0.45F};
    uint32_t    max_detections{100};
    uint32_t    max_frames{0};
  };

  std::string source_path(const char* relpath)
  {
    return std::string(VISION_SOURCE_DIR) + relpath;
  }

  std::string usage(const char* argv0)
  {
    return "Usage: " + std::string(argv0) +
           " <run_dir> --model-dir <dir> --classes <path> [options]\n"
           "\n"
           "Directly decodes <run_dir>/video/source.ts, reverses the validated 1280x720\n"
           "Rockchip 8-pixel preview wrap in memory, and runs the same corrected BGR\n"
           "frame sequentially through v2-S FP, v2-S INT8, v2-M FP, and v2-M INT8.\n"
           "It writes video/comparison/comparison.mp4 and provenance.json.\n"
           "\n"
           "Options:\n"
           "  --model-dir <dir>        Directory containing the four YOLO-World RKNN artifacts\n"
           "  --classes <path>         Class list, using the normal vision_replay convention\n"
           "  --clip-model <path>      CLIP text encoder RKNN path\n"
           "  --clip-vocab <path>      CLIP BPE merges/vocab path\n"
           "  --pad-token <text>       Internal pad phrase for unused class slots\n"
           "  --warmup <u32>           RKNN warmup runs per model (default: 0)\n"
           "  --score-threshold <f>    Minimum detection confidence (default: 0.25)\n"
           "  --nms-iou-threshold <f>  Per-class NMS IoU threshold (default: 0.45)\n"
           "  --max-detections <u32>   Maximum detections per model/frame (default: 100)\n"
           "  --max-frames <u32>       Stop after this many source frames (default: all)\n"
           "  --help                   Show this help\n";
  }

  bool parse_u32(const char* text, uint32_t& out)
  {
    if (text == nullptr || *text == '\0')
      return false;
    char*              end   = nullptr;
    const unsigned long value = std::strtoul(text, &end, 10);
    if (end == nullptr || *end != '\0' || value > UINT32_MAX)
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

  Config parse_args(int argc, char** argv, bool& help_requested)
  {
    help_requested = false;
    Config config{};
    for (int i = 1; i < argc; ++i)
    {
      const std::string arg = argv[i];
      const auto require_value = [&](const char* option) -> const char*
      {
        if (i + 1 >= argc)
          throw std::runtime_error(std::string("missing value for ") + option);
        return argv[++i];
      };
      if (arg == "--help")
      {
        help_requested = true;
        return config;
      }
      if (arg == "--model-dir")
        config.model_dir = require_value("--model-dir");
      else if (arg == "--classes")
        config.class_list_path = require_value("--classes");
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
      else if (arg == "--max-frames")
      {
        const char* value = require_value("--max-frames");
        if (!parse_u32(value, config.max_frames))
          throw std::runtime_error("invalid integer for --max-frames: " + std::string(value));
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
          throw std::runtime_error("invalid float for --nms-iou-threshold: " + std::string(value));
      }
      else if (!arg.empty() && arg[0] == '-')
        throw std::runtime_error("unknown argument: " + arg);
      else if (config.run_dir.empty())
        config.run_dir = arg;
      else
        throw std::runtime_error("only one <run_dir> may be supplied");
    }

    if (config.run_dir.empty())
      throw std::runtime_error("<run_dir> is required");
    if (config.model_dir.empty())
      throw std::runtime_error("--model-dir is required");
    if (config.class_list_path.empty())
      throw std::runtime_error("--classes is required");
    if (config.score_threshold < 0.0F || config.score_threshold > 1.0F)
      throw std::runtime_error("--score-threshold must be in [0, 1]");
    if (config.nms_iou_threshold < 0.0F || config.nms_iou_threshold > 1.0F)
      throw std::runtime_error("--nms-iou-threshold must be in [0, 1]");
    if (config.max_detections == 0)
      throw std::runtime_error("--max-detections must be > 0");
    if (config.clip_model_path.empty())
      config.clip_model_path = source_path("/testdata/text_embeddings/clip_text_fp16.rknn");
    if (config.clip_vocab_path.empty())
      config.clip_vocab_path = source_path("/testdata/text_embeddings/clip_vocab.bpe");
    return config;
  }

  void require_regular_file(const fs::path& path, const char* description)
  {
    if (!fs::is_regular_file(path))
      throw std::runtime_error(std::string(description) + " is not a regular file: " + path.string());
  }

  std::string shell_quote(const std::string& value)
  {
    std::string quoted{"'"};
    for (const char ch : value)
    {
      if (ch == '\'')
        quoted += "'\\\"'\\\"'";
      else
        quoted += ch;
    }
    quoted += '\'';
    return quoted;
  }

  std::string sha256_file(const fs::path& path)
  {
    const std::string command = "sha256sum -- " + shell_quote(path.string());
    FILE*             pipe    = ::popen(command.c_str(), "r");
    if (pipe == nullptr)
      throw std::runtime_error("failed to start sha256sum for: " + path.string());
    std::array<char, 256> output{};
    const char*           line = std::fgets(output.data(), static_cast<int>(output.size()), pipe);
    const int             status = ::pclose(pipe);
    if (line == nullptr || status != 0)
      throw std::runtime_error("sha256sum failed for: " + path.string());
    const std::string result(output.data());
    if (result.size() < 64)
      throw std::runtime_error("sha256sum returned invalid output for: " + path.string());
    return result.substr(0, 64);
  }

  void draw_detections(cv::Mat& image, const omniseer::vision::DetectionsFrame& detections,
                       const std::vector<std::string>& class_names, const std::string& label)
  {
    cv::rectangle(image, cv::Rect(0, 0, image.cols, 34), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(image, label, cv::Point(12, 24), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    for (uint32_t i = 0; i < detections.count; ++i)
    {
      const auto& detection = detections.detections[i];
      const int   x1 = std::clamp(static_cast<int>(std::lround(detection.x1)), 0, image.cols - 1);
      const int   y1 = std::clamp(static_cast<int>(std::lround(detection.y1)), 0, image.rows - 1);
      const int   x2 = std::clamp(static_cast<int>(std::lround(detection.x2)), 0, image.cols - 1);
      const int   y2 = std::clamp(static_cast<int>(std::lround(detection.y2)), 0, image.rows - 1);
      cv::rectangle(image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2,
                    cv::LINE_AA);
      const std::string class_name = (detection.class_id < class_names.size())
                                         ? class_names[detection.class_id]
                                         : "<out-of-range>";
      const std::string text = class_name + " " + cv::format("%.2f", detection.score);
      const int text_y = std::max(48, y1 - 5);
      cv::putText(image, text, cv::Point(x1, text_y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
  }

  cv::Mat compose_quadrants(const cv::Mat& corrected_bgr,
                            const std::array<omniseer::vision::DetectionsFrame, 4>& detections,
                            const std::vector<std::string>& class_names)
  {
    std::array<cv::Mat, 4> quadrants{};
    for (size_t i = 0; i < quadrants.size(); ++i)
    {
      cv::Mat annotated = corrected_bgr.clone();
      draw_detections(annotated, detections[i], class_names,
                      omniseer::vision::kRunbundleComparisonModels[i].label);
      cv::resize(annotated, quadrants[i], cv::Size(640, 360), 0.0, 0.0, cv::INTER_AREA);
    }
    cv::Mat top{};
    cv::Mat bottom{};
    cv::Mat output{};
    cv::hconcat(quadrants[0], quadrants[1], top);
    cv::hconcat(quadrants[2], quadrants[3], bottom);
    cv::vconcat(top, bottom, output);
    return output;
  }

  void encode_browser_mp4(const fs::path& intermediate, const fs::path& output)
  {
    const std::string command =
        "ffmpeg -y -loglevel error -i " + shell_quote(intermediate.string()) +
        " -map 0:v:0 -an -c:v libx264 -preset ultrafast -crf 18 -pix_fmt yuv420p "
        "-movflags +faststart " + shell_quote(output.string());
    if (std::system(command.c_str()) != 0)
      throw std::runtime_error("ffmpeg failed to encode browser-compatible comparison MP4");
  }
} // namespace

int main(int argc, char** argv)
{
  try
  {
    bool         help_requested = false;
    const Config config = parse_args(argc, argv, help_requested);
    if (help_requested)
    {
      std::fputs(usage(argv[0]).c_str(), stdout);
      return 0;
    }

    const fs::path source_ts = config.run_dir / "video" / "source.ts";
    require_regular_file(source_ts, "raw RunBundle source.ts");
    require_regular_file(config.class_list_path, "class list");
    require_regular_file(config.clip_model_path, "CLIP model");
    require_regular_file(config.clip_vocab_path, "CLIP vocabulary");
    const std::string source_sha256_before = sha256_file(source_ts);

    const fs::path comparison_dir = config.run_dir / "video" / "comparison";
    fs::create_directories(comparison_dir);
    const fs::path output_mp4 = comparison_dir / "comparison.mp4";
    const fs::path intermediate_mp4 = comparison_dir / "comparison.rendering.mp4";
    const fs::path provenance_path = comparison_dir / "provenance.json";

    std::array<fs::path, 4> model_paths{};
    std::array<std::unique_ptr<omniseer::vision::OfflineDetector>, 4> detectors{};
    for (size_t i = 0; i < detectors.size(); ++i)
    {
      const auto& model = omniseer::vision::kRunbundleComparisonModels[i];
      model_paths[i] = config.model_dir / model.artifact_name;
      require_regular_file(model_paths[i], (std::string(model.label) + " model artifact").c_str());
      try
      {
        detectors[i] = std::make_unique<omniseer::vision::OfflineDetector>(
            omniseer::vision::OfflineDetectorConfig{
                .detector_model_path = model_paths[i].string(),
                .class_list_path     = config.class_list_path.string(),
                .clip_model_path     = config.clip_model_path.string(),
                .clip_vocab_path     = config.clip_vocab_path.string(),
                .pad_token           = config.pad_token,
                .warmup_runs         = config.warmup_runs,
                .score_threshold     = config.score_threshold,
                .nms_iou_threshold   = config.nms_iou_threshold,
                .max_detections      = config.max_detections,
                .source_width        = omniseer::vision::kRockchipPreviewWrapRepairWidth,
                .source_height       = omniseer::vision::kRockchipPreviewWrapRepairHeight,
            });
      }
      catch (const std::exception& e)
      {
        throw std::runtime_error(std::string("failed to initialize resident ") + model.label +
                                 " RKNN context: " + e.what());
      }
      if (i != 0 && detectors[i]->class_names() != detectors[0]->class_names())
        throw std::runtime_error("model contexts prepared different class lists");
    }

    cv::VideoCapture video(source_ts.string());
    if (!video.isOpened())
      throw std::runtime_error("failed to decode raw RunBundle source.ts: " + source_ts.string());
    const int source_width = static_cast<int>(video.get(cv::CAP_PROP_FRAME_WIDTH));
    const int source_height = static_cast<int>(video.get(cv::CAP_PROP_FRAME_HEIGHT));
    if (source_width != omniseer::vision::kRockchipPreviewWrapRepairWidth ||
        source_height != omniseer::vision::kRockchipPreviewWrapRepairHeight)
    {
      throw std::runtime_error("comparison accepts only validated 1280x720 raw preview recordings; got " +
                               std::to_string(source_width) + "x" + std::to_string(source_height));
    }
    const double output_fps = omniseer::vision::comparison_output_fps(video.get(cv::CAP_PROP_FPS));
    cv::VideoWriter writer(intermediate_mp4.string(), cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                           output_fps, cv::Size(source_width, source_height));
    if (!writer.isOpened())
      throw std::runtime_error("failed to open temporary comparison video for writing");

    uint64_t frame_index = 0;
    cv::Mat  frame{};
    while ((config.max_frames == 0 || frame_index < config.max_frames) && video.read(frame))
    {
      std::string repair_error{};
      if (frame.type() != CV_8UC3 || !omniseer::vision::repair_rockchip_preview_wrap_bgr(
                                      frame.data, frame.step, frame.cols, frame.rows, repair_error))
      {
        throw std::runtime_error("failed to repair decoded raw frame: " + repair_error);
      }

      std::array<omniseer::vision::DetectionsFrame, 4> detections{};
      // Calls are deliberately serial and all receive this exact corrected cv::Mat.
      omniseer::vision::visit_comparison_models(
          frame, [&](size_t i, const omniseer::vision::ComparisonModelSpec&, const cv::Mat& shared_frame)
          { detections[i] = detectors[i]->infer(shared_frame, frame_index); });
      writer.write(compose_quadrants(frame, detections, detectors[0]->class_names()));
      ++frame_index;
    }
    writer.release();
    if (frame_index == 0)
      throw std::runtime_error("raw RunBundle source.ts contained no decoded frames");

    encode_browser_mp4(intermediate_mp4, output_mp4);
    std::error_code remove_error{};
    fs::remove(intermediate_mp4, remove_error);
    if (remove_error)
      throw std::runtime_error("failed to remove temporary comparison rendering: " + remove_error.message());
    if (sha256_file(source_ts) != source_sha256_before)
      throw std::runtime_error("raw RunBundle source.ts changed while comparison was running");

    omniseer::vision::ComparisonProvenance provenance{};
    provenance.source_path      = "video/source.ts";
    provenance.source_sha256    = source_sha256_before;
    provenance.classes          = detectors[0]->class_names();
    provenance.score_threshold  = config.score_threshold;
    provenance.nms_iou_threshold = config.nms_iou_threshold;
    provenance.max_detections    = config.max_detections;
    provenance.output_path      = "video/comparison/comparison.mp4";
    provenance.output_sha256    = sha256_file(output_mp4);
    provenance.output_fps       = output_fps;
    provenance.source_frames_rendered = frame_index;
    if (const char* git_sha = std::getenv("OMNISEER_GIT_SHA"); git_sha != nullptr && *git_sha != '\0')
      provenance.git_sha = git_sha;
    for (size_t i = 0; i < model_paths.size(); ++i)
    {
      provenance.models.push_back({
          .label         = omniseer::vision::kRunbundleComparisonModels[i].label,
          .artifact_name = omniseer::vision::kRunbundleComparisonModels[i].artifact_name,
          .path          = model_paths[i].string(),
          .sha256        = sha256_file(model_paths[i]),
      });
    }
    std::ofstream provenance_output(provenance_path, std::ios::out | std::ios::trunc);
    if (!provenance_output)
      throw std::runtime_error("failed to open comparison provenance output");
    provenance_output << omniseer::vision::comparison_provenance_json(provenance);
    if (!provenance_output)
      throw std::runtime_error("failed to write comparison provenance output");

    std::fprintf(stdout, "vision_runbundle_compare: wrote %llu corrected source frames to %s\n",
                 static_cast<unsigned long long>(frame_index), output_mp4.c_str());
    return 0;
  }
  catch (const std::exception& e)
  {
    std::fprintf(stderr, "vision_runbundle_compare: %s\n", e.what());
    return 1;
  }
}
