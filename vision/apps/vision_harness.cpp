#include <algorithm>
#include <chrono>
#include <csignal>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <linux/videodev2.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "omniseer/vision/class_list.hpp"
#include "omniseer/vision/consumer_pipeline.hpp"
#include "omniseer/vision/detections_sink.hpp"
#include "omniseer/vision/dma_heap_alloc.hpp"
#include "omniseer/vision/frame_preview_sink.hpp"
#include "omniseer/vision/image_buffer_pool.hpp"
#include "omniseer/vision/producer_pipeline.hpp"
#include "omniseer/vision/rga_preprocess.hpp"
#include "omniseer/vision/rknn_runner.hpp"
#include "omniseer/vision/v4l2_capture.hpp"
#include "omniseer/vision/yolo_world_text_embeddings.hpp"

namespace
{
  volatile std::sig_atomic_t g_stop_requested = 0;

  void handle_signal(int) noexcept
  {
    g_stop_requested = 1;
  }

  std::string source_path(const char* relpath)
  {
    return std::string(VISION_SOURCE_DIR) + relpath;
  }

  struct HarnessConfig
  {
    std::string device{"/dev/video12"};
    uint32_t    width{1280};
    uint32_t    height{720};
    uint32_t    buffer_count{4};
    uint32_t    dst_width{640};
    uint32_t    dst_height{640};
    uint32_t    warmup_runs{0};
    uint32_t    preflight_capture_wait_ms{200};
    std::string clip_model_path{source_path("/testdata/text_embeddings/clip_text_fp16.rknn")};
    std::string clip_vocab_path{source_path("/testdata/text_embeddings/clip_vocab.bpe")};
    std::string detector_model_path{source_path("/testdata/rknn_runner/yolo_world_v2s_i8.rknn")};
    std::string class_list_path{};
    std::string pad_token{"nothing"};
    bool        preview{true};
  };

  void print_usage(const char* argv0)
  {
    std::fprintf(stderr,
                 "Usage: %s --classes <path> [options]\n"
                 "Options:\n"
                 "  --device <path>          V4L2 device (default: /dev/video12)\n"
                 "  --width <u32>            Capture width (default: 1280)\n"
                 "  --height <u32>           Capture height (default: 720)\n"
                 "  --buffers <u32>          V4L2 buffer count (default: 4)\n"
                 "  --dst-width <u32>        Model input width (default: 640)\n"
                 "  --dst-height <u32>       Model input height (default: 640)\n"
                 "  --warmup <u32>           RKNN warmup runs (default: 0)\n"
                 "  --clip-model <path>      CLIP text encoder RKNN path\n"
                 "  --clip-vocab <path>      CLIP BPE merges/vocab path\n"
                 "  --detector-model <path>  YOLO-World RKNN path\n"
                 "  --pad-token <text>       Internal pad phrase for unused class slots\n"
                 "  --no-preview             Disable the OpenCV preview window\n"
                 "  --help                   Show this help\n",
                 argv0);
  }

  bool parse_u32(const char* text, uint32_t& out)
  {
    if (text == nullptr || *text == '\0')
      return false;
    char*         end = nullptr;
    unsigned long n   = std::strtoul(text, &end, 10);
    if (end == nullptr || *end != '\0' || n > 0xffffffffUL)
      return false;
    out = static_cast<uint32_t>(n);
    return true;
  }

  bool parse_args(int argc, char** argv, HarnessConfig& cfg)
  {
    for (int i = 1; i < argc; ++i)
    {
      const std::string arg = argv[i];
      auto require_value = [&](const char* name) -> const char*
      {
        if (i + 1 >= argc)
          throw std::runtime_error(std::string("missing value for ") + name);
        return argv[++i];
      };

      if (arg == "--help")
      {
        print_usage(argv[0]);
        return false;
      }
      if (arg == "--classes")
      {
        cfg.class_list_path = require_value("--classes");
        continue;
      }
      if (arg == "--device")
      {
        cfg.device = require_value("--device");
        continue;
      }
      if (arg == "--clip-model")
      {
        cfg.clip_model_path = require_value("--clip-model");
        continue;
      }
      if (arg == "--clip-vocab")
      {
        cfg.clip_vocab_path = require_value("--clip-vocab");
        continue;
      }
      if (arg == "--detector-model")
      {
        cfg.detector_model_path = require_value("--detector-model");
        continue;
      }
      if (arg == "--pad-token")
      {
        cfg.pad_token = require_value("--pad-token");
        continue;
      }
      if (arg == "--no-preview")
      {
        cfg.preview = false;
        continue;
      }

      uint32_t* target = nullptr;
      if (arg == "--width")
        target = &cfg.width;
      else if (arg == "--height")
        target = &cfg.height;
      else if (arg == "--buffers")
        target = &cfg.buffer_count;
      else if (arg == "--dst-width")
        target = &cfg.dst_width;
      else if (arg == "--dst-height")
        target = &cfg.dst_height;
      else if (arg == "--warmup")
        target = &cfg.warmup_runs;

      if (target != nullptr)
      {
        const char* value = require_value(arg.c_str());
        if (!parse_u32(value, *target))
          throw std::runtime_error("invalid integer for " + arg + ": " + value);
        continue;
      }

      throw std::runtime_error("unknown argument: " + arg);
    }

    if (cfg.class_list_path.empty())
      throw std::runtime_error("--classes is required");

    return true;
  }

  void prefill_pool(omniseer::vision::ImageBufferPool& pool,
                    const omniseer::vision::RgaPreprocess& preprocess)
  {
    std::vector<omniseer::vision::ImageBufferPool::WriteLease> init_leases{};
    for (;;)
    {
      auto lease = pool.acquire_write_lease();
      if (!lease.has_value())
        break;
      preprocess.prefill(lease->buffer());
      init_leases.emplace_back(std::move(*lease));
    }
    if (init_leases.empty())
      throw std::runtime_error("failed to prefill image buffer pool");
  }

  class ConsoleDetectionsSink final : public omniseer::vision::IDetectionsSink
  {
  public:
    explicit ConsoleDetectionsSink(std::vector<std::string> class_names)
        : _class_names(std::move(class_names))
    {
    }

    void publish(const omniseer::vision::DetectionsFrame& frame) noexcept override
    {
      if (frame.count == 0)
        return;

      std::fprintf(stdout, "frame=%llu seq=%llu detections=%u\n",
                   static_cast<unsigned long long>(frame.frame_id),
                   static_cast<unsigned long long>(frame.sequence), frame.count);
      for (uint32_t i = 0; i < frame.count; ++i)
      {
        const auto& det   = frame.detections[i];
        const char* label = "<out-of-range>";
        if (det.class_id < _class_names.size())
          label = _class_names[det.class_id].c_str();
        std::fprintf(stdout, "  %s score=%.3f box=[%.1f %.1f %.1f %.1f]\n", label, det.score,
                     det.x1, det.y1, det.x2, det.y2);
      }
      std::fflush(stdout);
    }

  private:
    std::vector<std::string> _class_names{};
  };

  class OpenCvPreviewSink final : public omniseer::vision::IFramePreviewSink
  {
  public:
    explicit OpenCvPreviewSink(std::vector<std::string> class_names,
                               std::string              window_name = "Omniseer Vision")
        : _class_names(std::move(class_names)), _window_name(std::move(window_name))
    {
      cv::namedWindow(_window_name, cv::WINDOW_NORMAL);
    }

    ~OpenCvPreviewSink() override
    {
      for (auto& [fd, mapping] : _mappings)
      {
        if (mapping.addr != nullptr && mapping.length != 0)
          ::munmap(mapping.addr, mapping.length);
      }
      cv::destroyWindow(_window_name);
    }

    void publish(const omniseer::vision::ImageBuffer&        image,
                 const omniseer::vision::DetectionsFrame&    frame,
                 const omniseer::vision::PipelineRemapConfig& remap) noexcept override
    {
      if (image.fmt != omniseer::vision::PixelFormat::RGB888 || image.num_planes == 0)
        return;
      const auto& plane = image.planes[0];
      if (plane.fd < 0 || plane.stride == 0)
        return;

      Mapping* mapping = find_or_create_mapping(image);
      if (mapping == nullptr)
        return;

      auto* base = static_cast<uint8_t*>(mapping->addr);
      cv::Mat rgb(image.size.h, image.size.w, CV_8UC3, base + plane.offset, plane.stride);
      cv::Mat bgr{};
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);

      draw_overlay(bgr, frame, remap);
      cv::imshow(_window_name, bgr);
      const int key = cv::waitKey(1);
      if (key == 27 || key == 'q' || key == 'Q')
        g_stop_requested = 1;
    }

  private:
    struct Mapping
    {
      void*  addr{nullptr};
      size_t length{0};
    };

    Mapping* find_or_create_mapping(const omniseer::vision::ImageBuffer& image) noexcept
    {
      const int fd = image.planes[0].fd;
      auto      it = _mappings.find(fd);
      if (it != _mappings.end())
        return &it->second;

      const size_t map_length =
          (image.total_alloc_size != 0) ? image.total_alloc_size
                                        : (image.planes[0].offset + image.planes[0].alloc_size);
      if (map_length == 0)
        return nullptr;

      void* addr = ::mmap(nullptr, map_length, PROT_READ, MAP_SHARED, fd, 0);
      if (addr == MAP_FAILED)
      {
        if (!_logged_map_failure)
        {
          std::fprintf(stderr, "vision_harness: preview mmap failed: %d\n", errno);
          _logged_map_failure = true;
        }
        return nullptr;
      }

      auto [inserted, ok] = _mappings.emplace(fd, Mapping{.addr = addr, .length = map_length});
      return ok ? &inserted->second : nullptr;
    }

    void draw_overlay(cv::Mat&                                  image,
                      const omniseer::vision::DetectionsFrame&  frame,
                      const omniseer::vision::PipelineRemapConfig& remap) noexcept
    {
      cv::putText(image,
                  "frame=" + std::to_string(frame.frame_id) + " det=" + std::to_string(frame.count),
                  cv::Point(12, 24), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(240, 240, 240), 2,
                  cv::LINE_AA);

      for (uint32_t i = 0; i < frame.count; ++i)
      {
        const auto& det = frame.detections[i];
        const float x1f = det.x1 * remap.scale + static_cast<float>(remap.pad_x);
        const float y1f = det.y1 * remap.scale + static_cast<float>(remap.pad_y);
        const float x2f = det.x2 * remap.scale + static_cast<float>(remap.pad_x);
        const float y2f = det.y2 * remap.scale + static_cast<float>(remap.pad_y);

        const int x1 = std::clamp(static_cast<int>(x1f), 0, image.cols - 1);
        const int y1 = std::clamp(static_cast<int>(y1f), 0, image.rows - 1);
        const int x2 = std::clamp(static_cast<int>(x2f), 0, image.cols - 1);
        const int y2 = std::clamp(static_cast<int>(y2f), 0, image.rows - 1);
        if (x2 <= x1 || y2 <= y1)
          continue;

        cv::rectangle(image, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)),
                      cv::Scalar(64, 220, 64), 2);

        const char* label = "<out-of-range>";
        if (det.class_id < _class_names.size())
          label = _class_names[det.class_id].c_str();
        const std::string text =
            std::string(label) + " " + cv::format("%.2f", static_cast<double>(det.score));
        int               baseline = 0;
        const cv::Size    size =
            cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        const int text_y = std::max(y1, size.height + 4);
        const int box_y  = std::max(0, text_y - size.height - baseline - 4);
        const int box_x2 = std::min(x1 + size.width + 6, image.cols);
        cv::rectangle(image, cv::Point(x1, box_y), cv::Point(box_x2, text_y + baseline),
                      cv::Scalar(64, 220, 64), cv::FILLED);
        cv::putText(image, text, cv::Point(x1 + 3, text_y - 2), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(16, 16, 16), 1, cv::LINE_AA);
      }
    }

    std::vector<std::string>        _class_names{};
    std::string                     _window_name{};
    std::unordered_map<int, Mapping> _mappings{};
    bool                            _logged_map_failure{false};
  };
} // namespace

int main(int argc, char** argv)
{
  try
  {
    HarnessConfig cfg{};
    if (!parse_args(argc, argv, cfg))
      return 0;

    const std::vector<std::string> class_names =
        omniseer::vision::load_class_list_file(cfg.class_list_path);
    std::fprintf(stdout, "Loaded %zu class names from %s\n", class_names.size(),
                 cfg.class_list_path.c_str());
    std::fprintf(stdout,
                 "Unused detector text slots will be padded internally with \"%s\" and ignored by postprocess.\n",
                 cfg.pad_token.c_str());
    std::fprintf(stdout, "Preview: %s\n", cfg.preview ? "enabled" : "disabled");

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    omniseer::vision::V4l2Capture capture({
        .device       = cfg.device,
        .width        = cfg.width,
        .height       = cfg.height,
        .fourcc       = V4L2_PIX_FMT_NV12,
        .buffer_count = cfg.buffer_count,
    });
    capture.start();

    omniseer::vision::ImageBufferPool pool{};
    omniseer::vision::DmaHeapAllocator allocator{};
    pool.allocate_all(allocator, static_cast<int>(cfg.dst_width), static_cast<int>(cfg.dst_height),
                      omniseer::vision::PixelFormat::RGB888);

    omniseer::vision::RgaPreprocess preprocess({
        .src_w     = static_cast<int>(cfg.width),
        .src_h     = static_cast<int>(cfg.height),
        .dst_w     = static_cast<int>(cfg.dst_width),
        .dst_h     = static_cast<int>(cfg.dst_height),
        .pad_value = 114,
    });
    prefill_pool(pool, preprocess);

    omniseer::vision::ProducerPipeline producer(
        capture, preprocess, pool, nullptr,
        {.preflight_capture_wait_ms = cfg.preflight_capture_wait_ms});
    producer.preflight();

    omniseer::vision::YoloWorldTextEmbeddingsBuilder builder({
        .text_encoder_model_path = cfg.clip_model_path,
        .detector_model_path     = cfg.detector_model_path,
        .clip_vocab_path         = cfg.clip_vocab_path,
        .pad_token               = cfg.pad_token,
    });
    const omniseer::vision::PreparedTextEmbeddings prepared = builder.build(class_names);
    ConsoleDetectionsSink sink(prepared.class_names);

    omniseer::vision::RknnRunner runner({
        .model_path  = cfg.detector_model_path,
        .warmup_runs = cfg.warmup_runs,
    });
    omniseer::vision::ConsumerPipeline consumer(pool, runner, nullptr, &sink);
    std::unique_ptr<OpenCvPreviewSink> preview_sink{};
    if (cfg.preview)
    {
      preview_sink = std::make_unique<OpenCvPreviewSink>(prepared.class_names);
      consumer.set_preview_sink(preview_sink.get());
    }
    consumer.preflight({
        .remap           = producer.remap(),
        .text_embeddings = prepared.view(),
    });

    std::fprintf(stdout, "Pipeline armed. Press Ctrl-C to stop.\n");
    if (cfg.preview)
      std::fprintf(stdout, "Press 'q' or Esc in the preview window to stop.\n");
    std::fflush(stdout);

    while (g_stop_requested == 0)
    {
      const auto producer_tick = producer.run();
      switch (producer_tick.status)
      {
      case omniseer::vision::ProducerTickStatus::Produced:
        break;
      case omniseer::vision::ProducerTickStatus::NoFrame:
      case omniseer::vision::ProducerTickStatus::CaptureRetryableError:
      case omniseer::vision::ProducerTickStatus::NoWritableBuffer:
        break;
      case omniseer::vision::ProducerTickStatus::CaptureFatalError:
      case omniseer::vision::ProducerTickStatus::PreprocessError:
        std::fprintf(stderr, "producer stopped: status=%u stage=%u errno=%d\n",
                     static_cast<unsigned>(producer_tick.status),
                     static_cast<unsigned>(producer_tick.stage), producer_tick.stage_errno);
        g_stop_requested = 1;
        continue;
      }

      const auto consumer_tick = consumer.run();
      switch (consumer_tick.status)
      {
      case omniseer::vision::ConsumerTickStatus::Consumed:
        break;
      case omniseer::vision::ConsumerTickStatus::NoReadyBuffer:
        break;
      case omniseer::vision::ConsumerTickStatus::InferError:
        std::fprintf(stderr, "consumer stopped: stage=%u errno=%d\n",
                     static_cast<unsigned>(consumer_tick.stage), consumer_tick.stage_errno);
        g_stop_requested = 1;
        continue;
      }

      if (producer_tick.status != omniseer::vision::ProducerTickStatus::Produced &&
          consumer_tick.status != omniseer::vision::ConsumerTickStatus::Consumed)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }

    capture.stop();
    return 0;
  }
  catch (const std::exception& e)
  {
    std::fprintf(stderr, "vision_harness: %s\n", e.what());
    return 1;
  }
}
