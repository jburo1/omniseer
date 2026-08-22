#include <cmath>
#include <cstdio>
#include <limits>
#include <opencv2/videoio.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "omniseer/vision/class_list.hpp"
#include "omniseer/vision/consumer_pipeline.hpp"
#include "omniseer/vision/cpu_letterbox_preprocess.hpp"
#include "omniseer/vision/detections_sink.hpp"
#include "omniseer/vision/dma_heap_alloc.hpp"
#include "omniseer/vision/image_buffer_pool.hpp"
#include "omniseer/vision/letterbox.hpp"
#include "omniseer/vision/replay_jsonl.hpp"
#include "omniseer/vision/rknn_runner.hpp"
#include "omniseer/vision/vision_replay_config.hpp"
#include "omniseer/vision/yolo_world_text_embeddings.hpp"

namespace
{
  constexpr omniseer::vision::Size kModelInputSize{640, 640};

  std::string source_path(const char* relpath)
  {
    return std::string(VISION_SOURCE_DIR) + relpath;
  }

  class ReplayDetectionsSink final : public omniseer::vision::IDetectionsSink
  {
  public:
    explicit ReplayDetectionsSink(omniseer::vision::ReplayJsonlWriter& writer) : _writer(writer) {}

    void set_frame_identity(uint64_t frame_index, double timestamp_sec) noexcept
    {
      _frame_index   = frame_index;
      _timestamp_sec = timestamp_sec;
      _pending       = true;
    }

    void publish(const omniseer::vision::DetectionsFrame& frame) noexcept override
    {
      if (!_pending || !_error.empty())
        return;
      try
      {
        _writer.write(_frame_index, _timestamp_sec, frame);
        _pending = false;
      }
      catch (const std::exception& e)
      {
        _error = e.what();
      }
    }

    void throw_if_failed() const
    {
      if (!_error.empty())
        throw std::runtime_error(_error);
      if (_pending)
        throw std::runtime_error("consumer did not publish replay detections");
    }

  private:
    omniseer::vision::ReplayJsonlWriter& _writer;
    uint64_t                             _frame_index{0};
    double                               _timestamp_sec{0.0};
    bool                                 _pending{false};
    std::string                          _error{};
  };

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

    const omniseer::vision::Size                source_size{source_w, source_h};
    const omniseer::vision::PipelineRemapConfig remap =
        omniseer::vision::make_letterbox_remap(source_size, kModelInputSize);

    const std::vector<std::string> class_names =
        omniseer::vision::load_class_list_file(config.class_list_path);
    omniseer::vision::YoloWorldTextEmbeddingsBuilder embeddings_builder({
        .text_encoder_model_path = config.clip_model_path,
        .detector_model_path     = config.detector_model_path,
        .clip_vocab_path         = config.clip_vocab_path,
        .pad_token               = config.pad_token,
    });
    const omniseer::vision::PreparedTextEmbeddings   embeddings =
        embeddings_builder.build(class_names);

    omniseer::vision::ImageBufferPool  pool{};
    omniseer::vision::DmaHeapAllocator allocator{};
    pool.allocate_all(allocator, kModelInputSize.w, kModelInputSize.h,
                      omniseer::vision::PixelFormat::RGB888);

    omniseer::vision::RknnRunner        runner({
               .model_path  = config.detector_model_path,
               .warmup_runs = config.warmup_runs,
    });
    omniseer::vision::ReplayJsonlWriter jsonl(config.output_path, embeddings.class_names);
    ReplayDetectionsSink                sink(jsonl);
    omniseer::vision::ConsumerPipeline  consumer(pool, runner, nullptr, &sink,
                                                 {
                                                     .score_threshold   = config.score_threshold,
                                                     .nms_iou_threshold = config.nms_iou_threshold,
                                                     .max_detections    = config.max_detections,
                                                });
    consumer.preflight({
        .remap           = remap,
        .text_embeddings = embeddings.view(),
    });

    cv::Mat  frame{};
    uint64_t frame_index = 0;
    while (video.read(frame))
    {
      if (frame_index > std::numeric_limits<uint32_t>::max())
        throw std::runtime_error("input video has more frames than the replay sequence supports");

      auto write_lease = pool.acquire_write_lease();
      if (!write_lease.has_value())
        throw std::runtime_error("no writable DMA-backed image buffer for replay frame");

      omniseer::vision::cpu_letterbox_bgr_to_rgb(frame, write_lease->buffer(), remap);
      write_lease->buffer().sequence           = static_cast<uint32_t>(frame_index);
      write_lease->buffer().capture_ts_real_ns = 0;
      write_lease->buffer().frame_id           = frame_index;
      write_lease->publish();

      sink.set_frame_identity(frame_index, replay_timestamp_sec(video, frame_index, fps));
      const omniseer::vision::ConsumerTick tick = consumer.run();
      if (tick.status != omniseer::vision::ConsumerTickStatus::Consumed)
      {
        throw std::runtime_error("consumer failed for replay frame " + std::to_string(frame_index) +
                                 ": stage=" + std::to_string(static_cast<unsigned>(tick.stage)) +
                                 " errno=" + std::to_string(tick.stage_errno));
      }
      sink.throw_if_failed();
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
