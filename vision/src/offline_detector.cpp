#include "omniseer/vision/offline_detector.hpp"

#include <stdexcept>
#include <utility>

#include "omniseer/vision/class_list.hpp"
#include "omniseer/vision/consumer_pipeline.hpp"
#include "omniseer/vision/cpu_letterbox_preprocess.hpp"
#include "omniseer/vision/detections_sink.hpp"
#include "omniseer/vision/dma_heap_alloc.hpp"
#include "omniseer/vision/image_buffer_pool.hpp"
#include "omniseer/vision/letterbox.hpp"
#include "omniseer/vision/rknn_runner.hpp"
#include "omniseer/vision/yolo_world_text_embeddings.hpp"

namespace omniseer::vision
{
  namespace
  {
    constexpr Size kModelInputSize{640, 640};

    class CapturingDetectionsSink final : public IDetectionsSink
    {
    public:
      void reset() noexcept
      {
        _published = false;
      }

      void publish(const DetectionsFrame& frame) noexcept override
      {
        _frame     = frame;
        _published = true;
      }

      DetectionsFrame take()
      {
        if (!_published)
          throw std::runtime_error("offline detector did not publish detections");
        return _frame;
      }

    private:
      DetectionsFrame _frame{};
      bool            _published{false};
    };
  } // namespace

  struct OfflineDetector::Impl
  {
    explicit Impl(OfflineDetectorConfig config)
        : _config(std::move(config)),
          _remap(
              make_letterbox_remap({_config.source_width, _config.source_height}, kModelInputSize)),
          _class_names(load_class_list_file(_config.class_list_path)),
          _embeddings(
              YoloWorldTextEmbeddingsBuilder({
                                                 .text_encoder_model_path = _config.clip_model_path,
                                                 .detector_model_path = _config.detector_model_path,
                                                 .clip_vocab_path     = _config.clip_vocab_path,
                                                 .pad_token           = _config.pad_token,
                                             })
                  .build(_class_names)),
          _runner({.model_path = _config.detector_model_path, .warmup_runs = _config.warmup_runs}),
          _consumer(_pool, _runner, nullptr, &_sink,
                    {
                        .score_threshold   = _config.score_threshold,
                        .nms_iou_threshold = _config.nms_iou_threshold,
                        .max_detections    = _config.max_detections,
                    })
    {
      _pool.allocate_all(_allocator, kModelInputSize.w, kModelInputSize.h, PixelFormat::RGB888);
      _consumer.preflight({.remap = _remap, .text_embeddings = _embeddings.view()});
    }

    DetectionsFrame infer(const cv::Mat& corrected_bgr_frame, uint64_t frame_index)
    {
      if (corrected_bgr_frame.empty() || corrected_bgr_frame.type() != CV_8UC3 ||
          corrected_bgr_frame.cols != _config.source_width ||
          corrected_bgr_frame.rows != _config.source_height)
      {
        throw std::invalid_argument(
            "offline detector source frame does not match configured BGR geometry");
      }
      if (frame_index > UINT32_MAX)
        throw std::runtime_error("offline detector frame sequence exceeds uint32_t");

      auto write_lease = _pool.acquire_write_lease();
      if (!write_lease.has_value())
        throw std::runtime_error("no writable DMA-backed image buffer for offline detector");

      cpu_letterbox_bgr_to_rgb(corrected_bgr_frame, write_lease->buffer(), _remap);
      write_lease->buffer().sequence           = static_cast<uint32_t>(frame_index);
      write_lease->buffer().capture_ts_real_ns = 0;
      write_lease->buffer().frame_id           = frame_index;
      write_lease->publish();

      _sink.reset();
      const ConsumerTick tick = _consumer.run();
      if (tick.status != ConsumerTickStatus::Consumed)
      {
        throw std::runtime_error("offline detector consumer failed: stage=" +
                                 std::to_string(static_cast<unsigned>(tick.stage)) +
                                 " errno=" + std::to_string(tick.stage_errno));
      }
      return _sink.take();
    }

    OfflineDetectorConfig    _config{};
    PipelineRemapConfig      _remap{};
    std::vector<std::string> _class_names{};
    PreparedTextEmbeddings   _embeddings{};
    ImageBufferPool          _pool{};
    DmaHeapAllocator         _allocator{};
    RknnRunner               _runner;
    CapturingDetectionsSink  _sink{};
    ConsumerPipeline         _consumer;
  };

  OfflineDetector::OfflineDetector(OfflineDetectorConfig config)
      : _impl(std::make_unique<Impl>(std::move(config)))
  {
  }

  OfflineDetector::~OfflineDetector() = default;

  const std::vector<std::string>& OfflineDetector::class_names() const noexcept
  {
    return _impl->_embeddings.class_names;
  }

  DetectionsFrame OfflineDetector::infer(const cv::Mat& corrected_bgr_frame, uint64_t frame_index)
  {
    return _impl->infer(corrected_bgr_frame, frame_index);
  }
} // namespace omniseer::vision
