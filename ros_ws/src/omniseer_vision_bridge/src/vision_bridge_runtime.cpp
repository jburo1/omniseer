#include "omniseer_vision_bridge/vision_bridge_runtime.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <linux/videodev2.h>

#include "omniseer/vision/class_list.hpp"
#include "omniseer/vision/consumer_pipeline.hpp"
#include "omniseer/vision/detections_sink.hpp"
#include "omniseer/vision/dma_heap_alloc.hpp"
#include "omniseer/vision/image_buffer_pool.hpp"
#include "omniseer/vision/producer_pipeline.hpp"
#include "omniseer/vision/rga_preprocess.hpp"
#include "omniseer/vision/rknn_runner.hpp"
#include "omniseer/vision/rolling_telemetry.hpp"
#include "omniseer/vision/v4l2_capture.hpp"
#include "omniseer/vision/yolo_world_text_embeddings.hpp"

namespace omniseer_vision_bridge
{
namespace
{
template<typename T>
uint32_t checked_u32(T value, const char * name)
{
  if (value <= 0) {
    throw std::invalid_argument(std::string(name) + " must be > 0");
  }
  if (value > static_cast<T>(0xffffffffu)) {
    throw std::invalid_argument(std::string(name) + " exceeds uint32_t");
  }
  return static_cast<uint32_t>(value);
}

template<typename T>
uint32_t checked_non_negative_u32(T value, const char * name)
{
  if (value < 0) {
    throw std::invalid_argument(std::string(name) + " must be >= 0");
  }
  if (value > static_cast<T>(0xffffffffu)) {
    throw std::invalid_argument(std::string(name) + " exceeds uint32_t");
  }
  return static_cast<uint32_t>(value);
}

float checked_unit_float(double value, const char * name)
{
  if (value < 0.0 || value > 1.0) {
    throw std::invalid_argument(std::string(name) + " must be in [0, 1]");
  }
  return static_cast<float>(value);
}

void prefill_pool(
  omniseer::vision::ImageBufferPool & pool,
  const omniseer::vision::RgaPreprocess & preprocess)
{
  std::vector<omniseer::vision::ImageBufferPool::WriteLease> init_leases{};
  for (;; ) {
    auto lease = pool.acquire_write_lease();
    if (!lease.has_value()) {
      break;
    }
    preprocess.prefill(lease->buffer());
    init_leases.emplace_back(std::move(*lease));
  }
  if (init_leases.empty()) {
    throw std::runtime_error("VisionBridgeRuntime: failed to prefill image buffer pool");
  }
}
}   // namespace

struct VisionBridgeRuntime::Impl
{
  explicit Impl(VisionBridgeRuntimeConfig config_in)
  : config(std::move(config_in))
  {
  }

  void start()
  {
    if (running.load(std::memory_order_acquire)) {
      return;
    }

    validate_config();
    stop_requested.store(false, std::memory_order_release);
    fatal_error.store(false, std::memory_order_release);
    {
      std::lock_guard<std::mutex> lock(fatal_error_mutex);
      fatal_message.clear();
    }

    try {
      class_names = omniseer::vision::load_class_list_file(config.class_list_path);

      omniseer::vision::V4l2Capture::Config capture_config{};
      capture_config.device = config.camera_device;
      capture_config.width = checked_u32(config.camera_width, "camera.width");
      capture_config.height = checked_u32(config.camera_height, "camera.height");
      capture_config.fourcc = V4L2_PIX_FMT_NV12;
      capture_config.buffer_count = checked_u32(config.camera_buffer_count, "camera.buffer_count");
      capture = std::make_unique<omniseer::vision::V4l2Capture>(capture_config);
      capture->start();

      allocator = std::make_unique<omniseer::vision::DmaHeapAllocator>();
      pool = std::make_unique<omniseer::vision::ImageBufferPool>();
      pool->allocate_all(*allocator,
          static_cast<int>(checked_u32(config.pipeline_dst_width, "pipeline.dst_width")),
                           static_cast<int>(checked_u32(config.pipeline_dst_height,
          "pipeline.dst_height")),
                           omniseer::vision::PixelFormat::RGB888);

      omniseer::vision::RgaPreprocessConfig preprocess_config{};
      preprocess_config.src_w = static_cast<int>(checked_u32(config.camera_width, "camera.width"));
      preprocess_config.src_h = static_cast<int>(checked_u32(config.camera_height, "camera.height"));
      preprocess_config.dst_w = static_cast<int>(checked_u32(config.pipeline_dst_width, "pipeline.dst_width"));
      preprocess_config.dst_h = static_cast<int>(checked_u32(config.pipeline_dst_height, "pipeline.dst_height"));
      preprocess_config.pad_value = 114;
      preprocess = std::make_unique<omniseer::vision::RgaPreprocess>(preprocess_config);
      prefill_pool(*pool, *preprocess);

      omniseer::vision::ProducerPipelineConfig producer_config{};
      producer_config.preflight_capture_wait_ms = checked_u32(
        config.producer_preflight_capture_wait_ms, "producer.preflight_capture_wait_ms");
      producer = std::make_unique<omniseer::vision::ProducerPipeline>(
        *capture, *preprocess, *pool, &rolling_stats, producer_config);
      producer->preflight();

      omniseer::vision::YoloWorldTextEmbeddingsBuilderConfig embeddings_config{};
      embeddings_config.text_encoder_model_path = config.clip_model_path;
      embeddings_config.detector_model_path = config.detector_model_path;
      embeddings_config.clip_vocab_path = config.clip_vocab_path;
      embeddings_config.pad_token = config.pad_token;
      text_builder = std::make_unique<omniseer::vision::YoloWorldTextEmbeddingsBuilder>(
        embeddings_config);
      prepared_embeddings = std::make_unique<omniseer::vision::PreparedTextEmbeddings>(
        text_builder->build(class_names));

      omniseer::vision::RknnRunnerConfig runner_config{};
      runner_config.model_path = config.detector_model_path;
      runner_config.warmup_runs = checked_non_negative_u32(
        config.runner_warmup_runs, "runner.warmup_runs");
      runner = std::make_unique<omniseer::vision::RknnRunner>(runner_config);

      omniseer::vision::ConsumerPipelineConfig consumer_config{};
      consumer_config.score_threshold = checked_unit_float(
        config.score_threshold, "postprocess.score_threshold");
      consumer_config.nms_iou_threshold = checked_unit_float(
        config.nms_iou_threshold, "postprocess.nms_iou_threshold");
      consumer_config.max_detections = checked_u32(
        config.max_detections, "postprocess.max_detections");
      consumer = std::make_unique<omniseer::vision::ConsumerPipeline>(
        *pool, *runner, &rolling_stats, config.detections_sink, consumer_config);
      omniseer::vision::ConsumerPipelineStartup consumer_startup{};
      consumer_startup.remap = producer->remap();
      consumer_startup.text_embeddings = prepared_embeddings->view();
      consumer->preflight(consumer_startup);

      producer_thread = std::thread([this]() noexcept {producer_loop();});
      consumer_thread = std::thread([this]() noexcept {consumer_loop();});
      running.store(true, std::memory_order_release);
    } catch (...) {
      stop();
      throw;
    }
  }

  void stop() noexcept
  {
    stop_requested.store(true, std::memory_order_release);

    if (producer_thread.joinable()) {
      producer_thread.join();
    }
    if (consumer_thread.joinable()) {
      consumer_thread.join();
    }

    if (capture) {
      capture->stop();
    }

    consumer.reset();
    runner.reset();
    prepared_embeddings.reset();
    text_builder.reset();
    producer.reset();
    preprocess.reset();
    pool.reset();
    allocator.reset();
    capture.reset();
    class_names.clear();

    running.store(false, std::memory_order_release);
  }

  bool has_fatal_error() const noexcept
  {
    return fatal_error.load(std::memory_order_acquire);
  }

  std::string fatal_error_message() const
  {
    std::lock_guard<std::mutex> lock(fatal_error_mutex);
    return fatal_message;
  }

  void producer_loop() noexcept
  {
    while (!stop_requested.load(std::memory_order_acquire)) {
      const auto tick = producer->run();
      switch (tick.status) {
        case omniseer::vision::ProducerTickStatus::Produced:
          break;
        case omniseer::vision::ProducerTickStatus::NoFrame:
        case omniseer::vision::ProducerTickStatus::CaptureRetryableError:
        case omniseer::vision::ProducerTickStatus::NoWritableBuffer:
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          break;
        case omniseer::vision::ProducerTickStatus::CaptureFatalError:
          record_fatal("producer stopped: status=capture_fatal_error stage=" +
                       std::to_string(static_cast<unsigned>(tick.stage)) + " errno=" +
                       std::to_string(tick.stage_errno));
          return;
        case omniseer::vision::ProducerTickStatus::PreprocessError:
          record_fatal("producer stopped: status=preprocess_error stage=" +
                       std::to_string(static_cast<unsigned>(tick.stage)) + " errno=" +
                       std::to_string(tick.stage_errno));
          return;
      }
    }
  }

  void consumer_loop() noexcept
  {
    while (!stop_requested.load(std::memory_order_acquire)) {
      const auto tick = consumer->run();
      switch (tick.status) {
        case omniseer::vision::ConsumerTickStatus::Consumed:
          break;
        case omniseer::vision::ConsumerTickStatus::NoReadyBuffer:
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          break;
        case omniseer::vision::ConsumerTickStatus::InferError:
          record_fatal("consumer stopped: status=infer_error stage=" +
                       std::to_string(static_cast<unsigned>(tick.stage)) + " errno=" +
                       std::to_string(tick.stage_errno));
          return;
      }
    }
  }

  void validate_config() const
  {
    if (config.class_list_path.empty()) {
      throw std::invalid_argument("classes.path must not be empty");
    }
    if (config.detector_model_path.empty()) {
      throw std::invalid_argument("models.detector_model_path must not be empty");
    }
    if (config.clip_model_path.empty()) {
      throw std::invalid_argument("models.clip_model_path must not be empty");
    }
    if (config.clip_vocab_path.empty()) {
      throw std::invalid_argument("models.clip_vocab_path must not be empty");
    }
    (void) checked_u32(config.camera_width, "camera.width");
    (void) checked_u32(config.camera_height, "camera.height");
    (void) checked_u32(config.camera_buffer_count, "camera.buffer_count");
    (void) checked_u32(config.pipeline_dst_width, "pipeline.dst_width");
    (void) checked_u32(config.pipeline_dst_height, "pipeline.dst_height");
    (void) checked_u32(config.producer_preflight_capture_wait_ms,
                         "producer.preflight_capture_wait_ms");
    (void) checked_non_negative_u32(config.runner_warmup_runs, "runner.warmup_runs");
    (void) checked_u32(config.max_detections, "postprocess.max_detections");
    (void) checked_unit_float(config.score_threshold, "postprocess.score_threshold");
    (void) checked_unit_float(config.nms_iou_threshold, "postprocess.nms_iou_threshold");
  }

  void record_fatal(const std::string & message) noexcept
  {
    {
      std::lock_guard<std::mutex> lock(fatal_error_mutex);
      if (fatal_message.empty()) {
        fatal_message = message;
      }
    }
    fatal_error.store(true, std::memory_order_release);
    stop_requested.store(true, std::memory_order_release);
  }

  VisionBridgeRuntimeConfig config{};
  std::atomic<bool> stop_requested{false};
  std::atomic<bool> running{false};
  std::atomic<bool> fatal_error{false};
  mutable std::mutex        fatal_error_mutex{};
  std::string               fatal_message{};

  omniseer::vision::RollingTelemetryStats rolling_stats{};
  std::vector<std::string> class_names{};

  std::unique_ptr<omniseer::vision::V4l2Capture> capture{};
  std::unique_ptr<omniseer::vision::DmaHeapAllocator> allocator{};
  std::unique_ptr<omniseer::vision::ImageBufferPool> pool{};
  std::unique_ptr<omniseer::vision::RgaPreprocess> preprocess{};
  std::unique_ptr<omniseer::vision::ProducerPipeline> producer{};
  std::unique_ptr<omniseer::vision::YoloWorldTextEmbeddingsBuilder> text_builder{};
  std::unique_ptr<omniseer::vision::PreparedTextEmbeddings> prepared_embeddings{};
  std::unique_ptr<omniseer::vision::RknnRunner> runner{};
  std::unique_ptr<omniseer::vision::ConsumerPipeline> consumer{};

  std::thread producer_thread{};
  std::thread consumer_thread{};
};

VisionBridgeRuntime::VisionBridgeRuntime(VisionBridgeRuntimeConfig cfg)
: _impl(std::make_unique<Impl>(std::move(cfg)))
{
}

VisionBridgeRuntime::~VisionBridgeRuntime()
{
  stop();
}

void VisionBridgeRuntime::start()
{
  _impl->start();
}

void VisionBridgeRuntime::stop() noexcept
{
  _impl->stop();
}

bool VisionBridgeRuntime::is_running() const noexcept
{
  return _impl->running.load(std::memory_order_acquire);
}

bool VisionBridgeRuntime::has_fatal_error() const noexcept
{
  return _impl->has_fatal_error();
}

std::string VisionBridgeRuntime::fatal_error_message() const
{
  return _impl->fatal_error_message();
}

omniseer::vision::RollingTelemetrySnapshot VisionBridgeRuntime::telemetry_snapshot() const noexcept
{
  return _impl->rolling_stats.snapshot();
}
} // namespace omniseer_vision_bridge
