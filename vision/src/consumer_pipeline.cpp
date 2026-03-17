#include "omniseer/vision/consumer_pipeline.hpp"

#include <cerrno>
#include <stdexcept>

#include "omniseer/vision/detections.hpp"
#include "omniseer/vision/detections_sink.hpp"
#include "omniseer/vision/frame_preview_sink.hpp"
#include "omniseer/vision/image_buffer_pool.hpp"
#include "omniseer/vision/rknn_runner.hpp"
#include "omniseer/vision/telemetry.hpp"
#include "omniseer/vision/yolo_world_postprocess.hpp"
#include "telemetry_timing.hpp"

namespace omniseer::vision
{
  namespace
  {
    using telemetry_timing::clock;
    using telemetry_timing::ScopedStageTimer;

    uint32_t stage_mask_bit(ConsumerStageMask bit) noexcept
    {
      return static_cast<uint32_t>(bit);
    }
  } // namespace

  ConsumerPipeline::ConsumerPipeline(ImageBufferPool& pool, RknnRunner& runner,
                                     ITelemetry* telemetry, IDetectionsSink* sink,
                                     ConsumerPipelineConfig cfg) noexcept
      : _pool(pool), _runner(runner), _telemetry(telemetry), _sink(sink), _cfg(cfg)
  {
  }

  void ConsumerPipeline::preflight(const ConsumerPipelineStartup& startup)
  {
    if (startup.text_embeddings.data == nullptr)
      throw std::invalid_argument("ConsumerPipeline::preflight: text_embeddings.data is null");
    if (startup.text_embeddings.bytes == 0)
      throw std::invalid_argument("ConsumerPipeline::preflight: text_embeddings.bytes is zero");
    if (startup.text_embeddings.active_class_count == 0)
      throw std::invalid_argument(
          "ConsumerPipeline::preflight: text_embeddings.active_class_count is zero");

    _runner.preflight(_pool, startup.text_embeddings.data, startup.text_embeddings.bytes);
    const YoloWorldOutputLayout layout = resolve_yolo_world_output_layout(_runner.output_descs());
    if (startup.text_embeddings.active_class_count > layout.class_capacity)
    {
      throw std::invalid_argument(
          "ConsumerPipeline::preflight: active class count exceeds model class capacity");
    }

    _remap              = startup.remap;
    _active_class_count = startup.text_embeddings.active_class_count;
    _output_layout      = layout;
    _armed              = true;
  }

  bool ConsumerPipeline::is_armed() const noexcept
  {
    return _armed;
  }

  void ConsumerPipeline::set_preview_sink(IFramePreviewSink* sink) noexcept
  {
    _preview_sink = sink;
  }

  ConsumerTick ConsumerPipeline::run() noexcept
  {
    ConsumerTick tick{};
    const bool              telemetry_on = (_telemetry != nullptr && _telemetry->timing_enabled());
    const clock::time_point total_start  = telemetry_on ? clock::now() : clock::time_point{};
    InferStatus             sample_infer_status    = InferStatus::Ok;
    uint64_t                sample_acquire_read_ns = 0;
    uint64_t                sample_infer_ns        = 0;
    uint64_t                sample_postprocess_ns  = 0;
    uint64_t                sample_publish_ns      = 0;
    uint64_t                sample_release_ns      = 0;

    auto emit_sample = [&]() noexcept
    {
      if (!telemetry_on || tick.status == ConsumerTickStatus::NoReadyBuffer)
        return;

      ConsumerSample sample{};
      sample.tick_id            = _next_tick_id++;
      sample.frame_id           = tick.frame_id;
      sample.has_frame_id       = (tick.frame_id != 0) ? 1u : 0u;
      sample.sequence           = tick.sequence;
      sample.has_sequence       = (tick.sequence != 0) ? 1u : 0u;
      sample.event_ts_real_ns   = tick.capture_ts_real_ns;
      sample.acquire_read_ns    = sample_acquire_read_ns;
      sample.infer_ns           = sample_infer_ns;
      sample.postprocess_ns     = sample_postprocess_ns;
      sample.publish_ns         = sample_publish_ns;
      sample.release_ns         = sample_release_ns;
      sample.total_ns           = telemetry_timing::elapsed_ns(total_start, clock::now());
      sample.stage_mask         = tick.stage_mask;
      sample.infer_errno        = tick.stage_errno;
      sample.consumer_status    = static_cast<uint8_t>(tick.status);
      sample.infer_status       = static_cast<uint8_t>(sample_infer_status);
      sample.postprocess_status = 0;
      _telemetry->emit_consumer(sample);
    };

    auto finish = [&](ConsumerTickStatus status, ConsumerStage stage,
                      int stage_errno = 0) noexcept
    {
      tick.status      = status;
      tick.stage       = stage;
      tick.stage_errno = stage_errno;
      emit_sample();
      return tick;
    };

    if (!_armed)
    {
      sample_infer_status = InferStatus::NotArmed;
      return finish(ConsumerTickStatus::InferError, ConsumerStage::Infer, EINVAL);
    }

    auto read_lease = [&]() noexcept
    {
      ScopedStageTimer timer(telemetry_on, sample_acquire_read_ns);
      return _pool.acquire_read_lease();
    }();
    if (!read_lease.has_value())
      return finish(ConsumerTickStatus::NoReadyBuffer, ConsumerStage::AcquireRead);

    tick.sequence           = read_lease->buffer().sequence;
    tick.capture_ts_real_ns = read_lease->buffer().capture_ts_real_ns;
    tick.frame_id           = read_lease->buffer().frame_id;
    tick.pool_index         = read_lease->index();
    tick.stage_mask |= stage_mask_bit(ConsumerStageMask::AcquireRead);

    const InferResult infer = [&]() noexcept
    {
      ScopedStageTimer timer(telemetry_on, sample_infer_ns);
      return _runner.infer(read_lease->index());
    }();
    sample_infer_status = infer.status;
    if (!infer.ok())
      return finish(ConsumerTickStatus::InferError, ConsumerStage::Infer, infer.sys_errno);
    tick.stage_mask |= stage_mask_bit(ConsumerStageMask::Infer);

    DetectionsFrame frame{};
    frame.frame_id           = tick.frame_id;
    frame.sequence           = tick.sequence;
    frame.capture_ts_real_ns = tick.capture_ts_real_ns;
    frame.active_class_count = _active_class_count;
    frame.source_size        = _remap.source_size;
    {
      ScopedStageTimer timer(telemetry_on, sample_postprocess_ns);
      decode_yolo_world_detections(_runner.outputs(), _runner.output_descs(), _output_layout, _cfg,
                                   _remap, _active_class_count, frame);
    }
    tick.stage_mask |= stage_mask_bit(ConsumerStageMask::Postprocess);

    {
      ScopedStageTimer timer(telemetry_on, sample_publish_ns);
      if (_preview_sink != nullptr)
        _preview_sink->publish(read_lease->buffer(), frame, _remap);

      if (_sink != nullptr)
        _sink->publish(frame);
    }
    if (_preview_sink != nullptr || _sink != nullptr)
      tick.stage_mask |= stage_mask_bit(ConsumerStageMask::Publish);

    {
      ScopedStageTimer timer(telemetry_on, sample_release_ns);
      read_lease->release();
    }
    tick.stage_mask |= stage_mask_bit(ConsumerStageMask::Release);

    return finish(ConsumerTickStatus::Consumed, ConsumerStage::Release);
  }
} // namespace omniseer::vision
