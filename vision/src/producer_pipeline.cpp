#include "omniseer/vision/producer_pipeline.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

#include "omniseer/vision/image_buffer_pool.hpp"
#include "omniseer/vision/rga_preprocess.hpp"
#include "omniseer/vision/telemetry.hpp"
#include "telemetry_timing.hpp"

namespace omniseer::vision
{
  namespace
  {
    using telemetry_timing::ScopedStageTimer;
    using telemetry_timing::clock;

    uint64_t source_age_ns(uint64_t source_ts_real_ns, uint64_t sample_ts_real_ns) noexcept
    {
      if (source_ts_real_ns == 0 || sample_ts_real_ns < source_ts_real_ns)
        return 0;
      return sample_ts_real_ns - source_ts_real_ns;
    }

    uint32_t stage_mask_bit(ProducerStageMask bit) noexcept
    {
      return static_cast<uint32_t>(bit);
    }

    ProducerTickStatus map_capture_tick_status(CaptureStatus status) noexcept
    {
      switch (status)
      {
      case CaptureStatus::NoFrame:
        return ProducerTickStatus::NoFrame;
      case CaptureStatus::RetryableError:
        return ProducerTickStatus::CaptureRetryableError;
      case CaptureStatus::FatalError:
        return ProducerTickStatus::CaptureFatalError;
      case CaptureStatus::Ok:
        break;
      }
      return ProducerTickStatus::CaptureFatalError;
    }
  } // namespace

  ProducerPipeline::ProducerPipeline(V4l2Capture& capture, RgaPreprocess& preprocess,
                                     ImageBufferPool& pool, ITelemetry* telemetry,
                                     ProducerPipelineConfig cfg) noexcept
      : _capture(capture), _preprocess(preprocess), _pool(pool), _telemetry(telemetry),
        _cfg(cfg)
  {
  }

  void ProducerPipeline::preflight()
  {
    auto write_lease = _pool.acquire_write_lease();
    if (!write_lease.has_value())
      throw std::runtime_error("ProducerPipeline::preflight: no writable buffer");

    using clock = std::chrono::steady_clock;
    const clock::time_point deadline =
        clock::now() + std::chrono::milliseconds(_cfg.preflight_capture_wait_ms);

    V4l2Capture::DequeueLeaseResult dq{};
    for (;;)
    {
      dq = _capture.dequeue_lease();
      if (dq.ok())
        break;

      if (dq.capture.status == CaptureStatus::FatalError)
      {
        throw std::runtime_error("ProducerPipeline::preflight: capture dequeue fatal error");
      }

      if (clock::now() >= deadline)
      {
        throw std::runtime_error(
            "ProducerPipeline::preflight: timed out waiting for capture frame");
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    LetterboxMeta          lb{};
    const PreprocessResult preprocess =
        _preprocess.preflight(dq.lease->frame(), write_lease->buffer(), &lb);
    if (!preprocess.ok())
      throw std::runtime_error("ProducerPipeline::preflight: preprocess preflight failed");

    _remap.source_size      = dq.lease->frame().size;
    _remap.model_input_size = write_lease->buffer().size;
    _remap.scale            = lb.scale;
    _remap.pad_x            = lb.pad_x;
    _remap.pad_y            = lb.pad_y;
    _remap.resized_w        = lb.resized_w;
    _remap.resized_h        = lb.resized_h;

    _armed = true;
  }

  bool ProducerPipeline::is_armed() const noexcept
  {
    return _armed;
  }

  ProducerTick ProducerPipeline::run() noexcept
  {
    ProducerTick tick{};

    const bool       telemetry_on = (_telemetry != nullptr && _telemetry->timing_enabled());
    const clock::time_point total_start = telemetry_on ? clock::now() : clock::time_point{};
    CaptureStatus    sample_capture_status    = CaptureStatus::Ok;
    PreprocessStatus sample_preprocess_status = PreprocessStatus::Ok;
    uint64_t         sample_source_age_dequeue_ns       = 0;
    uint64_t         sample_source_age_publish_ready_ns = 0;
    uint64_t         sample_dequeue_ns        = 0;
    uint64_t         sample_acquire_write_ns  = 0;
    uint64_t         sample_preprocess_ns     = 0;
    uint64_t         sample_publish_ready_ns  = 0;
    uint64_t         sample_requeue_ns        = 0;

    auto emit_sample = [&]() noexcept
    {
      if (!telemetry_on || tick.status == ProducerTickStatus::NoFrame)
        return;

      ProducerSample sample{};
      sample.tick_id           = _next_tick_id++;
      sample.frame_id          = tick.frame_id;
      sample.has_frame_id      = (tick.frame_id != 0) ? 1u : 0u;
      sample.sequence          = tick.sequence;
      sample.has_sequence      = (tick.sequence != 0) ? 1u : 0u;
      sample.event_ts_real_ns  = tick.capture_ts_real_ns;
      sample.source_age_dequeue_ns = sample_source_age_dequeue_ns;
      sample.source_age_publish_ready_ns = sample_source_age_publish_ready_ns;
      sample.dequeue_ns        = sample_dequeue_ns;
      sample.acquire_write_ns  = sample_acquire_write_ns;
      sample.preprocess_ns     = sample_preprocess_ns;
      sample.publish_ready_ns  = sample_publish_ready_ns;
      sample.requeue_ns        = sample_requeue_ns;
      sample.total_ns          = telemetry_timing::elapsed_ns(total_start, clock::now());
      sample.stage_mask        = tick.stage_mask;
      sample.capture_errno     = tick.stage_errno;
      sample.producer_status   = static_cast<uint8_t>(tick.status);
      sample.capture_status    = static_cast<uint8_t>(sample_capture_status);
      sample.preprocess_status = static_cast<uint8_t>(sample_preprocess_status);
      _telemetry->emit_producer(sample);
    };

    auto finish = [&](ProducerTickStatus status, ProducerStage stage, int stage_errno = 0) noexcept
    {
      tick.status      = status;
      tick.stage       = stage;
      tick.stage_errno = stage_errno;
      emit_sample();
      return tick;
    };

    // Stage 1: Dequeue one capture frame.
    auto dq = [&]() noexcept
    {
      ScopedStageTimer timer(telemetry_on, sample_dequeue_ns);
      return _capture.dequeue_lease();
    }();
    if (!dq.ok())
    {
      sample_capture_status = dq.capture.status;
      return finish(map_capture_tick_status(dq.capture.status), ProducerStage::Dequeue,
                    dq.capture.sys_errno);
    }
    tick.sequence           = dq.lease->frame().sequence;
    tick.capture_ts_real_ns = dq.lease->frame().capture_ts_real_ns;
    if (telemetry_on)
      sample_source_age_dequeue_ns =
          source_age_ns(tick.capture_ts_real_ns, telemetry_timing::now_real_ns());
    tick.stage_mask |= stage_mask_bit(ProducerStageMask::Dequeue);

    // Stage 2: Acquire writable destination slot.
    auto write_lease = [&]() noexcept
    {
      ScopedStageTimer timer(telemetry_on, sample_acquire_write_ns);
      return _pool.acquire_write_lease();
    }();
    if (!write_lease.has_value())
      return finish(ProducerTickStatus::NoWritableBuffer, ProducerStage::AcquireWrite);
    tick.pool_index = write_lease->index();
    tick.stage_mask |= stage_mask_bit(ProducerStageMask::AcquireWrite);

    // Stage 3: Preprocess src -> dst.
    const PreprocessResult preprocess = [&]() noexcept
    {
      ScopedStageTimer timer(telemetry_on, sample_preprocess_ns);
      return _preprocess.run(dq.lease->frame(), write_lease->buffer(), nullptr);
    }();
    sample_preprocess_status = preprocess.status;
    if (!preprocess.ok())
      return finish(ProducerTickStatus::PreprocessError, ProducerStage::Preprocess);
    tick.stage_mask |= stage_mask_bit(ProducerStageMask::Preprocess);

    // Stage 4: Publish destination slot as ready.
    {
      ScopedStageTimer timer(telemetry_on, sample_publish_ready_ns);
      write_lease->buffer().sequence           = static_cast<uint32_t>(tick.sequence);
      write_lease->buffer().capture_ts_real_ns = tick.capture_ts_real_ns;
      tick.frame_id                            = _next_frame_id++;
      write_lease->buffer().frame_id           = tick.frame_id;
      write_lease->publish();
      if (telemetry_on)
        sample_source_age_publish_ready_ns =
            source_age_ns(tick.capture_ts_real_ns, telemetry_timing::now_real_ns());
    }
    tick.stage_mask |= stage_mask_bit(ProducerStageMask::PublishReady);

    // Stage 5: Requeue capture frame back to V4L2.
    const CaptureResult requeue = [&]() noexcept
    {
      ScopedStageTimer timer(telemetry_on, sample_requeue_ns);
      return dq.lease->release();
    }();
    sample_capture_status       = requeue.status;
    if (!requeue.ok())
      return finish(map_capture_tick_status(requeue.status), ProducerStage::Requeue,
                    requeue.sys_errno);
    tick.stage_mask |= stage_mask_bit(ProducerStageMask::Requeue);

    return finish(ProducerTickStatus::Produced, ProducerStage::Requeue);
  }

  const PipelineRemapConfig& ProducerPipeline::remap() const noexcept
  {
    return _remap;
  }
} // namespace omniseer::vision
