#pragma once

#include <atomic>
#include <cstdint>

#include "omniseer/vision/consumer_pipeline.hpp"
#include "omniseer/vision/producer_pipeline.hpp"
#include "omniseer/vision/telemetry.hpp"

namespace omniseer::vision
{
  struct RollingTelemetrySnapshot
  {
    uint64_t produced_count{0};
    uint64_t no_writable_buffer_count{0};
    uint64_t capture_retryable_error_count{0};
    uint64_t capture_fatal_error_count{0};
    uint64_t preprocess_error_count{0};

    uint64_t consumed_count{0};
    uint64_t infer_error_count{0};

    uint64_t last_producer_total_ns{0};
    uint64_t last_preprocess_ns{0};
    uint64_t last_consumer_total_ns{0};
    uint64_t last_infer_ns{0};
    uint64_t last_postprocess_ns{0};
  };

  class RollingTelemetryStats final : public ITelemetry
  {
  public:
    bool timing_enabled() const noexcept override
    {
      return true;
    }

    void emit_producer(const ProducerSample& sample) noexcept override
    {
      switch (static_cast<ProducerTickStatus>(sample.producer_status))
      {
      case ProducerTickStatus::Produced:
        _produced_count.fetch_add(1, std::memory_order_relaxed);
        break;
      case ProducerTickStatus::NoWritableBuffer:
        _no_writable_buffer_count.fetch_add(1, std::memory_order_relaxed);
        break;
      case ProducerTickStatus::CaptureRetryableError:
        _capture_retryable_error_count.fetch_add(1, std::memory_order_relaxed);
        break;
      case ProducerTickStatus::CaptureFatalError:
        _capture_fatal_error_count.fetch_add(1, std::memory_order_relaxed);
        break;
      case ProducerTickStatus::PreprocessError:
        _preprocess_error_count.fetch_add(1, std::memory_order_relaxed);
        break;
      case ProducerTickStatus::NoFrame:
        break;
      }

      _last_producer_total_ns.store(sample.total_ns, std::memory_order_relaxed);
      _last_preprocess_ns.store(sample.preprocess_ns, std::memory_order_relaxed);
    }

    void emit_consumer(const ConsumerSample& sample) noexcept override
    {
      switch (static_cast<ConsumerTickStatus>(sample.consumer_status))
      {
      case ConsumerTickStatus::Consumed:
        _consumed_count.fetch_add(1, std::memory_order_relaxed);
        break;
      case ConsumerTickStatus::InferError:
        _infer_error_count.fetch_add(1, std::memory_order_relaxed);
        break;
      case ConsumerTickStatus::NoReadyBuffer:
        break;
      }

      _last_consumer_total_ns.store(sample.total_ns, std::memory_order_relaxed);
      _last_infer_ns.store(sample.infer_ns, std::memory_order_relaxed);
      _last_postprocess_ns.store(sample.postprocess_ns, std::memory_order_relaxed);
    }

    RollingTelemetrySnapshot snapshot() const noexcept
    {
      RollingTelemetrySnapshot out{};
      out.produced_count = _produced_count.load(std::memory_order_relaxed);
      out.no_writable_buffer_count = _no_writable_buffer_count.load(std::memory_order_relaxed);
      out.capture_retryable_error_count =
          _capture_retryable_error_count.load(std::memory_order_relaxed);
      out.capture_fatal_error_count =
          _capture_fatal_error_count.load(std::memory_order_relaxed);
      out.preprocess_error_count = _preprocess_error_count.load(std::memory_order_relaxed);
      out.consumed_count         = _consumed_count.load(std::memory_order_relaxed);
      out.infer_error_count      = _infer_error_count.load(std::memory_order_relaxed);
      out.last_producer_total_ns = _last_producer_total_ns.load(std::memory_order_relaxed);
      out.last_preprocess_ns     = _last_preprocess_ns.load(std::memory_order_relaxed);
      out.last_consumer_total_ns = _last_consumer_total_ns.load(std::memory_order_relaxed);
      out.last_infer_ns          = _last_infer_ns.load(std::memory_order_relaxed);
      out.last_postprocess_ns    = _last_postprocess_ns.load(std::memory_order_relaxed);
      return out;
    }

  private:
    std::atomic<uint64_t> _produced_count{0};
    std::atomic<uint64_t> _no_writable_buffer_count{0};
    std::atomic<uint64_t> _capture_retryable_error_count{0};
    std::atomic<uint64_t> _capture_fatal_error_count{0};
    std::atomic<uint64_t> _preprocess_error_count{0};
    std::atomic<uint64_t> _consumed_count{0};
    std::atomic<uint64_t> _infer_error_count{0};
    std::atomic<uint64_t> _last_producer_total_ns{0};
    std::atomic<uint64_t> _last_preprocess_ns{0};
    std::atomic<uint64_t> _last_consumer_total_ns{0};
    std::atomic<uint64_t> _last_infer_ns{0};
    std::atomic<uint64_t> _last_postprocess_ns{0};
  };
} // namespace omniseer::vision
