#include "omniseer/vision/jsonl_telemetry.hpp"

#include <atomic>
#include <chrono>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "omniseer/vision/consumer_pipeline.hpp"
#include "omniseer/vision/producer_pipeline.hpp"
#include "omniseer/vision/rknn_runner.hpp"
#include "omniseer/vision/spsc_ring.hpp"
#include "omniseer/vision/types.hpp"
#include "omniseer/vision/v4l2_capture.hpp"

namespace omniseer::vision
{
  namespace
  {
    const char* producer_status_name(uint8_t status) noexcept
    {
      switch (static_cast<ProducerTickStatus>(status))
      {
      case ProducerTickStatus::Produced:
        return "produced";
      case ProducerTickStatus::NoFrame:
        return "no_frame";
      case ProducerTickStatus::CaptureRetryableError:
        return "capture_retryable_error";
      case ProducerTickStatus::CaptureFatalError:
        return "capture_fatal_error";
      case ProducerTickStatus::NoWritableBuffer:
        return "no_writable_buffer";
      case ProducerTickStatus::PreprocessError:
        return "preprocess_error";
      }
      return "unknown";
    }

    const char* capture_status_name(uint8_t status) noexcept
    {
      switch (static_cast<CaptureStatus>(status))
      {
      case CaptureStatus::Ok:
        return "ok";
      case CaptureStatus::NoFrame:
        return "no_frame";
      case CaptureStatus::RetryableError:
        return "retryable_error";
      case CaptureStatus::FatalError:
        return "fatal_error";
      }
      return "unknown";
    }

    const char* preprocess_status_name(uint8_t status) noexcept
    {
      switch (static_cast<PreprocessStatus>(status))
      {
      case PreprocessStatus::Ok:
        return "ok";
      case PreprocessStatus::InvalidConfig:
        return "invalid_config";
      case PreprocessStatus::SourceSizeMismatch:
        return "source_size_mismatch";
      case PreprocessStatus::InvalidSourceDescriptor:
        return "invalid_source_descriptor";
      case PreprocessStatus::InvalidDestinationDescriptor:
        return "invalid_destination_descriptor";
      case PreprocessStatus::ImcheckFailed:
        return "imcheck_failed";
      case PreprocessStatus::ImprocessFailed:
        return "improcess_failed";
      case PreprocessStatus::UnknownError:
        return "unknown_error";
      }
      return "unknown";
    }

    const char* consumer_status_name(uint8_t status) noexcept
    {
      switch (static_cast<ConsumerTickStatus>(status))
      {
      case ConsumerTickStatus::Consumed:
        return "consumed";
      case ConsumerTickStatus::NoReadyBuffer:
        return "no_ready_buffer";
      case ConsumerTickStatus::InferError:
        return "infer_error";
      }
      return "unknown";
    }

    const char* infer_status_name(uint8_t status) noexcept
    {
      switch (static_cast<InferStatus>(status))
      {
      case InferStatus::Ok:
        return "ok";
      case InferStatus::NotArmed:
        return "not_armed";
      case InferStatus::InvalidInputDescriptor:
        return "invalid_input_descriptor";
      case InferStatus::RknnError:
        return "rknn_error";
      }
      return "unknown";
    }

    void write_optional_u64(std::ofstream& os, uint64_t value, uint8_t has_value)
    {
      if (has_value != 0)
        os << value;
      else
        os << "null";
    }

    void write_stage_u64(std::ofstream& os, uint32_t stage_mask, uint32_t stage_bit, uint64_t value)
    {
      if ((stage_mask & stage_bit) != 0)
        os << value;
      else
        os << "null";
    }

    void write_producer_json(std::ofstream& os, const ProducerSample& sample)
    {
      os << "{\"schema_version\":1,\"source\":\"producer\",\"frame_id\":";
      write_optional_u64(os, sample.frame_id, sample.has_frame_id);
      os << ",\"tick_id\":" << sample.tick_id << ",\"sequence\":";
      write_optional_u64(os, sample.sequence, sample.has_sequence);
      os << ",\"event_ts_real_ns\":" << sample.event_ts_real_ns;
      os << ",\"producer_status\":\"" << producer_status_name(sample.producer_status) << "\"";
      os << ",\"capture_status\":\"" << capture_status_name(sample.capture_status) << "\"";
      os << ",\"preprocess_status\":\"" << preprocess_status_name(sample.preprocess_status) << "\"";
      os << ",\"capture_errno\":" << sample.capture_errno;
      os << ",\"stage_mask\":" << sample.stage_mask;
      os << ",\"dur_ns\":{\"dequeue\":";
      write_stage_u64(os, sample.stage_mask, static_cast<uint32_t>(ProducerStageMask::Dequeue),
                      sample.dequeue_ns);
      os << ",\"acquire_write\":";
      write_stage_u64(os, sample.stage_mask,
                      static_cast<uint32_t>(ProducerStageMask::AcquireWrite),
                      sample.acquire_write_ns);
      os << ",\"preprocess\":";
      write_stage_u64(os, sample.stage_mask, static_cast<uint32_t>(ProducerStageMask::Preprocess),
                      sample.preprocess_ns);
      os << ",\"publish_ready\":";
      write_stage_u64(os, sample.stage_mask, static_cast<uint32_t>(ProducerStageMask::PublishReady),
                      sample.publish_ready_ns);
      os << ",\"requeue\":";
      write_stage_u64(os, sample.stage_mask, static_cast<uint32_t>(ProducerStageMask::Requeue),
                      sample.requeue_ns);
      os << ",\"total\":" << sample.total_ns << "}}\n";
    }

    void write_consumer_json(std::ofstream& os, const ConsumerSample& sample)
    {
      os << "{\"schema_version\":1,\"source\":\"consumer\",\"frame_id\":";
      write_optional_u64(os, sample.frame_id, sample.has_frame_id);
      os << ",\"tick_id\":" << sample.tick_id << ",\"sequence\":";
      write_optional_u64(os, sample.sequence, sample.has_sequence);
      os << ",\"event_ts_real_ns\":" << sample.event_ts_real_ns;
      os << ",\"consumer_status\":\"" << consumer_status_name(sample.consumer_status) << "\"";
      os << ",\"infer_status\":\"" << infer_status_name(sample.infer_status) << "\"";
      os << ",\"postprocess_status\":" << static_cast<unsigned>(sample.postprocess_status);
      os << ",\"infer_errno\":" << sample.infer_errno;
      os << ",\"stage_mask\":" << sample.stage_mask;
      os << ",\"dur_ns\":{\"acquire_read\":";
      write_stage_u64(os, sample.stage_mask, static_cast<uint32_t>(ConsumerStageMask::AcquireRead),
                      sample.acquire_read_ns);
      os << ",\"infer\":";
      write_stage_u64(os, sample.stage_mask, static_cast<uint32_t>(ConsumerStageMask::Infer),
                      sample.infer_ns);
      os << ",\"postprocess\":";
      write_stage_u64(os, sample.stage_mask, static_cast<uint32_t>(ConsumerStageMask::Postprocess),
                      sample.postprocess_ns);
      os << ",\"publish\":";
      write_stage_u64(os, sample.stage_mask, static_cast<uint32_t>(ConsumerStageMask::Publish),
                      sample.publish_ns);
      os << ",\"release\":";
      write_stage_u64(os, sample.stage_mask, static_cast<uint32_t>(ConsumerStageMask::Release),
                      sample.release_ns);
      os << ",\"total\":" << sample.total_ns << "}}\n";
    }
  } // namespace

  struct JsonlTelemetry::Impl
  {
    explicit Impl(JsonlTelemetryConfig cfg_)
        : cfg(std::move(cfg_)),
          producer_slots(cfg.producer_queue_capacity),
          consumer_slots(cfg.consumer_queue_capacity),
          producer_free(cfg.producer_queue_capacity),
          producer_ready(cfg.producer_queue_capacity),
          consumer_free(cfg.consumer_queue_capacity),
          consumer_ready(cfg.consumer_queue_capacity)
    {
      if (cfg.path.empty())
        throw std::invalid_argument("JsonlTelemetry: path is empty");
      if (cfg.producer_queue_capacity == 0 || cfg.consumer_queue_capacity == 0)
        throw std::invalid_argument("JsonlTelemetry: queue capacity must be > 0");

      out.open(cfg.path, std::ios::out | std::ios::trunc);
      if (!out.is_open())
        throw std::runtime_error("JsonlTelemetry: failed to open output file");

      for (std::size_t i = 0; i < cfg.producer_queue_capacity; ++i)
        producer_free.push(static_cast<int>(i));
      for (std::size_t i = 0; i < cfg.consumer_queue_capacity; ++i)
        consumer_free.push(static_cast<int>(i));

      worker = std::thread([this]() noexcept { run(); });
    }

    ~Impl()
    {
      stop.store(true, std::memory_order_release);
      if (worker.joinable())
        worker.join();
      out.flush();
      out.close();
    }

    void emit_producer(const ProducerSample& sample) noexcept
    {
      if (!enabled.load(std::memory_order_acquire))
        return;

      int idx = -1;
      if (!producer_free.pop(idx))
      {
        producer_dropped.fetch_add(1, std::memory_order_relaxed);
        return;
      }

      producer_slots[static_cast<std::size_t>(idx)] = sample;
      if (!producer_ready.push(idx))
      {
        producer_dropped.fetch_add(1, std::memory_order_relaxed);
        producer_free.push(idx);
      }
    }

    void emit_consumer(const ConsumerSample& sample) noexcept
    {
      if (!enabled.load(std::memory_order_acquire))
        return;

      int idx = -1;
      if (!consumer_free.pop(idx))
      {
        consumer_dropped.fetch_add(1, std::memory_order_relaxed);
        return;
      }

      consumer_slots[static_cast<std::size_t>(idx)] = sample;
      if (!consumer_ready.push(idx))
      {
        consumer_dropped.fetch_add(1, std::memory_order_relaxed);
        consumer_free.push(idx);
      }
    }

    void run() noexcept
    {
      while (true)
      {
        bool wrote_any = false;
        wrote_any |= drain_producer();
        wrote_any |= drain_consumer();

        if (!enabled.load(std::memory_order_acquire))
          break;
        if (stop.load(std::memory_order_acquire) && !wrote_any && producer_ready.empty() &&
            consumer_ready.empty())
          break;

        if (wrote_any)
        {
          out.flush();
          continue;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }

    bool drain_producer() noexcept
    {
      bool wrote = false;
      int  idx   = -1;
      while (producer_ready.pop(idx))
      {
        write_producer_json(out, producer_slots[static_cast<std::size_t>(idx)]);
        producer_free.push(idx);
        wrote = true;
        if (!out.good())
        {
          enabled.store(false, std::memory_order_release);
          break;
        }
      }
      return wrote;
    }

    bool drain_consumer() noexcept
    {
      bool wrote = false;
      int  idx   = -1;
      while (consumer_ready.pop(idx))
      {
        write_consumer_json(out, consumer_slots[static_cast<std::size_t>(idx)]);
        consumer_free.push(idx);
        wrote = true;
        if (!out.good())
        {
          enabled.store(false, std::memory_order_release);
          break;
        }
      }
      return wrote;
    }

    JsonlTelemetryConfig      cfg{};
    std::ofstream             out{};
    std::thread               worker{};
    std::atomic<bool>         enabled{true};
    std::atomic<bool>         stop{false};
    std::atomic<uint64_t>     producer_dropped{0};
    std::atomic<uint64_t>     consumer_dropped{0};
    std::vector<ProducerSample> producer_slots{};
    std::vector<ConsumerSample> consumer_slots{};
    SpscRing                  producer_free;
    SpscRing                  producer_ready;
    SpscRing                  consumer_free;
    SpscRing                  consumer_ready;
  };

  JsonlTelemetry::JsonlTelemetry(JsonlTelemetryConfig cfg) : _impl(new Impl(std::move(cfg)))
  {
  }

  JsonlTelemetry::~JsonlTelemetry()
  {
    delete _impl;
    _impl = nullptr;
  }

  bool JsonlTelemetry::timing_enabled() const noexcept
  {
    return _impl->enabled.load(std::memory_order_acquire);
  }

  void JsonlTelemetry::emit_producer(const ProducerSample& sample) noexcept
  {
    _impl->emit_producer(sample);
  }

  void JsonlTelemetry::emit_consumer(const ConsumerSample& sample) noexcept
  {
    _impl->emit_consumer(sample);
  }
} // namespace omniseer::vision
