#pragma once

#include <cstdint>

namespace omniseer::vision
{
  enum class ProducerStageMask : uint32_t
  {
    Dequeue      = 1u << 0,
    AcquireWrite = 1u << 1,
    Preprocess   = 1u << 2,
    PublishReady = 1u << 3,
    Requeue      = 1u << 4,
    Total        = 1u << 5,
  };

  enum class ConsumerStageMask : uint32_t
  {
    AcquireRead = 1u << 0,
    Infer       = 1u << 1,
    Postprocess = 1u << 2,
    Publish     = 1u << 3,
    Release     = 1u << 4,
    Total       = 1u << 5,
  };

  struct ProducerSample
  {
    uint64_t frame_id{0};
    uint64_t tick_id{0};
    uint64_t sequence{0};
    uint64_t event_ts_real_ns{0};
    uint64_t dequeue_ns{0};
    uint64_t acquire_write_ns{0};
    uint64_t preprocess_ns{0};
    uint64_t publish_ready_ns{0};
    uint64_t requeue_ns{0};
    uint64_t total_ns{0};

    uint32_t stage_mask{0};
    int32_t  capture_errno{0};

    uint8_t producer_status{0};
    uint8_t capture_status{0};
    uint8_t preprocess_status{0};
    uint8_t has_frame_id{0};
    uint8_t has_sequence{0};
  };

  struct ConsumerSample
  {
    uint64_t frame_id{0};
    uint64_t tick_id{0};
    uint64_t sequence{0};
    uint64_t event_ts_real_ns{0};
    uint64_t acquire_read_ns{0};
    uint64_t infer_ns{0};
    uint64_t postprocess_ns{0};
    uint64_t publish_ns{0};
    uint64_t release_ns{0};
    uint64_t total_ns{0};

    uint32_t stage_mask{0};
    int32_t  infer_errno{0};

    uint8_t consumer_status{0};
    uint8_t infer_status{0};
    uint8_t postprocess_status{0};
    uint8_t has_frame_id{0};
    uint8_t has_sequence{0};
  };

  class ITelemetry
  {
  public:
    virtual ~ITelemetry() = default;

    // Absence is represented by nullptr at call sites.
    virtual bool timing_enabled() const noexcept = 0;

    virtual void emit_producer(const ProducerSample& sample) noexcept = 0;
    virtual void emit_consumer(const ConsumerSample& sample) noexcept = 0;
  };
} // namespace omniseer::vision
