#pragma once

#include <cstdint>

namespace omniseer::vision
{
  /**
   * @brief Bitmask of completed producer-stage milestones for one sample.
   */
  enum class ProducerStageMask : uint32_t
  {
    Dequeue      = 1u << 0,
    AcquireWrite = 1u << 1,
    Preprocess   = 1u << 2,
    PublishReady = 1u << 3,
    Requeue      = 1u << 4,
  };

  /**
   * @brief Bitmask of completed consumer-stage milestones for one sample.
   */
  enum class ConsumerStageMask : uint32_t
  {
    AcquireRead = 1u << 0,
    Infer       = 1u << 1,
    Postprocess = 1u << 2,
    Publish     = 1u << 3,
    Release     = 1u << 4,
  };

  /**
   * @brief Per-frame producer-side telemetry payload.
   */
  struct ProducerSample
  {
    /// @brief Logical frame identifier, when available.
    uint64_t frame_id{0};
    /// @brief Monotonic telemetry tick identifier.
    uint64_t tick_id{0};
    /// @brief Source capture sequence, when available.
    uint64_t sequence{0};
    /// @brief Event timestamp on realtime clock (nanoseconds).
    uint64_t event_ts_real_ns{0};
    /// @brief Duration spent in dequeue stage (nanoseconds).
    uint64_t dequeue_ns{0};
    /// @brief Duration spent in acquire-write stage (nanoseconds).
    uint64_t acquire_write_ns{0};
    /// @brief Duration spent in preprocess stage (nanoseconds).
    uint64_t preprocess_ns{0};
    /// @brief Duration spent in publish-ready stage (nanoseconds).
    uint64_t publish_ready_ns{0};
    /// @brief Duration spent in requeue stage (nanoseconds).
    uint64_t requeue_ns{0};
    /// @brief End-to-end producer duration (nanoseconds).
    uint64_t total_ns{0};

    /// @brief OR-ed ProducerStageMask flags for stages that executed.
    uint32_t stage_mask{0};
    /// @brief Captured errno associated with capture operations, if any.
    int32_t  capture_errno{0};

    /// @brief Producer status code (domain-specific).
    uint8_t producer_status{0};
    /// @brief Capture status code (domain-specific).
    uint8_t capture_status{0};
    /// @brief Preprocess status code (domain-specific).
    uint8_t preprocess_status{0};
    /// @brief True when frame_id is valid for this sample.
    uint8_t has_frame_id{0};
    /// @brief True when sequence is valid for this sample.
    uint8_t has_sequence{0};
  };

  /**
   * @brief Per-frame consumer-side telemetry payload.
   */
  struct ConsumerSample
  {
    /// @brief Logical frame identifier, when available.
    uint64_t frame_id{0};
    /// @brief Monotonic telemetry tick identifier.
    uint64_t tick_id{0};
    /// @brief Source capture sequence, when available.
    uint64_t sequence{0};
    /// @brief Event timestamp on realtime clock (nanoseconds).
    uint64_t event_ts_real_ns{0};
    /// @brief Duration spent in acquire-read stage (nanoseconds).
    uint64_t acquire_read_ns{0};
    /// @brief Duration spent in infer stage (nanoseconds).
    uint64_t infer_ns{0};
    /// @brief Duration spent in postprocess stage (nanoseconds).
    uint64_t postprocess_ns{0};
    /// @brief Duration spent in publish stage (nanoseconds).
    uint64_t publish_ns{0};
    /// @brief Duration spent in release stage (nanoseconds).
    uint64_t release_ns{0};
    /// @brief End-to-end consumer duration (nanoseconds).
    uint64_t total_ns{0};

    /// @brief OR-ed ConsumerStageMask flags for stages that executed.
    uint32_t stage_mask{0};
    /// @brief Captured errno associated with inference operations, if any.
    int32_t  infer_errno{0};

    /// @brief Consumer status code (domain-specific).
    uint8_t consumer_status{0};
    /// @brief Inference status code (domain-specific).
    uint8_t infer_status{0};
    /// @brief Postprocess status code (domain-specific).
    uint8_t postprocess_status{0};
    /// @brief True when frame_id is valid for this sample.
    uint8_t has_frame_id{0};
    /// @brief True when sequence is valid for this sample.
    uint8_t has_sequence{0};
  };

  /**
   * @brief Telemetry sink contract for producer/consumer timing and status samples.
   */
  class ITelemetry
  {
  public:
    virtual ~ITelemetry() = default;

    /// @brief Return true when timing collection should be performed by call sites.
    virtual bool timing_enabled() const noexcept = 0;

    /// @brief Emit one producer telemetry sample.
    virtual void emit_producer(const ProducerSample& sample) noexcept = 0;
    /// @brief Emit one consumer telemetry sample.
    virtual void emit_consumer(const ConsumerSample& sample) noexcept = 0;
  };
} // namespace omniseer::vision
