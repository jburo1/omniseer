#pragma once

#include <cstdint>

namespace omniseer::vision
{
  class ImageBufferPool;
  class ITelemetry;
  class RknnRunner;

  /**
   * @brief Consumer pipeline stage reached by one tick.
   */
  enum class ConsumerStage : uint8_t
  {
    None,
    AcquireRead,
    Infer,
    Postprocess,
    Publish,
    Release,
  };

  /**
   * @brief High-level outcome of one consumer tick.
   */
  enum class ConsumerTickStatus : uint8_t
  {
    Consumed,
    NoReadyBuffer,
    InferError,
  };

  /**
   * @brief One non-throwing consumer loop iteration result.
   */
  struct ConsumerTick
  {
    /// @brief Overall tick status.
    ConsumerTickStatus status{ConsumerTickStatus::NoReadyBuffer};
    /// @brief Last stage reached before returning.
    ConsumerStage stage{ConsumerStage::None};
    /// @brief errno associated with the failure stage, if any.
    int stage_errno{0};
    /// @brief OR-ed ConsumerStageMask bits for completed stages.
    uint32_t stage_mask{0};

    /// @brief Source capture sequence copied from input buffer
    uint64_t sequence{0};
    /// @brief Source capture timestamp on realtime clock in nanoseconds
    uint64_t capture_ts_real_ns{0};
    /// @brief Monotonic logical frame identifier assigned by the pipeline
    uint64_t frame_id{0};
    /// @brief Pool slot index used for this tick, or -1 if none.
    int32_t pool_index{-1};
  };

  /**
   * @brief Consumer pipeline: acquire-ready -> infer -> postprocess/publish -> release.
   *
   * Ownership:
   * - Non-owning references to pool and optional RKNN runner/telemetry collaborators.
   *
   * Error policy:
   * - preflight() may throw on startup validation failures.
   * - run() is noexcept and returns ConsumerTick with stage/status details.
   *
   * Contract:
   * - Caller is expected to call preflight() before entering the run loop.
   * - run() executes one consumer pass in stage order:
   *   1) acquire ready input buffer
   *   2) run inference
   *   3) postprocess/publish results
   *   4) release input buffer back to pool
   * - run() returns early on first non-success stage outcome.
   */
  class ConsumerPipeline
  {
  public:
    ConsumerPipeline(ImageBufferPool& pool, RknnRunner* runner = nullptr,
                     ITelemetry* telemetry = nullptr) noexcept;
    ~ConsumerPipeline() = default;

    ConsumerPipeline(const ConsumerPipeline&)            = delete;
    ConsumerPipeline& operator=(const ConsumerPipeline&) = delete;
    ConsumerPipeline(ConsumerPipeline&&)                 = delete;
    ConsumerPipeline& operator=(ConsumerPipeline&&)      = delete;

    /**
     * @brief Validate startup assumptions and arm the hot path.
     */
    void preflight();

    /**
     * @brief Run one consumer tick on the hot path.
     *
     * Performs one acquire->infer->postprocess/publish->release pass and returns the
     * first stage outcome reached.
     *
     * @return ConsumerTick carrying stage, status, and diagnostics.
     */
    ConsumerTick run() noexcept;

    /// @brief True after successful preflight.
    bool is_armed() const noexcept;

  private:
    ImageBufferPool& _pool;
    RknnRunner*      _runner{nullptr};
    ITelemetry*      _telemetry{nullptr};

    bool     _armed{false};
    uint64_t _next_tick_id{1};
  };
} // namespace omniseer::vision
