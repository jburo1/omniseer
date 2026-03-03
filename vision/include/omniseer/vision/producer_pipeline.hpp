#pragma once

#include <cstdint>

#include "omniseer/vision/config.hpp"
#include "omniseer/vision/v4l2_capture.hpp"

namespace omniseer::vision
{
  class ImageBufferPool;
  class ITelemetry;
  class RgaPreprocess;

  /**
   * @brief Producer pipeline stage reached by one tick.
   */
  enum class ProducerStage : uint8_t
  {
    None,
    Dequeue,
    AcquireWrite,
    Preprocess,
    PublishReady,
    Requeue,
  };

  /**
   * @brief High-level outcome of one producer tick.
   */
  enum class ProducerTickStatus : uint8_t
  {
    Produced,
    NoFrame,
    CaptureRetryableError,
    CaptureFatalError,
    NoWritableBuffer,
    PreprocessError,
  };

  /**
   * @brief One non-throwing producer loop iteration result.
   */
  struct ProducerTick
  {
    /// @brief Overall tick status.
    ProducerTickStatus status{ProducerTickStatus::NoFrame};
    /// @brief Last stage reached before returning.
    ProducerStage stage{ProducerStage::None};
    /// @brief errno associated with the failure stage, if any.
    int stage_errno{0};
    /// @brief OR-ed ProducerStageMask bits for completed stages.
    uint32_t stage_mask{0};

    /// @brief Source capture sequence copied from dequeued frame, when available.
    uint64_t sequence{0};
    /// @brief Capture timestamp on realtime clock in nanoseconds, when available.
    uint64_t capture_ts_real_ns{0};
    /// @brief Monotonic logical frame identifier assigned by the pipeline, when available.
    uint64_t frame_id{0};
    /// @brief Pool slot index used for this tick, or -1 if none.
    int32_t pool_index{-1};
  };

  /**
   * @brief Producer pipeline: capture -> preprocess -> publish-ready -> requeue.
   *
   * Ownership:
   * - Non-owning references to capture/preprocess/pool collaborators.
   *
   * Error policy:
   * - preflight() may throw on startup validation failures.
   * - run() is noexcept and returns ProducerTick with stage/status details.
   *
   * Contract:
   * - Caller is expected to call preflight() before entering the run loop.
   * - run() executes one producer pass in the following stage order:
   *   1) dequeue frame from capture
   *   2) acquire writable destination buffer
   *   3) run preprocess src->dst
   *   4) publish destination as ready
   *   5) requeue capture frame
   * - run() returns early on first non-success stage outcome.
   */
  class ProducerPipeline
  {
  public:
    ProducerPipeline(V4l2Capture& capture, RgaPreprocess& preprocess, ImageBufferPool& pool,
                     ITelemetry* telemetry = nullptr) noexcept;
    ~ProducerPipeline() = default;

    ProducerPipeline(const ProducerPipeline&)            = delete;
    ProducerPipeline& operator=(const ProducerPipeline&) = delete;
    ProducerPipeline(ProducerPipeline&&)                 = delete;
    ProducerPipeline& operator=(ProducerPipeline&&)      = delete;

    /**
     * @brief Validate startup assumptions and arm the hot path.
     *
     */
    void preflight();

    /**
     * @brief Run one producer tick on the hot path.
     *
     * Performs one capture->preprocess->publish->requeue pass and returns the first
     * stage outcome reached.
     *
     * @return ProducerTick carrying stage, status, and diagnostics.
     */
    ProducerTick run() noexcept;

    /// @brief True after successful preflight.
    bool is_armed() const noexcept;

    /// @brief Cached source->model remap geometry from preflight.
    const PipelineRemapConfig& remap() const noexcept;

  private:
    V4l2Capture&     _capture;
    RgaPreprocess&   _preprocess;
    ImageBufferPool& _pool;
    ITelemetry*      _telemetry{nullptr};

    bool                _armed{false};
    uint64_t            _next_tick_id{1};
    uint64_t            _next_frame_id{1};
    PipelineRemapConfig _remap{};
  };
} // namespace omniseer::vision
