#pragma once

#include <cstdint>

#include "omniseer/vision/image_buffer_pool.hpp"
#include "omniseer/vision/rga_preprocess.hpp"
#include "omniseer/vision/v4l2_capture.hpp"

namespace omniseer::vision
{
  enum class ProducerTickStatus : uint8_t
  {
    Produced,
    NoFrame,
    CaptureRetryableError,
    CaptureFatalError,
    NoWritableBuffer,
    PreprocessError,
  };

  struct ProducerTickResult
  {
    ProducerTickStatus status{ProducerTickStatus::NoFrame};
    CaptureResult      capture{};
    PreprocessResult   preprocess{};

    bool produced() const noexcept
    {
      return status == ProducerTickStatus::Produced;
    }
  };

  // Minimal producer-side pipeline orchestration.
  // Preconditions (owned/enforced by runtime/orchestrator):
  // - capture.start() already called.
  // - pool.allocate_all(...) already succeeded.
  // - destination buffers were prefixed once via preprocess.prefill(...).
  //
  // One step performs:
  // dequeue_lease -> acquire_write_lease -> preprocess -> stamp metadata -> publish_ready.
  class ProducerPipeline
  {
  public:
    ProducerPipeline(V4l2Capture& capture, RgaPreprocess& preprocess, ImageBufferPool& pool);

    // One-time startup validation hook. Throws on failure.
    // Safe to call multiple times.
    void preflight();

    // Non-blocking producer step.
    // Returns explicit status for all hot-path outcomes.
    // This function does not throw by contract.
    ProducerTickResult run() noexcept;

  private:
    V4l2Capture&     _capture;
    RgaPreprocess&   _preprocess;
    ImageBufferPool& _pool;
    bool             _armed{false};
  };
} // namespace omniseer::vision
