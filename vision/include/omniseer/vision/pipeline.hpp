#pragma once

#include "omniseer/vision/image_buffer_pool.hpp"
#include "omniseer/vision/rga_preprocess.hpp"
#include "omniseer/vision/v4l2_capture.hpp"

namespace omniseer::vision
{
  // Minimal producer-side pipeline orchestration.
  // Preconditions (owned/enforced by runtime/orchestrator):
  // - capture.start() already called.
  // - pool.allocate_all(...) already succeeded.
  // - destination buffers were prefixed once via preprocess.prefill(...).
  //
  // One step performs:
  // dequeue -> acquire_write -> preprocess -> stamp metadata -> publish_ready -> requeue.
  class ProducerPipeline
  {
  public:
    ProducerPipeline(V4l2Capture& capture, RgaPreprocess& preprocess, ImageBufferPool& pool);

    // Non-blocking producer step.
    // Returns true when a new processed frame was published to the pool.
    // Returns false when no frame is ready or no writable buffer is available.
    // Throws on preprocess failures (fail-fast; no abort path yet).
    bool run_once();

  private:
    V4l2Capture&     _capture;
    RgaPreprocess&   _preprocess;
    ImageBufferPool& _pool;
  };
} // namespace omniseer::vision
