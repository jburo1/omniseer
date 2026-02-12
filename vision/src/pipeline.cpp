#include "omniseer/vision/pipeline.hpp"

#include <stdexcept>

namespace omniseer::vision
{
  ProducerPipeline::ProducerPipeline(V4l2Capture& capture, RgaPreprocess& preprocess,
                                     ImageBufferPool& pool)
      : _capture(capture), _preprocess(preprocess), _pool(pool)
  {
  }

  // Non-blocking producer tick.
  bool ProducerPipeline::run_once()
  {
    FrameDescriptor src{};
    if (!_capture.dequeue(src))
      return false;

    int write_idx = -1;
    if (!_pool.acquire_write(write_idx))
    {
      _capture.requeue(src.v4l2_index);
      return false;
    }

    ImageBuffer& dst = _pool.buffer_at(write_idx);
    if (!_preprocess.run(src, dst))
    {
      _capture.requeue(src.v4l2_index);
      throw std::runtime_error("ProducerPipeline::run_once: preprocess failed");
    }

    dst.sequence           = src.sequence;
    dst.capture_ts_real_ns = src.capture_ts_real_ns;
    _pool.publish_ready(write_idx);
    _capture.requeue(src.v4l2_index);

    return true;
  }
} // namespace omniseer::vision
