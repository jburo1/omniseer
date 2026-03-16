#include "omniseer/vision/consumer_pipeline.hpp"

#include <stdexcept>

#include "omniseer/vision/image_buffer_pool.hpp"
#include "omniseer/vision/rknn_runner.hpp"

namespace omniseer::vision
{
  ConsumerPipeline::ConsumerPipeline(ImageBufferPool& pool, RknnRunner& runner,
                                     ITelemetry* telemetry, ConsumerPipelineConfig cfg) noexcept
      : _pool(pool), _runner(runner), _telemetry(telemetry), _cfg(cfg)
  {
  }

  void ConsumerPipeline::preflight(const ConsumerPipelineStartup& startup)
  {
    if (startup.text_embeddings.data == nullptr)
      throw std::invalid_argument("ConsumerPipeline::preflight: text_embeddings.data is null");
    if (startup.text_embeddings.bytes == 0)
      throw std::invalid_argument("ConsumerPipeline::preflight: text_embeddings.bytes is zero");

    _runner.preflight(_pool, startup.text_embeddings.data, startup.text_embeddings.bytes);
    _remap = startup.remap;
    _armed = true;
  }

  bool ConsumerPipeline::is_armed() const noexcept
  {
    return _armed;
  }
} // namespace omniseer::vision
