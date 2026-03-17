#pragma once

#include <cstddef>
#include <string>

#include "omniseer/vision/telemetry.hpp"

namespace omniseer::vision
{
  struct JsonlTelemetryConfig
  {
    std::string path{};
    std::size_t producer_queue_capacity{512};
    std::size_t consumer_queue_capacity{512};
  };

  class JsonlTelemetry final : public ITelemetry
  {
  public:
    explicit JsonlTelemetry(JsonlTelemetryConfig cfg);
    ~JsonlTelemetry() override;

    JsonlTelemetry(const JsonlTelemetry&)            = delete;
    JsonlTelemetry& operator=(const JsonlTelemetry&) = delete;
    JsonlTelemetry(JsonlTelemetry&&)                 = delete;
    JsonlTelemetry& operator=(JsonlTelemetry&&)      = delete;

    bool timing_enabled() const noexcept override;
    void emit_producer(const ProducerSample& sample) noexcept override;
    void emit_consumer(const ConsumerSample& sample) noexcept override;

  private:
    struct Impl;
    Impl* _impl{nullptr};
  };
} // namespace omniseer::vision
