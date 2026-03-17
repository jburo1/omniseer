#pragma once

#include <initializer_list>
#include <utility>
#include <vector>

#include "omniseer/vision/telemetry.hpp"

namespace omniseer::vision
{
  class CompositeTelemetry final : public ITelemetry
  {
  public:
    CompositeTelemetry() = default;

    explicit CompositeTelemetry(std::initializer_list<ITelemetry*> sinks)
        : _sinks(sinks)
    {
    }

    explicit CompositeTelemetry(std::vector<ITelemetry*> sinks) : _sinks(std::move(sinks))
    {
    }

    bool timing_enabled() const noexcept override
    {
      for (ITelemetry* sink : _sinks)
      {
        if (sink != nullptr && sink->timing_enabled())
          return true;
      }
      return false;
    }

    void emit_producer(const ProducerSample& sample) noexcept override
    {
      for (ITelemetry* sink : _sinks)
      {
        if (sink != nullptr && sink->timing_enabled())
          sink->emit_producer(sample);
      }
    }

    void emit_consumer(const ConsumerSample& sample) noexcept override
    {
      for (ITelemetry* sink : _sinks)
      {
        if (sink != nullptr && sink->timing_enabled())
          sink->emit_consumer(sample);
      }
    }

  private:
    std::vector<ITelemetry*> _sinks{};
  };
} // namespace omniseer::vision
