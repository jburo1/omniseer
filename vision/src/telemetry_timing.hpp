#pragma once

#include <chrono>
#include <cstdint>

namespace omniseer::vision::telemetry_timing
{
  using clock = std::chrono::steady_clock;
  using real_clock = std::chrono::system_clock;

  inline uint64_t elapsed_ns(clock::time_point start, clock::time_point end) noexcept
  {
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    return (ns > 0) ? static_cast<uint64_t>(ns) : 0u;
  }

  inline uint64_t now_real_ns() noexcept
  {
    const auto ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(real_clock::now().time_since_epoch())
            .count();
    return (ns > 0) ? static_cast<uint64_t>(ns) : 0u;
  }

  class ScopedStageTimer
  {
  public:
    ScopedStageTimer(bool enabled, uint64_t& out) noexcept
        : _enabled(enabled), _out(&out), _start(enabled ? clock::now() : clock::time_point{})
    {
    }

    ~ScopedStageTimer() noexcept
    {
      if (_enabled)
        *_out = elapsed_ns(_start, clock::now());
    }

    ScopedStageTimer(const ScopedStageTimer&)            = delete;
    ScopedStageTimer& operator=(const ScopedStageTimer&) = delete;

  private:
    bool              _enabled{false};
    uint64_t*         _out{nullptr};
    clock::time_point _start{};
  };
} // namespace omniseer::vision::telemetry_timing
