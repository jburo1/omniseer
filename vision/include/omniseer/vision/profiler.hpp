// include/omniseer/vision/profiler.hpp
#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace omniseer::vision
{

  enum class Stage : uint8_t
  {
    Capture,
    Preprocess,
    Infer,
    Total,
    Count
  };

  struct Sample
  {
    uint64_t frame_id        = 0;
    uint64_t t_capture_ns    = 0;
    uint64_t t_preprocess_ns = 0;
    uint64_t t_infer_ns      = 0;
    uint64_t t_total_ns      = 0;
    uint64_t timestamp_ns    = 0;
  };

  class ProfilerSink
  {
  public:
    virtual ~ProfilerSink()                                    = default;
    virtual void write_batch(const std::vector<Sample>& batch) = 0;
  };

  class Profiler
  {
  public:
    void begin(Stage s);
    void end(Stage s);

    // Call once per frame after Total ends.
    Sample finalize(uint64_t frame_id, uint64_t timestamp_ns = 0);

    // Optional: attach a sink (CSV, JSONL). Profiler buffers and flushes in batches.
    void set_sink(ProfilerSink* sink, size_t batch_size = 256);
    void flush();

  private:
    std::array<uint64_t, static_cast<size_t>(Stage::Count)> t0_{};
    std::array<uint64_t, static_cast<size_t>(Stage::Count)> acc_{};

    ProfilerSink*       sink_       = nullptr;
    size_t              batch_size_ = 256;
    std::vector<Sample> batch_;

    static uint64_t now_ns_();
  };

} // namespace omniseer::vision
