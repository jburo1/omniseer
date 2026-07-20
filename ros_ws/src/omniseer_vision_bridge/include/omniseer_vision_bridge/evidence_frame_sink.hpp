#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "omniseer/vision/frame_preview_sink.hpp"

namespace omniseer_vision_bridge
{
struct EvidenceFrameSinkConfig
{
  std::string              evidence_dir{};
  std::vector<std::string> class_names{};
  double                   interval_sec{1.0};
  int64_t                  jpeg_quality{85};
  int64_t                  storage_budget_mb{1024};
  int64_t                  min_free_mb{256};
  std::size_t              queue_capacity{4};
};

struct EvidenceFrameSinkSnapshot
{
  uint64_t enqueued_count{0};
  uint64_t written_count{0};
  uint64_t dropped_count{0};
  bool     stopped{false};
  std::string stopped_reason{};
};

class EvidenceFrameSink final : public omniseer::vision::IFramePreviewSink
{
public:
  explicit EvidenceFrameSink(EvidenceFrameSinkConfig config);
  ~EvidenceFrameSink() override;

  EvidenceFrameSink(const EvidenceFrameSink &) = delete;
  EvidenceFrameSink & operator=(const EvidenceFrameSink &) = delete;
  EvidenceFrameSink(EvidenceFrameSink &&) = delete;
  EvidenceFrameSink & operator=(EvidenceFrameSink &&) = delete;

  void publish(
    const omniseer::vision::ImageBuffer & image,
    const omniseer::vision::DetectionsFrame & detections,
    const omniseer::vision::PipelineRemapConfig & remap) noexcept override;

  EvidenceFrameSinkSnapshot snapshot() const;

private:
  struct Impl;
  std::unique_ptr<Impl> _impl;
};
} // namespace omniseer_vision_bridge
