#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "omniseer/vision/detections.hpp"

namespace omniseer::vision
{
  /** @brief Startup configuration for one production-path offline detector instance. */
  struct OfflineDetectorConfig
  {
    std::string detector_model_path{};
    std::string class_list_path{};
    std::string clip_model_path{};
    std::string clip_vocab_path{};
    std::string pad_token{"nothing"};
    uint32_t    warmup_runs{0};
    float       score_threshold{0.25F};
    float       nms_iou_threshold{0.45F};
    uint32_t    max_detections{100};
    int         source_width{1280};
    int         source_height{720};
  };

  /**
   * @brief One reusable decoded-frame boundary over the production RKNN pipeline.
   *
   * The caller owns the BGR source image.  This facade only creates the normal
   * 640x640 DMA-backed model input and invokes the existing ConsumerPipeline.
   */
  class OfflineDetector
  {
  public:
    explicit OfflineDetector(OfflineDetectorConfig config);
    ~OfflineDetector();

    OfflineDetector(const OfflineDetector&)            = delete;
    OfflineDetector& operator=(const OfflineDetector&) = delete;
    OfflineDetector(OfflineDetector&&)                 = delete;
    OfflineDetector& operator=(OfflineDetector&&)      = delete;

    const std::vector<std::string>& class_names() const noexcept;
    DetectionsFrame infer(const cv::Mat& corrected_bgr_frame, uint64_t frame_index);

  private:
    struct Impl;
    std::unique_ptr<Impl> _impl;
  };
} // namespace omniseer::vision
