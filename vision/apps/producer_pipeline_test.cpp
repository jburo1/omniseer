#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <gtest/gtest.h>
#include <linux/videodev2.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include "omniseer/vision/dma_heap_alloc.hpp"
#include "omniseer/vision/pipeline.hpp"
#include "omniseer/vision/telemetry.hpp"

namespace
{
  uint32_t env_u32(const char* key, uint32_t def)
  {
    const char* v = std::getenv(key);
    if (!v || !*v)
      return def;
    char*         end = nullptr;
    unsigned long n   = std::strtoul(v, &end, 10);
    if (end && *end)
      return def;
    if (n > 0xffffffffUL)
      return def;
    return static_cast<uint32_t>(n);
  }

  const char* tick_status_name(omniseer::vision::ProducerTickStatus status)
  {
    using omniseer::vision::ProducerTickStatus;
    switch (status)
    {
    case ProducerTickStatus::Produced:
      return "produced";
    case ProducerTickStatus::NoFrame:
      return "no-frame";
    case ProducerTickStatus::CaptureRetryableError:
      return "capture-retryable-error";
    case ProducerTickStatus::CaptureFatalError:
      return "capture-fatal-error";
    case ProducerTickStatus::NoWritableBuffer:
      return "no-writable-buffer";
    case ProducerTickStatus::PreprocessError:
      return "preprocess-error";
    }
    return "unknown";
  }

  const char* stage_name(omniseer::vision::ProducerStage stage)
  {
    using omniseer::vision::ProducerStage;
    switch (stage)
    {
    case ProducerStage::None:
      return "none";
    case ProducerStage::Preconditions:
      return "preconditions";
    case ProducerStage::Dequeue:
      return "dequeue";
    case ProducerStage::AcquireWrite:
      return "acquire-write";
    case ProducerStage::Preprocess:
      return "preprocess";
    case ProducerStage::PublishReady:
      return "publish-ready";
    case ProducerStage::Requeue:
      return "requeue";
    }
    return "unknown";
  }

  uint32_t stage_mask_bit(omniseer::vision::ProducerStageMask bit)
  {
    return static_cast<uint32_t>(bit);
  }

  class TestTelemetry final : public omniseer::vision::ITelemetry
  {
  public:
    bool timing_enabled() const noexcept override
    {
      return enabled;
    }

    void emit_producer(const omniseer::vision::ProducerSample& sample) noexcept override
    {
      producer_samples.push_back(sample);
    }

    void emit_consumer(const omniseer::vision::ConsumerSample&) noexcept override
    {
    }

    bool                                       enabled{true};
    std::vector<omniseer::vision::ProducerSample> producer_samples{};
  };
} // namespace

TEST(ProducerPipeline, NotArmedReportsPreconditionsAndTelemetry)
{
  omniseer::vision::V4l2Capture capture({
      .device       = "/dev/video12",
      .width        = 1280,
      .height       = 720,
      .fourcc       = V4L2_PIX_FMT_NV12,
      .buffer_count = 4,
  });
  omniseer::vision::ImageBufferPool pool;
  omniseer::vision::RgaPreprocess preprocess;
  TestTelemetry                   telemetry;

  omniseer::vision::ProducerPipeline producer(capture, preprocess, pool, &telemetry);

  const auto tick = producer.run();
  EXPECT_EQ(tick.status, omniseer::vision::ProducerTickStatus::CaptureFatalError);
  EXPECT_EQ(tick.stage, omniseer::vision::ProducerStage::Preconditions);
  EXPECT_EQ(tick.stage_errno, EPERM);
  EXPECT_EQ(tick.stage_mask, 0u);

  ASSERT_EQ(telemetry.producer_samples.size(), 1u);
  EXPECT_EQ(telemetry.producer_samples[0].producer_status,
            static_cast<uint8_t>(omniseer::vision::ProducerTickStatus::CaptureFatalError));

  telemetry.enabled = false;
  (void) producer.run();
  EXPECT_EQ(telemetry.producer_samples.size(), 1u);
}

TEST(ProducerPipeline, PreflightThenProducesFrames)
{
  const char*       dev_env = std::getenv("VISION_V4L2_DEV");
  const std::string dev     = (dev_env && *dev_env) ? dev_env : "/dev/video12";

  if (::access(dev.c_str(), R_OK | W_OK) != 0)
    GTEST_SKIP() << "no access to " << dev << " (" << std::strerror(errno) << ")";

  if (::access("/dev/rga", R_OK | W_OK) != 0 && ::access("/dev/rga0", R_OK | W_OK) != 0)
    GTEST_SKIP() << "no access to /dev/rga(/dev/rga0) (" << std::strerror(errno) << ")";

  const uint32_t src_w   = env_u32("VISION_V4L2_W", 1280);
  const uint32_t src_h   = env_u32("VISION_V4L2_H", 720);
  const uint32_t buffers = env_u32("VISION_V4L2_BUFFERS", 4);
  const int      dst_w   = static_cast<int>(env_u32("VISION_DST_W", 640));
  const int      dst_h   = static_cast<int>(env_u32("VISION_DST_H", 640));

  omniseer::vision::V4l2Capture capture({
      .device       = dev,
      .width        = src_w,
      .height       = src_h,
      .fourcc       = V4L2_PIX_FMT_NV12,
      .buffer_count = buffers,
  });
  ASSERT_NO_THROW(capture.start());

  omniseer::vision::ImageBufferPool pool;
  try
  {
    omniseer::vision::DmaHeapAllocator allocator;
    pool.allocate_all(allocator, dst_w, dst_h, omniseer::vision::PixelFormat::RGB888);
  }
  catch (const std::exception& e)
  {
    GTEST_SKIP() << e.what();
  }

  omniseer::vision::RgaPreprocess preprocess({
      .src_w     = static_cast<int>(src_w),
      .src_h     = static_cast<int>(src_h),
      .dst_w     = dst_w,
      .dst_h     = dst_h,
      .pad_value = 114,
  });

  {
    std::vector<omniseer::vision::ImageBufferPool::WriteLease> init_leases{};
    for (;;)
    {
      auto lease = pool.acquire_write_lease();
      if (!lease)
        break;
      ASSERT_NO_THROW(preprocess.prefill(lease->buffer()));
      init_leases.emplace_back(std::move(*lease));
    }
    ASSERT_GT(init_leases.size(), 0u);
  }

  TestTelemetry telemetry;
  omniseer::vision::ProducerPipeline producer(capture, preprocess, pool, &telemetry);

  const auto pre_preflight = producer.run();
  EXPECT_EQ(pre_preflight.status, omniseer::vision::ProducerTickStatus::CaptureFatalError);
  EXPECT_EQ(pre_preflight.stage, omniseer::vision::ProducerStage::Preconditions);
  EXPECT_EQ(pre_preflight.stage_errno, EPERM);
  EXPECT_EQ(pre_preflight.stage_mask, 0u);
  EXPECT_EQ(pre_preflight.capture.status, omniseer::vision::CaptureStatus::FatalError);
  EXPECT_EQ(pre_preflight.capture.sys_errno, EPERM);
  ASSERT_EQ(telemetry.producer_samples.size(), 1u);

  ASSERT_NO_THROW(producer.preflight());

  int           produced  = 0;
  int           nonfatal  = 0;
  constexpr int max_ticks = 400;
  for (int i = 0; i < max_ticks && produced < 3; ++i)
  {
    const std::size_t emitted_before = telemetry.producer_samples.size();
    const auto tick = producer.run();

    if (tick.status == omniseer::vision::ProducerTickStatus::NoFrame)
    {
      EXPECT_EQ(tick.stage, omniseer::vision::ProducerStage::Dequeue);
      EXPECT_EQ(telemetry.producer_samples.size(), emitted_before);
    }
    else
    {
      EXPECT_EQ(telemetry.producer_samples.size(), emitted_before + 1);
    }

    if (tick.status == omniseer::vision::ProducerTickStatus::Produced)
    {
      ++produced;
      EXPECT_TRUE(tick.preprocess.ok());
      EXPECT_EQ(tick.capture.status, omniseer::vision::CaptureStatus::Ok);
      EXPECT_EQ(tick.stage, omniseer::vision::ProducerStage::Requeue);

      const uint32_t expected_mask =
          stage_mask_bit(omniseer::vision::ProducerStageMask::Dequeue) |
          stage_mask_bit(omniseer::vision::ProducerStageMask::AcquireWrite) |
          stage_mask_bit(omniseer::vision::ProducerStageMask::Preprocess) |
          stage_mask_bit(omniseer::vision::ProducerStageMask::PublishReady) |
          stage_mask_bit(omniseer::vision::ProducerStageMask::Requeue);
      EXPECT_EQ((tick.stage_mask & expected_mask), expected_mask);
      continue;
    }

    if (tick.status == omniseer::vision::ProducerTickStatus::NoWritableBuffer)
    {
      EXPECT_TRUE(tick.stage == omniseer::vision::ProducerStage::AcquireWrite ||
                  tick.stage == omniseer::vision::ProducerStage::Requeue)
          << "unexpected stage for NoWritableBuffer: " << stage_name(tick.stage);
    }

    if (tick.status == omniseer::vision::ProducerTickStatus::CaptureFatalError ||
        tick.status == omniseer::vision::ProducerTickStatus::PreprocessError)
    {
      FAIL() << "unexpected fatal-ish status: " << tick_status_name(tick.status)
             << ", capture_errno=" << tick.capture.sys_errno;
    }

    ++nonfatal;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  EXPECT_GT(produced, 0) << "producer did not produce any frames in " << max_ticks << " ticks";
  EXPECT_GE(nonfatal, 0);

  ASSERT_NO_THROW(capture.stop());
}
