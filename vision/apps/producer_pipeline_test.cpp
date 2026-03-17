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
#include "omniseer/vision/image_buffer_pool.hpp"
#include "omniseer/vision/pipeline.hpp"
#include "omniseer/vision/rga_preprocess.hpp"
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

  ASSERT_NO_THROW(producer.preflight());
  EXPECT_TRUE(producer.is_armed());

  const auto& remap = producer.remap();
  EXPECT_EQ(remap.source_size.w, static_cast<int>(src_w));
  EXPECT_EQ(remap.source_size.h, static_cast<int>(src_h));
  EXPECT_EQ(remap.model_input_size.w, dst_w);
  EXPECT_EQ(remap.model_input_size.h, dst_h);
  EXPECT_GT(remap.scale, 0.0f);
  EXPECT_GE(remap.pad_x, 0);
  EXPECT_GE(remap.pad_y, 0);
  EXPECT_GT(remap.resized_w, 0);
  EXPECT_GT(remap.resized_h, 0);

  int           produced      = 0;
  uint64_t      last_frame_id = 0;
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
      ASSERT_LT(emitted_before, telemetry.producer_samples.size());
      EXPECT_EQ(telemetry.producer_samples[emitted_before].producer_status,
                static_cast<uint8_t>(tick.status));
    }

    if (tick.status == omniseer::vision::ProducerTickStatus::Produced)
    {
      ++produced;
      EXPECT_EQ(tick.stage, omniseer::vision::ProducerStage::Requeue);
      EXPECT_EQ(tick.stage_errno, 0);
      EXPECT_GT(tick.capture_ts_real_ns, 0u);
      EXPECT_GE(tick.pool_index, 0);
      EXPECT_GT(tick.frame_id, last_frame_id);
      last_frame_id = tick.frame_id;

      const uint32_t expected_mask =
          stage_mask_bit(omniseer::vision::ProducerStageMask::Dequeue) |
          stage_mask_bit(omniseer::vision::ProducerStageMask::AcquireWrite) |
          stage_mask_bit(omniseer::vision::ProducerStageMask::Preprocess) |
          stage_mask_bit(omniseer::vision::ProducerStageMask::PublishReady) |
          stage_mask_bit(omniseer::vision::ProducerStageMask::Requeue);
      EXPECT_EQ((tick.stage_mask & expected_mask), expected_mask);
      const auto& sample = telemetry.producer_samples[emitted_before];
      EXPECT_GT(sample.dequeue_ns, 0u);
      EXPECT_GE(sample.acquire_write_ns, 0u);
      EXPECT_GT(sample.preprocess_ns, 0u);
      EXPECT_GE(sample.publish_ready_ns, 0u);
      EXPECT_GE(sample.requeue_ns, 0u);
      EXPECT_GT(sample.total_ns, 0u);

      auto read_lease = pool.acquire_read_lease();
      ASSERT_TRUE(read_lease.has_value());
      EXPECT_EQ(read_lease->index(), tick.pool_index);
      EXPECT_EQ(read_lease->buffer().sequence, static_cast<uint32_t>(tick.sequence));
      EXPECT_EQ(read_lease->buffer().capture_ts_real_ns, tick.capture_ts_real_ns);
      EXPECT_EQ(read_lease->buffer().frame_id, tick.frame_id);
      continue;
    }

    if (tick.status == omniseer::vision::ProducerTickStatus::NoWritableBuffer)
    {
      EXPECT_EQ(tick.stage, omniseer::vision::ProducerStage::AcquireWrite)
          << "unexpected stage for NoWritableBuffer: " << stage_name(tick.stage);
    }

    if (tick.status == omniseer::vision::ProducerTickStatus::CaptureFatalError ||
        tick.status == omniseer::vision::ProducerTickStatus::PreprocessError)
    {
      FAIL() << "unexpected fatal-ish status: " << tick_status_name(tick.status)
             << ", stage_errno=" << tick.stage_errno;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  EXPECT_GT(produced, 0) << "producer did not produce any frames in " << max_ticks << " ticks";

  ASSERT_NO_THROW(capture.stop());
}

TEST(ProducerPipeline, TelemetryTracksReachedStagesOnNoWritableBuffer)
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
  ASSERT_NO_THROW(producer.preflight());

  std::vector<omniseer::vision::ImageBufferPool::WriteLease> occupied{};
  for (;;)
  {
    auto lease = pool.acquire_write_lease();
    if (!lease)
      break;
    occupied.emplace_back(std::move(*lease));
  }
  ASSERT_EQ(static_cast<int>(occupied.size()), omniseer::vision::ImageBufferPool::capacity());

  constexpr int max_ticks = 200;
  for (int i = 0; i < max_ticks; ++i)
  {
    const std::size_t emitted_before = telemetry.producer_samples.size();
    const auto        tick           = producer.run();

    if (tick.status == omniseer::vision::ProducerTickStatus::NoFrame)
    {
      EXPECT_EQ(telemetry.producer_samples.size(), emitted_before);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    ASSERT_EQ(tick.status, omniseer::vision::ProducerTickStatus::NoWritableBuffer);
    ASSERT_EQ(tick.stage, omniseer::vision::ProducerStage::AcquireWrite);
    ASSERT_EQ(telemetry.producer_samples.size(), emitted_before + 1);
    const auto& sample = telemetry.producer_samples.back();
    EXPECT_EQ(sample.producer_status,
              static_cast<uint8_t>(omniseer::vision::ProducerTickStatus::NoWritableBuffer));
    EXPECT_GT(sample.dequeue_ns, 0u);
    EXPECT_GE(sample.acquire_write_ns, 0u);
    EXPECT_EQ(sample.preprocess_ns, 0u);
    EXPECT_EQ(sample.publish_ready_ns, 0u);
    EXPECT_EQ(sample.requeue_ns, 0u);
    EXPECT_GT(sample.total_ns, 0u);

    ASSERT_NO_THROW(capture.stop());
    return;
  }

  ASSERT_NO_THROW(capture.stop());
  FAIL() << "producer did not reach NoWritableBuffer in " << max_ticks << " ticks";
}
