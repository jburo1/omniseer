#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <gtest/gtest.h>
#include <linux/videodev2.h>
#include <string>
#include <thread>
#include <unistd.h>

#include "omniseer/vision/class_list.hpp"
#include "omniseer/vision/consumer_pipeline.hpp"
#include "omniseer/vision/detections.hpp"
#include "omniseer/vision/detections_sink.hpp"
#include "omniseer/vision/dma_heap_alloc.hpp"
#include "omniseer/vision/image_buffer_pool.hpp"
#include "omniseer/vision/producer_pipeline.hpp"
#include "omniseer/vision/rga_preprocess.hpp"
#include "omniseer/vision/rknn_runner.hpp"
#include "omniseer/vision/v4l2_capture.hpp"
#include "omniseer/vision/yolo_world_text_embeddings.hpp"

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

  std::string source_path(const char* relpath)
  {
    return std::string(VISION_SOURCE_DIR) + relpath;
  }

  class TestSink final : public omniseer::vision::IDetectionsSink
  {
  public:
    void publish(const omniseer::vision::DetectionsFrame& frame) noexcept override
    {
      last_frame = frame;
      publish_count += 1;
    }

    omniseer::vision::DetectionsFrame last_frame{};
    uint32_t                          publish_count{0};
  };
} // namespace

TEST(VisionPipelineIntegration, ProducesAndConsumesFramesWithShortClassList)
{
  const char*       dev_env = std::getenv("VISION_V4L2_DEV");
  const std::string dev     = (dev_env && *dev_env) ? dev_env : "/dev/video12";

  if (::access(dev.c_str(), R_OK | W_OK) != 0)
    GTEST_SKIP() << "no access to " << dev << " (" << std::strerror(errno) << ")";

  if (::access("/dev/rga", R_OK | W_OK) != 0 && ::access("/dev/rga0", R_OK | W_OK) != 0)
    GTEST_SKIP() << "no access to /dev/rga(/dev/rga0) (" << std::strerror(errno) << ")";

  const std::string clip_model = source_path("/testdata/text_embeddings/clip_text_fp16.rknn");
  const std::string clip_vocab = source_path("/testdata/text_embeddings/clip_vocab.bpe");
  const std::string yolo_model = source_path("/testdata/rknn_runner/yolo_world_v2s_i8.rknn");
  const std::string class_list = source_path("/testdata/text_embeddings/classes_person_bus.txt");

  if (::access(clip_model.c_str(), R_OK) != 0)
    GTEST_SKIP() << "missing CLIP model asset: " << clip_model;
  if (::access(clip_vocab.c_str(), R_OK) != 0)
    GTEST_SKIP() << "missing CLIP vocab asset: " << clip_vocab;
  if (::access(yolo_model.c_str(), R_OK) != 0)
    GTEST_SKIP() << "missing YOLO model asset: " << yolo_model;
  if (::access(class_list.c_str(), R_OK) != 0)
    GTEST_SKIP() << "missing class list asset: " << class_list;

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

  omniseer::vision::ImageBufferPool pool{};
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

  omniseer::vision::ProducerPipeline producer(capture, preprocess, pool);
  ASSERT_NO_THROW(producer.preflight());

  omniseer::vision::YoloWorldTextEmbeddingsBuilder builder({
      .text_encoder_model_path = clip_model,
      .detector_model_path     = yolo_model,
      .clip_vocab_path         = clip_vocab,
      .pad_token               = "nothing",
  });
  const std::vector<std::string> class_names = omniseer::vision::load_class_list_file(class_list);
  ASSERT_EQ(class_names.size(), 2u);
  const omniseer::vision::PreparedTextEmbeddings prepared = builder.build(class_names);

  TestSink sink{};
  omniseer::vision::RknnRunner runner({
      .model_path  = yolo_model,
      .warmup_runs = 0,
  });
  omniseer::vision::ConsumerPipeline consumer(pool, runner, nullptr, &sink);
  ASSERT_NO_THROW(consumer.preflight({
      .remap           = producer.remap(),
      .text_embeddings = prepared.view(),
  }));

  int      produced      = 0;
  int      consumed      = 0;
  uint64_t last_frame_id = 0;
  constexpr int max_ticks = 400;

  for (int i = 0; i < max_ticks && (produced == 0 || consumed == 0); ++i)
  {
    const auto producer_tick = producer.run();
    switch (producer_tick.status)
    {
    case omniseer::vision::ProducerTickStatus::Produced:
      produced += 1;
      EXPECT_GT(producer_tick.frame_id, last_frame_id);
      last_frame_id = producer_tick.frame_id;
      break;
    case omniseer::vision::ProducerTickStatus::NoFrame:
    case omniseer::vision::ProducerTickStatus::CaptureRetryableError:
    case omniseer::vision::ProducerTickStatus::NoWritableBuffer:
      break;
    case omniseer::vision::ProducerTickStatus::CaptureFatalError:
    case omniseer::vision::ProducerTickStatus::PreprocessError:
      FAIL() << "producer failure: status=" << static_cast<int>(producer_tick.status)
             << " errno=" << producer_tick.stage_errno;
    }

    const auto consumer_tick = consumer.run();
    switch (consumer_tick.status)
    {
    case omniseer::vision::ConsumerTickStatus::Consumed:
      consumed += 1;
      EXPECT_EQ(consumer_tick.stage, omniseer::vision::ConsumerStage::Release);
      EXPECT_GE(consumer_tick.pool_index, 0);
      break;
    case omniseer::vision::ConsumerTickStatus::NoReadyBuffer:
      break;
    case omniseer::vision::ConsumerTickStatus::InferError:
      FAIL() << "consumer infer error: errno=" << consumer_tick.stage_errno;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  EXPECT_GT(produced, 0) << "producer did not produce any frames";
  EXPECT_GT(consumed, 0) << "consumer did not consume any frames";
  EXPECT_GT(sink.publish_count, 0u) << "consumer did not publish any detections frames";
  EXPECT_EQ(sink.last_frame.active_class_count, 2u);
  EXPECT_EQ(sink.last_frame.source_size.w, static_cast<int>(src_w));
  EXPECT_EQ(sink.last_frame.source_size.h, static_cast<int>(src_h));
  EXPECT_LE(sink.last_frame.count, omniseer::vision::DetectionsFrame::capacity);
  for (uint32_t i = 0; i < sink.last_frame.count; ++i)
  {
    const auto& det = sink.last_frame.detections[i];
    EXPECT_LT(det.class_id, sink.last_frame.active_class_count);
    EXPECT_GE(det.x1, 0.0F);
    EXPECT_GE(det.y1, 0.0F);
    EXPECT_LE(det.x2, static_cast<float>(sink.last_frame.source_size.w));
    EXPECT_LE(det.y2, static_cast<float>(sink.last_frame.source_size.h));
    EXPECT_LT(det.x1, det.x2);
    EXPECT_LT(det.y1, det.y2);
  }

  ASSERT_NO_THROW(capture.stop());
}
