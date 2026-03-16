#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <gtest/gtest.h>
#include <optional>
#include <rknn_api.h>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

#include "omniseer/vision/config.hpp"
#include "omniseer/vision/consumer_pipeline.hpp"
#include "omniseer/vision/detections.hpp"
#include "omniseer/vision/detections_sink.hpp"
#include "omniseer/vision/dma_heap_alloc.hpp"
#include "omniseer/vision/image_buffer_pool.hpp"
#include "omniseer/vision/rknn_runner.hpp"

namespace
{
  constexpr const char* kModelRelPath = "/testdata/rknn_runner/yolo_world_v2s_i8.rknn";

  std::string make_rknn_error(const char* where, int code)
  {
    return std::string(where) + " failed, code=" + std::to_string(code);
  }

  std::vector<uint8_t> read_file(const std::string& path)
  {
    std::ifstream ifs(path, std::ios::binary | std::ios::ate);
    if (!ifs)
      throw std::runtime_error("failed to open file: " + path);

    const std::ifstream::pos_type end = ifs.tellg();
    if (end <= 0)
      throw std::runtime_error("file is empty: " + path);

    std::vector<uint8_t> data(static_cast<size_t>(end));
    ifs.seekg(0, std::ios::beg);
    if (!ifs.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(data.size())))
      throw std::runtime_error("failed to read file: " + path);
    return data;
  }

  size_t tensor_bytes(const rknn_tensor_attr& attr)
  {
    return (attr.size_with_stride != 0) ? attr.size_with_stride : attr.size;
  }

  std::string model_path()
  {
    return std::string(VISION_SOURCE_DIR) + kModelRelPath;
  }

  std::vector<int8_t> make_zero_text_blob(const std::string& model)
  {
    const std::vector<uint8_t> model_data = read_file(model);

    rknn_context ctx = 0;
    int          rc  = rknn_init(&ctx, const_cast<uint8_t*>(model_data.data()),
                                 static_cast<uint32_t>(model_data.size()), 0, nullptr);
    if (rc != RKNN_SUCC)
      throw std::runtime_error(make_rknn_error("rknn_init", rc));

    std::optional<std::vector<int8_t>> blob{};
    try
    {
      rknn_input_output_num io_num{};
      rc = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
      if (rc != RKNN_SUCC)
        throw std::runtime_error(make_rknn_error("rknn_query(RKNN_QUERY_IN_OUT_NUM)", rc));

      for (uint32_t i = 0; i < io_num.n_input; ++i)
      {
        rknn_tensor_attr attr{};
        attr.index = i;
        rc         = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &attr, sizeof(attr));
        if (rc != RKNN_SUCC)
          throw std::runtime_error(make_rknn_error("rknn_query(RKNN_QUERY_INPUT_ATTR)", rc));
        if (std::strcmp(attr.name, "texts") == 0)
          blob = std::vector<int8_t>(tensor_bytes(attr), 0);
      }
    }
    catch (...)
    {
      (void) rknn_destroy(ctx);
      throw;
    }

    (void) rknn_destroy(ctx);
    if (!blob.has_value())
      throw std::runtime_error("model does not expose a texts input");
    return std::move(*blob);
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

  class ConsumerPipelineSmokeTest : public ::testing::Test
  {
  protected:
    void SetUp() override
    {
      model_path_ = model_path();
      if (::access(model_path_.c_str(), R_OK) != 0)
        GTEST_SKIP() << "missing model asset: " << model_path_;

      text_i8_ = make_zero_text_blob(model_path_);

      try
      {
        omniseer::vision::DmaHeapAllocator allocator;
        pool_.allocate_all(allocator, 640, 640, omniseer::vision::PixelFormat::RGB888);
      }
      catch (const std::exception& e)
      {
        GTEST_SKIP() << e.what();
      }

      omniseer::vision::RknnRunnerConfig runner_cfg{};
      runner_cfg.model_path  = model_path_;
      runner_cfg.warmup_runs = 0;
      runner_.emplace(runner_cfg);
      consumer_.emplace(pool_, *runner_, nullptr, &sink_);

      omniseer::vision::ConsumerPipelineStartup startup{};
      startup.text_embeddings = {
          .data               = text_i8_.data(),
          .bytes              = text_i8_.size(),
          .active_class_count = 1,
      };
      ASSERT_NO_THROW(consumer_->preflight(startup));
    }

    std::string                                    model_path_{};
    omniseer::vision::ImageBufferPool              pool_{};
    std::vector<int8_t>                            text_i8_{};
    std::optional<omniseer::vision::RknnRunner>    runner_{};
    TestSink                                       sink_{};
    std::optional<omniseer::vision::ConsumerPipeline> consumer_{};
  };
} // namespace

TEST_F(ConsumerPipelineSmokeTest, RunConsumesReadyBuffer)
{
  auto write_lease = pool_.acquire_write_lease();
  ASSERT_TRUE(write_lease.has_value());

  write_lease->buffer().sequence           = 42;
  write_lease->buffer().capture_ts_real_ns = 123456789ULL;
  write_lease->buffer().frame_id           = 7;
  const int published_index                = write_lease->index();
  write_lease->publish();

  const auto tick = consumer_->run();
  EXPECT_EQ(tick.status, omniseer::vision::ConsumerTickStatus::Consumed);
  EXPECT_EQ(tick.stage, omniseer::vision::ConsumerStage::Release);
  EXPECT_EQ(tick.stage_errno, 0);
  EXPECT_EQ(tick.sequence, 42u);
  EXPECT_EQ(tick.capture_ts_real_ns, 123456789ULL);
  EXPECT_EQ(tick.frame_id, 7u);
  EXPECT_EQ(tick.pool_index, published_index);
  EXPECT_EQ(sink_.publish_count, 1u);
  EXPECT_EQ(sink_.last_frame.frame_id, tick.frame_id);
  EXPECT_EQ(sink_.last_frame.sequence, tick.sequence);
  EXPECT_EQ(sink_.last_frame.capture_ts_real_ns, tick.capture_ts_real_ns);
  EXPECT_EQ(sink_.last_frame.active_class_count, 1u);
  EXPECT_EQ(sink_.last_frame.source_size.w, 1280);
  EXPECT_EQ(sink_.last_frame.source_size.h, 720);
  EXPECT_LE(sink_.last_frame.count, omniseer::vision::DetectionsFrame::capacity);
  for (uint32_t i = 0; i < sink_.last_frame.count; ++i)
  {
    const auto& det = sink_.last_frame.detections[i];
    EXPECT_LT(det.class_id, sink_.last_frame.active_class_count);
    EXPECT_GE(det.score, 0.0F);
    EXPECT_GE(det.x1, 0.0F);
    EXPECT_GE(det.y1, 0.0F);
    EXPECT_LE(det.x2, static_cast<float>(sink_.last_frame.source_size.w));
    EXPECT_LE(det.y2, static_cast<float>(sink_.last_frame.source_size.h));
    EXPECT_LT(det.x1, det.x2);
    EXPECT_LT(det.y1, det.y2);
  }

  EXPECT_FALSE(pool_.acquire_read_lease().has_value());

  std::vector<int> free_indices{};
  for (;;)
  {
    int idx = -1;
    if (!pool_.acquire_write(idx))
      break;
    free_indices.push_back(idx);
  }

  EXPECT_EQ(static_cast<int>(free_indices.size()), omniseer::vision::ImageBufferPool::capacity());
  EXPECT_NE(std::find(free_indices.begin(), free_indices.end(), published_index), free_indices.end());
}

TEST_F(ConsumerPipelineSmokeTest, RunReturnsNoReadyBufferWhenPoolEmpty)
{
  const auto tick = consumer_->run();
  EXPECT_EQ(tick.status, omniseer::vision::ConsumerTickStatus::NoReadyBuffer);
  EXPECT_EQ(tick.stage, omniseer::vision::ConsumerStage::AcquireRead);
  EXPECT_EQ(tick.stage_errno, 0);
  EXPECT_EQ(tick.stage_mask, 0u);
  EXPECT_EQ(tick.pool_index, -1);
  EXPECT_EQ(sink_.publish_count, 0u);
}

TEST_F(ConsumerPipelineSmokeTest, PreflightRejectsActiveClassCountAboveModelCapacity)
{
  omniseer::vision::RknnRunnerConfig runner_cfg{};
  runner_cfg.model_path  = model_path_;
  runner_cfg.warmup_runs = 0;
  omniseer::vision::RknnRunner local_runner(runner_cfg);
  omniseer::vision::ConsumerPipeline local_consumer(pool_, local_runner);

  omniseer::vision::ConsumerPipelineStartup startup{};
  startup.text_embeddings = {
      .data               = text_i8_.data(),
      .bytes              = text_i8_.size(),
      .active_class_count = 81,
  };

  EXPECT_THROW(local_consumer.preflight(startup), std::invalid_argument);
}
