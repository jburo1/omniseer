#include <fstream>
#include <gtest/gtest.h>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include "omniseer/vision/consumer_pipeline.hpp"
#include "omniseer/vision/jsonl_telemetry.hpp"
#include "omniseer/vision/producer_pipeline.hpp"
#include "omniseer/vision/types.hpp"
#include "omniseer/vision/v4l2_capture.hpp"

namespace
{
  std::string make_temp_path()
  {
    char pattern[] = "/tmp/omniseer_jsonl_telemetry_XXXXXX";
    const int fd   = ::mkstemp(pattern);
    if (fd < 0)
      throw std::runtime_error("mkstemp failed");
    ::close(fd);
    return pattern;
  }
} // namespace

TEST(JsonlTelemetry, WritesProducerAndConsumerJsonl)
{
  const std::string path = make_temp_path();

  {
    omniseer::vision::JsonlTelemetry telemetry({
        .path                    = path,
        .producer_queue_capacity = 8,
        .consumer_queue_capacity = 8,
    });

    omniseer::vision::ProducerSample producer{};
    producer.frame_id          = 42;
    producer.tick_id           = 1;
    producer.has_frame_id      = 1;
    producer.sequence          = 9;
    producer.has_sequence      = 1;
    producer.event_ts_real_ns  = 1000;
    producer.source_age_dequeue_ns = 25;
    producer.source_age_publish_ready_ns = 37;
    producer.dequeue_ns        = 10;
    producer.acquire_write_ns  = 11;
    producer.preprocess_ns     = 12;
    producer.publish_ready_ns  = 13;
    producer.requeue_ns        = 14;
    producer.total_ns          = 60;
    producer.stage_mask        = static_cast<uint32_t>(omniseer::vision::ProducerStageMask::Dequeue) |
                          static_cast<uint32_t>(omniseer::vision::ProducerStageMask::AcquireWrite) |
                          static_cast<uint32_t>(omniseer::vision::ProducerStageMask::Preprocess) |
                          static_cast<uint32_t>(omniseer::vision::ProducerStageMask::PublishReady) |
                          static_cast<uint32_t>(omniseer::vision::ProducerStageMask::Requeue);
    producer.capture_errno     = 0;
    producer.producer_status   =
        static_cast<uint8_t>(omniseer::vision::ProducerTickStatus::Produced);
    producer.capture_status    = static_cast<uint8_t>(omniseer::vision::CaptureStatus::Ok);
    producer.preprocess_status = static_cast<uint8_t>(omniseer::vision::PreprocessStatus::Ok);

    omniseer::vision::ConsumerSample consumer{};
    consumer.frame_id         = 42;
    consumer.tick_id          = 2;
    consumer.has_frame_id     = 1;
    consumer.sequence         = 9;
    consumer.has_sequence     = 1;
    consumer.event_ts_real_ns = 1001;
    consumer.consumer_start_ts_real_ns = 1100;
    consumer.consumer_end_ts_real_ns   = 1200;
    consumer.source_age_start_ns       = 99;
    consumer.source_age_end_ns         = 199;
    consumer.acquire_read_ns  = 20;
    consumer.infer_ns         = 21;
    consumer.postprocess_ns   = 22;
    consumer.publish_ns       = 23;
    consumer.release_ns       = 24;
    consumer.total_ns         = 110;
    consumer.stage_mask       = static_cast<uint32_t>(omniseer::vision::ConsumerStageMask::AcquireRead) |
                          static_cast<uint32_t>(omniseer::vision::ConsumerStageMask::Infer) |
                          static_cast<uint32_t>(omniseer::vision::ConsumerStageMask::Postprocess) |
                          static_cast<uint32_t>(omniseer::vision::ConsumerStageMask::Publish) |
                          static_cast<uint32_t>(omniseer::vision::ConsumerStageMask::Release);
    consumer.infer_errno      = 0;
    consumer.consumer_status =
        static_cast<uint8_t>(omniseer::vision::ConsumerTickStatus::Consumed);
    consumer.infer_status = static_cast<uint8_t>(omniseer::vision::InferStatus::Ok);
    consumer.postprocess_status =
        static_cast<uint8_t>(omniseer::vision::PostprocessStatus::Ok);

    telemetry.emit_producer(producer);
    telemetry.emit_consumer(consumer);
  }

  std::ifstream ifs(path);
  ASSERT_TRUE(ifs.is_open());
  const std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  EXPECT_NE(content.find("\"source\":\"producer\""), std::string::npos);
  EXPECT_NE(content.find("\"source\":\"consumer\""), std::string::npos);
  EXPECT_NE(content.find("\"schema_version\":3"), std::string::npos);
  EXPECT_NE(content.find("\"producer_status\":\"produced\""), std::string::npos);
  EXPECT_NE(content.find("\"consumer_status\":\"consumed\""), std::string::npos);
  EXPECT_NE(content.find("\"source_age_dequeue_ns\":25"), std::string::npos);
  EXPECT_NE(content.find("\"source_age_publish_ready_ns\":37"), std::string::npos);
  EXPECT_NE(content.find("\"postprocess_status\":\"ok\""), std::string::npos);
  EXPECT_NE(content.find("\"consumer_start_ts_real_ns\":1100"), std::string::npos);
  EXPECT_NE(content.find("\"consumer_end_ts_real_ns\":1200"), std::string::npos);
  EXPECT_NE(content.find("\"source_age_end_ns\":199"), std::string::npos);
  EXPECT_NE(content.find("\"frame_id\":42"), std::string::npos);
  EXPECT_NE(content.find("\"sequence\":9"), std::string::npos);

  ::unlink(path.c_str());
}
