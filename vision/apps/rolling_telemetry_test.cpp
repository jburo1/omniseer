#include <gtest/gtest.h>

#include "omniseer/vision/composite_telemetry.hpp"
#include "omniseer/vision/rolling_telemetry.hpp"

TEST(RollingTelemetryStats, TracksCountsAndDurations)
{
  omniseer::vision::RollingTelemetryStats stats{};

  omniseer::vision::ProducerSample producer{};
  producer.producer_status = static_cast<uint8_t>(omniseer::vision::ProducerTickStatus::Produced);
  producer.preprocess_ns   = 11;
  producer.total_ns        = 33;

  omniseer::vision::ConsumerSample consumer{};
  consumer.consumer_status = static_cast<uint8_t>(omniseer::vision::ConsumerTickStatus::Consumed);
  consumer.infer_ns        = 22;
  consumer.postprocess_ns  = 7;
  consumer.total_ns        = 44;

  stats.emit_producer(producer);
  stats.emit_consumer(consumer);

  const auto snapshot = stats.snapshot();
  EXPECT_EQ(snapshot.produced_count, 1u);
  EXPECT_EQ(snapshot.consumed_count, 1u);
  EXPECT_EQ(snapshot.last_preprocess_ns, 11u);
  EXPECT_EQ(snapshot.last_producer_total_ns, 33u);
  EXPECT_EQ(snapshot.last_infer_ns, 22u);
  EXPECT_EQ(snapshot.last_postprocess_ns, 7u);
  EXPECT_EQ(snapshot.last_consumer_total_ns, 44u);
}

namespace
{
  class CountingTelemetry final : public omniseer::vision::ITelemetry
  {
  public:
    explicit CountingTelemetry(bool enabled_) : enabled(enabled_)
    {
    }

    bool timing_enabled() const noexcept override
    {
      return enabled;
    }

    void emit_producer(const omniseer::vision::ProducerSample&) noexcept override
    {
      producer_count += 1;
    }

    void emit_consumer(const omniseer::vision::ConsumerSample&) noexcept override
    {
      consumer_count += 1;
    }

    bool enabled{true};
    int  producer_count{0};
    int  consumer_count{0};
  };
} // namespace

TEST(CompositeTelemetry, FansOutToEnabledSinks)
{
  CountingTelemetry disabled(false);
  CountingTelemetry enabled(true);
  omniseer::vision::CompositeTelemetry composite({&disabled, &enabled});

  EXPECT_TRUE(composite.timing_enabled());

  composite.emit_producer(omniseer::vision::ProducerSample{});
  composite.emit_consumer(omniseer::vision::ConsumerSample{});

  EXPECT_EQ(disabled.producer_count, 0);
  EXPECT_EQ(disabled.consumer_count, 0);
  EXPECT_EQ(enabled.producer_count, 1);
  EXPECT_EQ(enabled.consumer_count, 1);
}
