#pragma once

#include <atomic>
#include <thread>

namespace omniseer::vision
{
  class ProducerPipeline;
  class ConsumerPipeline;

  class VisionRuntime
  {
  public:
    VisionRuntime(ProducerPipeline& producer, ConsumerPipeline* consumer = nullptr);
    ~VisionRuntime();

    VisionRuntime(const VisionRuntime&)            = delete;
    VisionRuntime& operator=(const VisionRuntime&) = delete;
    VisionRuntime(VisionRuntime&&)                 = delete;
    VisionRuntime& operator=(VisionRuntime&&)      = delete;

    // launch producer and consumer thread
    void start();
    void stop();

    bool is_running() const noexcept;

  private:
    void _start_producer();
    void _start_consumer();

    ProducerPipeline& _producer;
    ConsumerPipeline* _consumer;

    std::atomic<bool> _running{false};

    std::thread _producer_thread{};
    std::thread _consumer_thread{};
  };
} // namespace omniseer::vision
