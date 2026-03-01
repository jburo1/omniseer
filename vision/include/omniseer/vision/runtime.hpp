#pragma once

#include <atomic>
#include <thread>

namespace omniseer::vision
{
  class ProducerPipeline;
  class ConsumerPipeline;

  /**
   * @brief Threaded runtime coordinator for producer and optional consumer pipelines.
   */
  class VisionRuntime
  {
  public:
    /**
     * @brief Construct a runtime bound to producer and optional consumer pipelines.
     */
    VisionRuntime(ProducerPipeline& producer, ConsumerPipeline* consumer = nullptr);
    /// @brief Stop runtime threads and release runtime resources.
    ~VisionRuntime();

    VisionRuntime(const VisionRuntime&)            = delete;
    VisionRuntime& operator=(const VisionRuntime&) = delete;
    VisionRuntime(VisionRuntime&&)                 = delete;
    VisionRuntime& operator=(VisionRuntime&&)      = delete;

    /// @brief Launch runtime worker threads.
    void start();
    /// @brief Request runtime shutdown and join worker threads.
    void stop();

    /// @brief True while the runtime is marked running.
    bool is_running() const noexcept;

  private:
    /// @brief Producer thread entrypoint.
    void _start_producer();
    /// @brief Consumer thread entrypoint.
    void _start_consumer();

    /// @brief Producer pipeline reference (required).
    ProducerPipeline& _producer;
    /// @brief Consumer pipeline pointer (optional).
    ConsumerPipeline* _consumer;

    /// @brief Runtime running flag shared across worker threads.
    std::atomic<bool> _running{false};

    /// @brief Thread that drives the producer pipeline.
    std::thread _producer_thread{};
    /// @brief Thread that drives the consumer pipeline.
    std::thread _consumer_thread{};
  };
} // namespace omniseer::vision
