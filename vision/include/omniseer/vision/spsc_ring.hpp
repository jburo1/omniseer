#pragma once
#include <atomic>
#include <cstddef>
#include <memory>

namespace omniseer::vision
{

  /**
   * @brief Lock-free single-producer/single-consumer ring buffer for `int`.
   *
   * Contract:
   * - Exactly one producer thread calls push().
   * - Exactly one consumer thread calls pop().
   */
  class SpscRing
  {
  public:
    /// @brief Construct a ring with the requested capacity.
    explicit SpscRing(std::size_t capacity);

    /// @brief Non-copyable/non-movable.
    SpscRing(const SpscRing&)            = delete;
    SpscRing& operator=(const SpscRing&) = delete;
    SpscRing(SpscRing&&)                 = delete;
    SpscRing& operator=(SpscRing&&)      = delete;

    /// @brief Producer API: enqueue one value; returns false if the ring is full.
    bool push(int v);
    /// @brief Consumer API: dequeue one value into `v`; returns false if the ring is empty.
    bool pop(int& v);

    /// @brief True when no elements are available for pop().
    bool empty() const;
    /// @brief True when no additional elements can be pushed.
    bool full() const;

    /// @brief Maximum number of elements storable in this ring.
    std::size_t capacity() const
    {
      return _capacity;
    }

  private:
    /// @brief Maximum number of elements.
    std::size_t _capacity{0};
    /// @brief Internal storage size (`capacity + 1`) to disambiguate full vs empty.
    std::size_t _buf_size{0};
    /// @brief Backing ring storage.
    std::unique_ptr<int[]> _buf;

    /// @brief Next read position; advanced only by the consumer.
    alignas(64) std::atomic<std::size_t> _head{0};
    /// @brief Next write position; advanced only by the producer.
    alignas(64) std::atomic<std::size_t> _tail{0};

    /// @brief Return wrapped next index within `_buf_size`.
    std::size_t next(std::size_t i) const noexcept
    {
      return (i + 1) % _buf_size;
    }
  };

} // namespace omniseer::vision
