#pragma once
#include <atomic>
#include <cstddef>
#include <memory>

namespace omniseer::vision
{

  // Lock-free single-producer single-consumer ring buffer for int
  class SpscRing
  {
  public:
    explicit SpscRing(std::size_t capacity);

    // Non-copyable/moveable
    SpscRing(const SpscRing&)            = delete;
    SpscRing& operator=(const SpscRing&) = delete;
    SpscRing(SpscRing&&)                 = delete;
    SpscRing& operator=(SpscRing&&)      = delete;

    bool push(int v); // producer
    bool pop(int& v); // consumer

    bool empty() const;
    bool full() const;

    std::size_t capacity() const
    {
      return _capacity;
    }

  private:
    std::size_t _capacity{0}; // max elements
    std::size_t _buf_size{0}; // capacity_ + 1, to deal with head == tail ambiguity, should be 2^k
    std::unique_ptr<int[]> _buf; // the buffer

    // head: next read position (advanced only by consumer)
    // tail: next write position (advanced only by producer)
    // align to cache lines to potentially prevent falsh sharing
    alignas(64) std::atomic<std::size_t> _head{0};
    alignas(64) std::atomic<std::size_t> _tail{0};

    // return next index, this wraps
    std::size_t next(std::size_t i) const noexcept
    {
      return (i + 1) % _buf_size;
    }
  };

} // namespace omniseer::vision
