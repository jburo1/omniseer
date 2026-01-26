#include "omniseer/vision/spsc_ring.hpp"

#include <limits>
#include <stdexcept>

namespace omniseer::vision
{

  SpscRing::SpscRing(std::size_t capacity) : _capacity(capacity)
  {
    _buf_size = _capacity + 1;
    _buf      = std::make_unique<int[]>(_buf_size);

    _head.store(0, std::memory_order_relaxed);
    _tail.store(0, std::memory_order_relaxed);
  }

  bool SpscRing::push(int v)
  {
    const std::size_t tail   = _tail.load(std::memory_order_relaxed);
    const std::size_t t_next = next(tail);

    // check if full
    const std::size_t head = _head.load(std::memory_order_acquire);
    if (t_next == head)
    {
      return false;
    }

    // write data in
    _buf[tail] = v;

    // notify consumer
    _tail.store(t_next, std::memory_order_release);
    return true;
  }

  bool SpscRing::pop(int& v)
  {
    // check empty
    const std::size_t head = _head.load(std::memory_order_relaxed);
    const std::size_t tail = _tail.load(std::memory_order_acquire);
    if (head == tail)
    {
      return false;
    }

    // read value out
    v = _buf[head];

    // notify prodcer
    _head.store(next(head), std::memory_order_release);
    return true;
  }

  bool SpscRing::empty() const
  {
    const std::size_t head = _head.load(std::memory_order_acquire);
    const std::size_t tail = _tail.load(std::memory_order_acquire);
    return head == tail;
  }

  bool SpscRing::full() const
  {
    const std::size_t tail = _tail.load(std::memory_order_acquire);
    const std::size_t head = _head.load(std::memory_order_acquire);
    return next(tail) == head;
  }

} // namespace omniseer::vision
