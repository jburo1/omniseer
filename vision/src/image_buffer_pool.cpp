#include "omniseer/vision/image_buffer_pool.hpp"

#include <cassert>

#include "omniseer/vision/spsc_ring.hpp"

namespace omniseer::vision
{
  ImageBufferPool::ImageBufferPool()
  {
    for (int i{0}; i < size; ++i)
    {
      free_ring.push(i);
    }
  }

  bool ImageBufferPool::allocate_all(DmaHeapAllocator& allocator, int width, int height,
                                     PixelFormat fmt)
  {
    if (width <= 0 || height <= 0)
      return false;
    if (fmt != PixelFormat::RGB888 && fmt != PixelFormat::BGR888)
      return false;

    // Allocate every slot. If a later allocation fails, already-allocated slots
    // remain valid and owned by the pool.
    for (int i = 0; i < size; ++i)
    {
      allocations[static_cast<std::size_t>(i)].reset();

      AllocatedImageBuffer allocated = allocator.allocate(width, height, fmt);
      if (!allocated.valid())
        return false;

      pool[static_cast<std::size_t>(i)]        = allocated.buf;
      allocations[static_cast<std::size_t>(i)] = std::move(allocated.alloc);
    }

    return true;
  }

  bool ImageBufferPool::acquire_write(int& idx)
  {
    // check producer stash
    if (producer_free_n > 0)
    {
      idx = producer_free[--producer_free_n];
      return true;
    }

    // otherwise, check shared ring
    return free_ring.pop(idx);
  }

  void ImageBufferPool::publish_ready(int idx)
  {
    // publish newest buffer (release is paired with consumer's acquire)
    // all writes before this line are guaranteed to be visible to the consumer if
    // it later does an acquire operation that reads ready_idx
    // establish happens-before relationship
    int old_idx = ready_idx.exchange(idx, std::memory_order_release);

    // ** latest-wins policy **
    // if we had a ready image that was waiting for consumption, drop it and
    // reclaim pool slot into producer-local stash,
    // no latency build-up if consumer is slow
    if (old_idx >= 0)
    {
      assert(producer_free_n < size);
      producer_free[producer_free_n++] = old_idx;
    }
  }

  bool ImageBufferPool::acquire_read(int& idx)
  {
    // take the ready buffer (acquire pairs with producer's release)
    // all the producer's earlier writes are guaranteed visible to me
    // if ready_idx was published with 'release'
    int current = ready_idx.exchange(-1, std::memory_order_acquire);
    if (current < 0)
    {
      return false;
    }

    idx = current;
    return true;
  }

  void ImageBufferPool::publish_release(int idx)
  {
    // free the pool slot
    bool ok = free_ring.push(idx);
    assert(ok);
    (void) ok;
  }

  ImageBuffer& ImageBufferPool::buffer_at(int idx)
  {
    assert(idx >= 0 && idx < size);
    return pool[static_cast<std::size_t>(idx)];
  }

  const ImageBuffer& ImageBufferPool::buffer_at(int idx) const
  {
    assert(idx >= 0 && idx < size);
    return pool[static_cast<std::size_t>(idx)];
  }

} // namespace omniseer::vision
