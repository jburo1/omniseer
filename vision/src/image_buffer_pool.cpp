#include "omniseer/vision/image_buffer_pool.hpp"

#include <cassert>
#include <stdexcept>
#include <utility>

#include "omniseer/vision/spsc_ring.hpp"

namespace omniseer::vision
{
  ImageBufferPool::WriteLease::WriteLease(ImageBufferPool* pool, int idx) noexcept
      : _pool(pool), _idx(idx)
  {
  }

  ImageBufferPool::WriteLease::~WriteLease() noexcept
  {
    _reset();
  }

  ImageBufferPool::WriteLease::WriteLease(WriteLease&& other) noexcept
  {
    *this = std::move(other);
  }

  ImageBufferPool::WriteLease& ImageBufferPool::WriteLease::operator=(WriteLease&& other) noexcept
  {
    if (this == &other)
      return *this;
    _reset();
    _pool       = other._pool;
    _idx        = other._idx;
    other._pool = nullptr;
    other._idx  = -1;
    return *this;
  }

  ImageBuffer& ImageBufferPool::WriteLease::buffer() noexcept
  {
    assert(_pool != nullptr);
    return _pool->buffer_at(_idx);
  }

  int ImageBufferPool::WriteLease::index() const noexcept
  {
    return _idx;
  }

  void ImageBufferPool::WriteLease::publish() noexcept
  {
    if (_pool == nullptr)
      return;
    _pool->publish_ready(_idx);
    _pool = nullptr;
    _idx  = -1;
  }

  ImageBufferPool::WriteLease::operator bool() const noexcept
  {
    return _pool != nullptr;
  }

  void ImageBufferPool::WriteLease::_reset() noexcept
  {
    if (_pool != nullptr)
      _pool->cancel_write(_idx);
    _pool = nullptr;
    _idx  = -1;
  }

  ImageBufferPool::ImageBufferPool()
  {
    for (int i{0}; i < size; ++i)
    {
      free_ring.push(i);
    }
  }

  void ImageBufferPool::allocate_all(DmaHeapAllocator& allocator, int width, int height,
                                     PixelFormat fmt)
  {
    if (width <= 0 || height <= 0)
      throw std::invalid_argument("ImageBufferPool::allocate_all: width/height must be > 0");
    if (fmt != PixelFormat::RGB888 && fmt != PixelFormat::BGR888)
      throw std::invalid_argument("ImageBufferPool::allocate_all: only RGB888/BGR888 are supported");

    for (auto& allocation : allocations)
    {
      allocation.reset();
    }

    try
    {
      // Allocate every slot
      for (int i = 0; i < size; ++i)
      {
        AllocatedImageBuffer allocated = allocator.allocate(width, height, fmt);
        pool[static_cast<std::size_t>(i)]        = allocated.buf;
        allocations[static_cast<std::size_t>(i)] = std::move(allocated.alloc);
      }
    }
    catch (...)
    {
      for (auto& allocation : allocations)
      {
        allocation.reset();
      }
      throw;
    }
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

  std::optional<ImageBufferPool::WriteLease> ImageBufferPool::acquire_write_lease() noexcept
  {
    int idx = -1;
    if (!acquire_write(idx))
      return std::nullopt;
    return WriteLease(this, idx);
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

  void ImageBufferPool::cancel_write(int idx) noexcept
  {
    if (idx < 0 || idx >= size)
      return;
    assert(producer_free_n < size);
    producer_free[producer_free_n++] = idx;
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
