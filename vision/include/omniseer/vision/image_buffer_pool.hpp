#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <queue>

#include "omniseer/vision/dma_heap_alloc.hpp"
#include "omniseer/vision/spsc_ring.hpp"
#include "omniseer/vision/types.hpp"

namespace omniseer::vision
{
  // Lock-free SPSC pool between RGA (producer) and RKNN (consumer).
  // - Producer thread: acquire_write() -> RGA writes -> publish_ready()
  // - Consumer thread: acquire_read()  -> RKNN reads -> publish_release()
  // Policy: "latest wins" (ready capacity = 1). If producer publishes a new ready buffer
  // while an older one is still waiting, the older one is dropped back to free.
  class ImageBufferPool
  {
  public:
    class WriteLease
    {
    public:
      WriteLease() = default;
      ~WriteLease() noexcept;

      WriteLease(const WriteLease&)            = delete;
      WriteLease& operator=(const WriteLease&) = delete;

      WriteLease(WriteLease&& other) noexcept;
      WriteLease& operator=(WriteLease&& other) noexcept;

      ImageBuffer& buffer() noexcept;
      int          index() const noexcept;

      void publish() noexcept;

      explicit operator bool() const noexcept;

    private:
      friend class ImageBufferPool;
      WriteLease(ImageBufferPool* pool, int idx) noexcept;
      void _reset() noexcept;

      ImageBufferPool* _pool{nullptr};
      int              _idx{-1};
    };

  private:
    // size of pool, N=5
    static const int size{5};

    // Backing allocations for each pool slot (owns DMA-BUF fds).
    std::array<DmabufAllocation, size> allocations{};

    // Preallocated ImageBuffer slots
    std::array<ImageBuffer, size> pool;

    // SPSC free index ring buffer: pool indices which are currently free
    // shared datastructure between consumer and producer threads
    // thread-safe
    SpscRing free_ring{size};

    // Producer-only stash for "dropped ready" indices (latest-wins)
    std::array<int, size> producer_free{};
    int                   producer_free_n{0};

    // index of freshest ImageBuffer
    std::atomic<int> ready_idx{-1};

  public:

    ImageBufferPool();

    // Allocate DMA-BUF memory for all pool slots.
    // This must be called before using the buffers with RGA/RKNN.
    // Throws on invalid arguments or allocation failures.
    void allocate_all(DmaHeapAllocator& allocator, int width, int height, PixelFormat fmt);

    // producer
    // RGA wants to write into the pool
    // returns false if no free buffers
    // OUTPUT idx: which buffer
    bool acquire_write(int& idx);
    std::optional<WriteLease> acquire_write_lease() noexcept;

    // RGA has finishsed writing into the pool
    void publish_ready(int idx);
    void cancel_write(int idx) noexcept;

    // consumer
    // RKNN acquires memory to read
    // returns false if nothing ready
    // OUTPUT idx: which buffer
    bool acquire_read(int& idx);

    // RKNN has consumed image and releases slot back into free list
    void publish_release(int idx);

    // Access preallocated buffer by pool index
    ImageBuffer&       buffer_at(int idx);
    const ImageBuffer& buffer_at(int idx) const;
  };

} // namespace omniseer::vision
