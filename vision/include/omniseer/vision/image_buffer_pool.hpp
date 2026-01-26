#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <queue>

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

  private:
    // size of pool, N=5
    static const int size{5};

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

    // producer
    // RGA wants to write into the pool
    // returns false if no free buffers
    // OUTPUT idx: which buffer
    bool acquire_write(int& idx);

    // RGA has finishsed writing into the pool
    void publish_ready(int idx);

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
