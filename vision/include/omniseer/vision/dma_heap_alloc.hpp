// Minimal DMA-BUF allocator for shareable image buffers.
//
// It allocates DMA-BUF fds directly via dma-heap (/dev/dma_heap/*) to keep the allocation
// path as small as possible for the RGA/RKNN pipeline.

#pragma once

#include <cstdint>
#include <string>

#include "omniseer/vision/types.hpp"

namespace omniseer::vision
{
  // Owns a DMA-BUF fd (move-only RAII).
  struct DmabufAllocation
  {
    int      dmabuf_fd    = -1;
    uint32_t stride_bytes = 0;
    uint64_t size_bytes   = 0;

    int         width  = 0;
    int         height = 0;
    PixelFormat fmt    = PixelFormat::RGB888;

    DmabufAllocation() = default;
    ~DmabufAllocation();

    DmabufAllocation(const DmabufAllocation&)            = delete;
    DmabufAllocation& operator=(const DmabufAllocation&) = delete;
    DmabufAllocation(DmabufAllocation&& other) noexcept;
    DmabufAllocation& operator=(DmabufAllocation&& other) noexcept;

    bool valid() const
    {
      return dmabuf_fd >= 0;
    }
    void reset() noexcept;
  };

  // A ready-to-use ImageBuffer plus the owning allocation that backs it.
  // Keep this object (or at least `alloc`) alive while consumers use `buf`.
  struct AllocatedImageBuffer
  {
    DmabufAllocation alloc{};
    ImageBuffer      buf{};

    bool valid() const
    {
      return alloc.valid();
    }
  };

  // Allocate shareable video buffers via dma-heap (/dev/dma_heap/*).
  // Returns DMA-BUF fds suitable for RGA write / RKNN read.
  class DmaHeapAllocator
  {
  public:
    // If `device_path` is empty, picks a default heap (typically /dev/dma_heap/cma or system).
    explicit DmaHeapAllocator(std::string device_path = "");
    ~DmaHeapAllocator();

    DmaHeapAllocator(const DmaHeapAllocator&)            = delete;
    DmaHeapAllocator& operator=(const DmaHeapAllocator&) = delete;
    DmaHeapAllocator(DmaHeapAllocator&&)                 = delete;
    DmaHeapAllocator& operator=(DmaHeapAllocator&&)      = delete;

    // Allocate a DMA-BUF suitable for RGA write / RKNN read and return an ImageBuffer
    // descriptor that points at it.
    // Supported formats: RGB888, BGR888.
    // Throws on invalid arguments or allocation failures.
    AllocatedImageBuffer allocate(int width, int height, PixelFormat fmt);

  private:
    int fd_ = -1;
  };

} // namespace omniseer::vision
