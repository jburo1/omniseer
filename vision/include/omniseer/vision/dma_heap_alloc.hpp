#pragma once

#include <cstdint>
#include <string>

#include "omniseer/vision/types.hpp"

namespace omniseer::vision
{
  /**
   * @brief Move-only RAII owner for one DMA-BUF allocation.
   */
  struct DmabufAllocation
  {
    /// @brief DMA-BUF file descriptor, or -1 when invalid.
    int dmabuf_fd = -1;
    /// @brief Row stride in bytes.
    uint32_t stride_bytes = 0;
    /// @brief Total allocation size in bytes.
    uint64_t size_bytes = 0;

    /// @brief Allocated width in pixels.
    int width = 0;
    /// @brief Allocated height in pixels.
    int         height = 0;
    /// @brief Pixel format of this allocation.
    PixelFormat fmt    = PixelFormat::RGB888;

    DmabufAllocation() = default;
    ~DmabufAllocation();

    DmabufAllocation(const DmabufAllocation&)            = delete;
    DmabufAllocation& operator=(const DmabufAllocation&) = delete;
    DmabufAllocation(DmabufAllocation&& other) noexcept;
    DmabufAllocation& operator=(DmabufAllocation&& other) noexcept;

    /// @brief True when this object owns a valid DMA-BUF fd.
    bool valid() const
    {
      return dmabuf_fd >= 0;
    }
    /// @brief Close/release the allocation and reset to invalid state.
    void reset() noexcept;
  };

  /**
   * @brief ImageBuffer descriptor paired with its owning DMA-BUF allocation.
   *
   * Keep this object (or at least `alloc`) alive while consumers use `buf`.
   */
  struct AllocatedImageBuffer
  {
    /// @brief Owning DMA-BUF allocation.
    DmabufAllocation alloc{};
    /// @brief ImageBuffer descriptor backed by `alloc`.
    ImageBuffer      buf{};

    /// @brief True when the backing allocation is valid.
    bool valid() const
    {
      return alloc.valid();
    }
  };

  /**
   * @brief Minimal DMA-BUF allocator for shareable video/image buffers.
   *
   * Allocates DMA-BUF fds via dma-heap (for example `/dev/dma_heap/system`) for
   * RGA write and RKNN read paths.
   */
  class DmaHeapAllocator
  {
  public:
    /**
     * @brief Open a dma-heap device for subsequent allocations.
     *
     * If `device_path` is empty, an implementation default heap is selected
     * (typically `/dev/dma_heap/cma` or `system`).
     */
    explicit DmaHeapAllocator(std::string device_path = "");
    ~DmaHeapAllocator();

    DmaHeapAllocator(const DmaHeapAllocator&)            = delete;
    DmaHeapAllocator& operator=(const DmaHeapAllocator&) = delete;
    DmaHeapAllocator(DmaHeapAllocator&&)                 = delete;
    DmaHeapAllocator& operator=(DmaHeapAllocator&&)      = delete;

    /**
     * @brief Allocate one DMA-BUF and return owner + ImageBuffer descriptor.
     *
     * Supported formats: RGB888, BGR888.
     *
     * @throws On invalid arguments or allocation failures.
     */
    AllocatedImageBuffer allocate(int width, int height, PixelFormat fmt);

  private:
    /// @brief Open dma-heap device fd used for allocation ioctls.
    int fd_ = -1;
  };

} // namespace omniseer::vision
