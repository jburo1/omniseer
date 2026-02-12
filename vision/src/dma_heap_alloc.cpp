#include "omniseer/vision/dma_heap_alloc.hpp"

#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>

#include <linux/dma-heap.h>

namespace omniseer::vision
{
  namespace
  {
    int open_heap_device(const std::string& path)
    {
      if (path.empty())
        return -1;
      return ::open(path.c_str(), O_RDWR | O_CLOEXEC);
    }

    int open_default_heap_device()
    {
      const char* candidates[] = {
          "/dev/dma_heap/cma",
          "/dev/dma_heap/system",
      };

      for (const char* p : candidates)
      {
        int fd = ::open(p, O_RDWR | O_CLOEXEC);
        if (fd >= 0)
          return fd;
      }
      return -1;
    }

    uint32_t bytes_per_pixel(PixelFormat fmt)
    {
      switch (fmt)
      {
        case PixelFormat::RGB888:
        case PixelFormat::BGR888:
          return 3;
        case PixelFormat::NV12:
          return 0;
      }
      return 0;
    }

    uint32_t align_up_u32(uint32_t v, uint32_t a)
    {
      return (a == 0) ? v : ((v + (a - 1)) / a) * a;
    }

  } // namespace

  DmabufAllocation::~DmabufAllocation()
  {
    reset();
  }

  DmabufAllocation::DmabufAllocation(DmabufAllocation&& other) noexcept
  {
    *this = std::move(other);
  }

  DmabufAllocation& DmabufAllocation::operator=(DmabufAllocation&& other) noexcept
  {
    if (this == &other)
      return *this;
    reset();

    dmabuf_fd    = other.dmabuf_fd;
    stride_bytes = other.stride_bytes;
    size_bytes   = other.size_bytes;
    width        = other.width;
    height       = other.height;
    fmt          = other.fmt;

    other.dmabuf_fd = -1;
    other.stride_bytes = 0;
    other.size_bytes   = 0;
    other.width        = 0;
    other.height       = 0;

    return *this;
  }

  void DmabufAllocation::reset() noexcept
  {
    if (dmabuf_fd >= 0)
    {
      ::close(dmabuf_fd);
      dmabuf_fd = -1;
    }

    stride_bytes = 0;
    size_bytes   = 0;
    width        = 0;
    height       = 0;
  }

  DmaHeapAllocator::DmaHeapAllocator(std::string device_path)
  {
    if (!device_path.empty())
      fd_ = open_heap_device(device_path);

    if (fd_ < 0)
      fd_ = open_default_heap_device();

    if (fd_ < 0)
    {
      throw std::runtime_error("DmaHeapAllocator: failed to open /dev/dma_heap/* ("
                               "check permissions and that dma-heap is available)");
    }
  }

  DmaHeapAllocator::~DmaHeapAllocator()
  {
    if (fd_ >= 0)
    {
      ::close(fd_);
      fd_ = -1;
    }
  }

  AllocatedImageBuffer DmaHeapAllocator::allocate(int width, int height, PixelFormat fmt)
  {
    AllocatedImageBuffer out{};
    out.alloc.width  = width;
    out.alloc.height = height;
    out.alloc.fmt    = fmt;

    if (width <= 0 || height <= 0)
      return out;
    if (fmt != PixelFormat::RGB888 && fmt != PixelFormat::BGR888)
      return out;
    if (fd_ < 0)
      return out;

    constexpr uint32_t kStrideAlignPixels = 16;
    const uint32_t     bpp               = bytes_per_pixel(fmt);
    if (bpp == 0)
      return out;

    const uint32_t wstride_px   = align_up_u32(static_cast<uint32_t>(width), kStrideAlignPixels);
    const uint64_t stride_bytes = static_cast<uint64_t>(wstride_px) * static_cast<uint64_t>(bpp);
    const uint64_t size_bytes =
        stride_bytes * static_cast<uint64_t>(static_cast<uint32_t>(height));

    dma_heap_allocation_data req{};
    req.len       = size_bytes;
    req.fd_flags  = O_RDWR | O_CLOEXEC;
    req.heap_flags = 0;

    if (::ioctl(fd_, DMA_HEAP_IOCTL_ALLOC, &req) != 0 || static_cast<int>(req.fd) < 0)
      return out;

    out.alloc.dmabuf_fd    = static_cast<int>(req.fd);
    out.alloc.stride_bytes = static_cast<uint32_t>(stride_bytes);
    out.alloc.size_bytes   = size_bytes;

    out.buf               = ImageBuffer{};
    out.buf.size.w        = out.alloc.width;
    out.buf.size.h        = out.alloc.height;
    out.buf.fmt           = out.alloc.fmt;
    out.buf.num_planes    = 1;
    out.buf.total_alloc_size = static_cast<size_t>(out.alloc.size_bytes);

    out.buf.planes[0].fd         = out.alloc.dmabuf_fd;
    out.buf.planes[0].stride     = out.alloc.stride_bytes;
    out.buf.planes[0].offset     = 0;
    out.buf.planes[0].alloc_size = static_cast<size_t>(out.alloc.size_bytes);
    out.buf.planes[0].bytesused  = static_cast<size_t>(out.alloc.size_bytes);

    return out;
  }

} // namespace omniseer::vision
