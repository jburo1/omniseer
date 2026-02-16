#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <gtest/gtest.h>
#include <linux/dma-buf.h>
#include <linux/videodev2.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>

#include "omniseer/vision/dma_heap_alloc.hpp"
#include "omniseer/vision/rga_preprocess.hpp"
#include "omniseer/vision/v4l2_capture.hpp"

namespace
{
  const char* capture_status_name(omniseer::vision::CaptureStatus status)
  {
    using omniseer::vision::CaptureStatus;
    switch (status)
    {
      case CaptureStatus::Ok:
        return "ok";
      case CaptureStatus::NoFrame:
        return "no-frame";
      case CaptureStatus::RetryableError:
        return "retryable-error";
      case CaptureStatus::FatalError:
        return "fatal-error";
    }
    return "unknown";
  }

  uint32_t env_u32(const char* key, uint32_t def)
  {
    const char* v = std::getenv(key);
    if (!v || !*v)
      return def;
    char*         end = nullptr;
    unsigned long n   = std::strtoul(v, &end, 10);
    if (end && *end)
      return def;
    if (n > 0xffffffffUL)
      return def;
    return static_cast<uint32_t>(n);
  }

  void dmabuf_sync(int fd, uint64_t flags)
  {
    dma_buf_sync sync{};
    sync.flags = flags;
    (void) ::ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
  }

  struct MmapView
  {
    void*  ptr  = nullptr;
    size_t size = 0;

    MmapView() = default;
    ~MmapView()
    {
      reset();
    }

    MmapView(const MmapView&)            = delete;
    MmapView& operator=(const MmapView&) = delete;

    MmapView(MmapView&& other) noexcept
    {
      *this = std::move(other);
    }

    MmapView& operator=(MmapView&& other) noexcept
    {
      if (this == &other)
        return *this;
      reset();
      ptr        = other.ptr;
      size       = other.size;
      other.ptr  = nullptr;
      other.size = 0;
      return *this;
    }

    void reset() noexcept
    {
      if (ptr && ptr != MAP_FAILED)
        ::munmap(ptr, size);
      ptr  = nullptr;
      size = 0;
    }

    static MmapView map_read(int fd, size_t size)
    {
      MmapView out{};
      if (fd < 0 || size == 0)
        return out;
      void* p = ::mmap(nullptr, size, PROT_READ, MAP_SHARED, fd, 0);
      if (p == MAP_FAILED)
        return out;
      out.ptr  = p;
      out.size = size;
      return out;
    }
  };

  void expect_row_is_gray114(const uint8_t* row, int width)
  {
    const int bytes = width * 3;
    for (int i = 0; i < bytes; ++i)
    {
      ASSERT_EQ(row[i], 114) << "byte " << i;
    }
  }

} // namespace

TEST(RgaPreprocess, LetterboxPaddingIs114_RealV4l2Frame)
{
  const char*       dev_env = std::getenv("VISION_V4L2_DEV");
  const std::string dev     = (dev_env && *dev_env) ? dev_env : "/dev/video12";

  if (::access(dev.c_str(), R_OK | W_OK) != 0)
    GTEST_SKIP() << "no access to " << dev << " (" << std::strerror(errno) << ")";

  if (::access("/dev/rga", R_OK | W_OK) != 0 && ::access("/dev/rga0", R_OK | W_OK) != 0)
    GTEST_SKIP() << "no access to /dev/rga(/dev/rga0) (" << std::strerror(errno) << ")";

  const uint32_t src_w   = env_u32("VISION_V4L2_W", 1280);
  const uint32_t src_h   = env_u32("VISION_V4L2_H", 720);
  const uint32_t buffers = env_u32("VISION_V4L2_BUFFERS", 4);

  omniseer::vision::V4l2Capture cap({
      .device       = dev,
      .width        = src_w,
      .height       = src_h,
      .fourcc       = V4L2_PIX_FMT_NV12,
      .buffer_count = buffers,
  });

  ASSERT_NO_THROW(cap.start());

  omniseer::vision::FrameDescriptor src{};
  bool                              got = false;
  omniseer::vision::CaptureResult   last{};
  for (int spins = 0; spins < 2000; ++spins)
  {
    last = cap.dequeue(src);
    if (last.ok())
    {
      got = true;
      break;
    }
    if (last.status != omniseer::vision::CaptureStatus::NoFrame &&
        last.status != omniseer::vision::CaptureStatus::RetryableError)
    {
      FAIL() << "dequeue failed with status=" << capture_status_name(last.status)
             << " errno=" << last.sys_errno << " (" << std::strerror(last.sys_errno) << ")";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(got) << "timeout waiting for frame (status=" << capture_status_name(last.status)
                   << ", errno=" << last.sys_errno << ")";

  ASSERT_EQ(src.fmt, omniseer::vision::PixelFormat::NV12);
  ASSERT_EQ(src.num_planes, 2u);
  ASSERT_GE(src.planes[0].fd, 0);
  ASSERT_EQ(src.planes[0].fd, src.planes[1].fd);
  ASSERT_EQ(src.planes[1].offset,
            static_cast<uint32_t>(static_cast<uint64_t>(src.planes[0].stride) *
                                  static_cast<uint64_t>(src.size.h)));

  const int dst_w = 640;
  const int dst_h = 640;

  omniseer::vision::ImageBuffer      dst{};
  omniseer::vision::DmabufAllocation dst_alloc{};
  try
  {
    omniseer::vision::DmaHeapAllocator alloc;
    omniseer::vision::AllocatedImageBuffer allocated =
        alloc.allocate(dst_w, dst_h, omniseer::vision::PixelFormat::RGB888);
    dst      = allocated.buf;
    dst_alloc = std::move(allocated.alloc);
  }
  catch (const std::exception& e)
  {
    GTEST_SKIP() << e.what();
  }

  omniseer::vision::RgaPreprocess stage({
      .src_w     = src.size.w,
      .src_h     = src.size.h,
      .dst_w     = dst_w,
      .dst_h     = dst_h,
      .pad_value = 114,
  });

  ASSERT_NO_THROW(stage.prefill(dst));

  omniseer::vision::LetterboxMeta meta{};
  const auto preflight_result = stage.preflight(src, dst, &meta);
  ASSERT_TRUE(preflight_result.ok());

  const auto run_result = stage.run(src, dst);
  ASSERT_TRUE(run_result.ok());

  EXPECT_EQ(meta.resized_w, 640);
  EXPECT_EQ(meta.resized_h, 360);
  EXPECT_EQ(meta.pad_x, 0);
  EXPECT_EQ(meta.pad_y, 140);

  ASSERT_GE(dst.planes[0].stride, static_cast<uint32_t>(dst_w * 3));
  const size_t map_size = dst.planes[0].alloc_size;
  ASSERT_GE(map_size, static_cast<size_t>(dst.planes[0].stride) * static_cast<size_t>(dst_h));

  dmabuf_sync(dst.planes[0].fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ);
  MmapView mapped = MmapView::map_read(dst.planes[0].fd, map_size);
  ASSERT_TRUE(mapped.ptr) << "failed to mmap destination DMA-BUF";

  const auto* base   = static_cast<const uint8_t*>(mapped.ptr);
  const auto  stride = static_cast<size_t>(dst.planes[0].stride);

  for (int y = 0; y < 140; ++y)
  {
    const uint8_t* row = base + static_cast<size_t>(y) * stride;
    expect_row_is_gray114(row, dst_w);
  }
  for (int y = dst_h - 140; y < dst_h; ++y)
  {
    const uint8_t* row = base + static_cast<size_t>(y) * stride;
    expect_row_is_gray114(row, dst_w);
  }

  dmabuf_sync(dst.planes[0].fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ);

  const omniseer::vision::CaptureResult rq = cap.requeue(src.v4l2_index);
  ASSERT_TRUE(rq.ok()) << "requeue failed with status=" << capture_status_name(rq.status)
                       << " errno=" << rq.sys_errno << " (" << std::strerror(rq.sys_errno)
                       << ")";
  ASSERT_NO_THROW(cap.stop());
}

TEST(RgaPreprocess, RejectsInvalidDescriptors)
{
  omniseer::vision::RgaPreprocess stage({
      .src_w     = 1280,
      .src_h     = 720,
      .dst_w     = 640,
      .dst_h     = 640,
      .pad_value = 114,
  });

  omniseer::vision::ImageBuffer dst{};
  dst.num_planes       = 1;
  dst.planes[0].fd     = 0;
  dst.planes[0].stride = 640 * 3;

  {
    omniseer::vision::FrameDescriptor src{};
    src.size.w       = 1280;
    src.size.h       = 720;
    src.fmt          = omniseer::vision::PixelFormat::RGB888;
    src.num_planes   = 2;
    src.planes[0].fd = 0;
    src.planes[1].fd = 0;
    EXPECT_FALSE(stage.preflight(src, dst).ok());
  }

  {
    omniseer::vision::FrameDescriptor src{};
    src.size.w       = 1280;
    src.size.h       = 720;
    src.fmt          = omniseer::vision::PixelFormat::NV12;
    src.num_planes   = 1;
    src.planes[0].fd = 0;
    EXPECT_FALSE(stage.preflight(src, dst).ok());
  }

  {
    omniseer::vision::FrameDescriptor src{};
    src.size.w       = 1280;
    src.size.h       = 720;
    src.fmt          = omniseer::vision::PixelFormat::NV12;
    src.num_planes   = 2;
    src.planes[0].fd = 0;
    src.planes[1].fd = 1;
    EXPECT_FALSE(stage.preflight(src, dst).ok());
  }

  {
    omniseer::vision::FrameDescriptor src{};
    src.size.w           = 1280;
    src.size.h           = 720;
    src.fmt              = omniseer::vision::PixelFormat::NV12;
    src.num_planes       = 2;
    src.planes[0].fd     = 0;
    src.planes[1].fd     = 0;
    src.planes[0].stride = 1280;
    src.planes[1].offset = static_cast<uint32_t>(1280u * 720u + 1u);
    EXPECT_FALSE(stage.preflight(src, dst).ok());
  }

  {
    omniseer::vision::FrameDescriptor src{};
    src.size.w           = 1280;
    src.size.h           = 720;
    src.fmt              = omniseer::vision::PixelFormat::NV12;
    src.num_planes       = 2;
    src.planes[0].fd     = 0;
    src.planes[1].fd     = 0;
    src.planes[0].stride = 1280;
    src.planes[1].offset = 1280u * 720u;

    omniseer::vision::ImageBuffer bad_dst = dst;
    bad_dst.planes[0].stride              = 640 * 3 + 1;
    EXPECT_FALSE(stage.preflight(src, bad_dst).ok());
  }

  // No additional configuration error cases: RgaPreprocess is RGB888-only.
}
