#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace omniseer::vision
{
  // Pixel formats used throughout the pipeline
  enum class PixelFormat : uint32_t
  {
    NV12,   // 2 planes: plane0=Y, plane1=interleaved UV (4:2:0)
    RGB888, // 1 plane: 3-bytes-per-pixel
    BGR888, // 1 plane: 3-bytes-per-pixel
  };

  // Width/height container used for frame and buffer dimensions
  struct Size
  {
    int w = 0;
    int h = 0;
  };

  // Container used for letterboxing and cropping
  struct Rect
  {
    int x = 0;
    int y = 0;
    int w = 0;
    int h = 0;
  };

  // Describes a plane of an image that is backed by a DMA-BUF file descriptor
  struct DMABufPlane
  {
    int      fd     = -1; // DMA-BUF file descriptor
    uint32_t stride = 0;  // bytes per line
    uint32_t offset = 0;  // byte offset within fd where this plane begins

    size_t alloc_size = 0; // allocated capacity for this plane (bytes)
    size_t bytesused  = 0; // how many bytes are valid for this frame (from V4L2 DQBUF);
  };

  // Frame descriptor from the V4L2 capture ring owned by the kernel driver
  // Typical lifetime:
  //   FrameDescriptor f = capture.dequeue();   // DQBUF: borrow slot i
  //   rga.process(f, dst);                     // RGA reads from f's DMABUF planes
  //   capture.requeue(f.v4l2_index);           // QBUF: give slot i back to driver
  struct FrameDescriptor
  {
    Size        size{}; // frame dimensions from v4l2
    PixelFormat fmt = PixelFormat::NV12;

    uint32_t                   num_planes = 0;
    std::array<DMABufPlane, 2> planes{};

    // Timing
    uint64_t capture_ts_mono_ns = 0; // when the driver says the frame timestamp occurred
    uint64_t dequeue_ts_mono_ns = 0; // when userspace thread got the frame back from DQBUF

    // Identity of the V4L2 ring slot that currently contains this frame, used to re-queue it.
    uint32_t v4l2_index = std::numeric_limits<uint32_t>::max();

    // Increasing frame counter provided by V4L2 (useful for detecting drops / reordering).
    uint32_t sequence = 0;
  };

  // Buffer slot allocated by app into a buffer pool
  // RGA reads into, RKNN reads from as model input tensor
  //
  // Memory allocated in pool/via allocator, reused across frames
  // Typical lifetime:
  //   ImageBuffer dst = pool.acquire();
  //   rga.process(src_frame, dst);
  //   rknn.infer(dst);
  //   pool.release(dst);
  struct ImageBuffer
  {
    Size        size{};                    // buffer dimensions (e.g., 640x384)
    PixelFormat fmt = PixelFormat::RGB888; // model input

    uint32_t                   num_planes = 0;
    std::array<DMABufPlane, 2> planes{};

    size_t total_alloc_size = 0; // capacity across planes
  };

} // namespace omniseer::vision
