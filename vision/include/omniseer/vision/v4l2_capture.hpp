#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "omniseer/vision/types.hpp"

namespace omniseer::vision
{

  enum class CaptureStatus : uint8_t
  {
    Ok,
    NoFrame,
    RetryableError,
    FatalError,
  };

  struct CaptureResult
  {
    CaptureStatus status{CaptureStatus::Ok};
    int           sys_errno{0};

    bool ok() const noexcept
    {
      return status == CaptureStatus::Ok;
    }
  };

  // Device lifecycle (open/configure/stream on/off)

  // Kernel ring setup (REQBUFS/QUERYBUF/EXPBUF/QBUF)

  // Hot path: dequeue() returns a status + FrameDescriptor
  // Hot path: requeue(index) returns a status

  //   This class opens /dev/videoXX, negotiates capture format (NV12 @ 1280x720), allocates a
  //   driver-managed buffer queue, exports each slot as a DMA-BUF fd, queues all slots, and starts
  //   streaming.

  // On each dequeue(), it obtains the index of the slot containing a fresh frame, packages the
  // corresponding DMA-BUF fd + layout metadata into a FrameDescriptor, and hands that to the next
  // stage (RGA).

  // Once the pipeline is done reading that buffer (i.e., RGA has consumed it), it calls
  // requeue(index) to return the slot to the driver so it can be filled again.

  class V4l2Capture
  {
  public:
    class FrameLease
    {
    public:
      FrameLease() = default;
      ~FrameLease() noexcept;

      FrameLease(const FrameLease&)            = delete;
      FrameLease& operator=(const FrameLease&) = delete;

      FrameLease(FrameLease&& other) noexcept;
      FrameLease& operator=(FrameLease&& other) noexcept;

      const FrameDescriptor& frame() const noexcept;
      FrameDescriptor&       frame() noexcept;

      CaptureResult release() noexcept;

      explicit operator bool() const noexcept;

    private:
      friend class V4l2Capture;
      FrameLease(V4l2Capture* capture, FrameDescriptor frame) noexcept;
      void _reset() noexcept;

      V4l2Capture*    _capture{nullptr};
      FrameDescriptor _frame{};
    };

    struct Config
    {
      std::string device;
      uint32_t    width, height;
      uint32_t    fourcc;       // NV12
      uint32_t    buffer_count; // capture ring size
    };

    struct DequeueLeaseResult
    {
      CaptureResult              capture{};
      std::optional<FrameLease> lease{};

      bool ok() const noexcept
      {
        return capture.ok() && lease.has_value();
      }
    };

    explicit V4l2Capture(Config config);
    ~V4l2Capture();

    // Non-copyable/movable
    V4l2Capture(const V4l2Capture&)            = delete;
    V4l2Capture& operator=(const V4l2Capture&) = delete;
    V4l2Capture(V4l2Capture&&)                 = delete;
    V4l2Capture& operator=(V4l2Capture&&)      = delete;

    // open + set fmt + reqbufs + export fds + qbuf all + streamon
    void start();

    void stop() noexcept;

    // Hot path:
    // borrow token style
    // fills FrameDescriptor using cached per-slot info + per-frame metadata
    // - dequeue(): returns a status (NoFrame for non-blocking EAGAIN)
    // - on Ok, out.v4l2_index identifies the slot to requeue
    CaptureResult dequeue(FrameDescriptor& out) noexcept;
    CaptureResult requeue(uint32_t index) noexcept;
    DequeueLeaseResult dequeue_lease() noexcept;

  private:
    struct Slot
    {
      int      dmabuf_fd{-1}; // from VIDIOC_EXPBUF, stable for lifetime
      uint32_t alloc_size{0}; // from VIDIOC_QUERYBUF (per plane 0)
    };

    std::string       _device{};
    uint32_t          _buffer_count{0};
    int               _fd{-1}; // camera device fd
    uint32_t          _width{0}, _height{0};
    uint32_t          _fourcc{0};       // NV12
    uint32_t          _bytesperline{0}; // stride from G_FMT readback
    std::vector<Slot> _slots{};         // size = buffer_count
    bool              _streaming{false};
  };

} // namespace omniseer::vision
