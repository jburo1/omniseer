#include "omniseer/vision/v4l2_capture.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>

namespace omniseer::vision
{

  // Small helpers
  namespace
  {
    // Wrapper around ioctl that restarts on errno == EINTR
    int xioctl(int fd, unsigned long request, void* arg)
    {
      for (;;)
      {
        int r = ::ioctl(fd, request, arg);
        if (r == -1 && errno == EINTR)
        {
          continue;
        }
        return r;
      }
    }

    // Throws runtime_error with strerror message
    [[noreturn]] void throw_errno(const std::string& what)
    {
      const int err = errno;
      throw std::runtime_error(what + ": " + std::strerror(err));
    }

  } // namespace

  // ctor
  V4l2Capture::V4l2Capture(Config config)
      : _device(std::move(config.device)), _buffer_count(config.buffer_count), _width(config.width),
        _height(config.height), _fourcc(config.fourcc)
  {
  }

  // dtor
  V4l2Capture::~V4l2Capture()
  {
    stop();
  }

  void V4l2Capture::start()
  {
    if (_streaming)
      throw std::runtime_error("V4l2Capture::start: already started");
    if (_device.empty())
      throw std::runtime_error("V4l2Capture::start: empty device path");
    if (_buffer_count == 0)
      throw std::runtime_error("V4l2Capture::start: buffer_count must be > 0");

    stop();

    _fd = ::open(_device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (_fd < 0)
      throw_errno("open(" + _device + ")");

    try
    {
      v4l2_capability cap{};
      if (xioctl(_fd, VIDIOC_QUERYCAP, &cap) == -1)
        throw_errno("VIDIOC_QUERYCAP");

      uint32_t caps =
          (cap.capabilities & V4L2_CAP_DEVICE_CAPS) ? cap.device_caps : cap.capabilities;
      if ((caps & V4L2_CAP_STREAMING) == 0)
      {
        throw std::runtime_error("V4l2Capture: device does not support V4L2_CAP_STREAMING");
      }
      if ((caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE) == 0)
        throw std::runtime_error(
            "V4l2Capture: device does not support V4L2_CAP_VIDEO_CAPTURE_MPLANE");

      if (_fourcc != V4L2_PIX_FMT_NV12)
        throw std::runtime_error("V4l2Capture: only V4L2_PIX_FMT_NV12 is supported");

      // Negotiate multi-planar NV12 at requested dimensions (e.g. 1280x720).
      v4l2_format fmt{};
      fmt.type                   = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
      fmt.fmt.pix_mp.width       = _width;
      fmt.fmt.pix_mp.height      = _height;
      fmt.fmt.pix_mp.pixelformat = _fourcc;
      fmt.fmt.pix_mp.field       = V4L2_FIELD_NONE;
      fmt.fmt.pix_mp.num_planes  = 1; // rkisp_selfpath NV12 reports 1 plane

      if (xioctl(_fd, VIDIOC_TRY_FMT, &fmt) == -1)
        throw_errno("VIDIOC_TRY_FMT");
      if (xioctl(_fd, VIDIOC_S_FMT, &fmt) == -1)
        throw_errno("VIDIOC_S_FMT");

      v4l2_format got{};
      got.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
      if (xioctl(_fd, VIDIOC_G_FMT, &got) == -1)
        throw_errno("VIDIOC_G_FMT");

      if (got.fmt.pix_mp.width != _width || got.fmt.pix_mp.height != _height)
      {
        throw std::runtime_error("V4l2Capture: driver did not accept requested size; got " +
                                 std::to_string(got.fmt.pix_mp.width) + "x" +
                                 std::to_string(got.fmt.pix_mp.height));
      }
      if (got.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_NV12)
        throw std::runtime_error("V4l2Capture: driver did not accept NV12");
      if (got.fmt.pix_mp.num_planes != 1)
        throw std::runtime_error("V4l2Capture: expected NV12 to expose 1 plane on this node");

      _bytesperline = got.fmt.pix_mp.plane_fmt[0].bytesperline;

      v4l2_requestbuffers req{};
      req.count  = _buffer_count;
      req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
      req.memory = V4L2_MEMORY_MMAP;
      if (xioctl(_fd, VIDIOC_REQBUFS, &req) == -1)
        throw_errno("VIDIOC_REQBUFS");
      if (req.count == 0)
      {
        throw std::runtime_error("V4l2Capture: driver returned 0 capture buffers");
      }

      _slots.assign(req.count, Slot{});
      for (uint32_t i = 0; i < req.count; ++i)
      {
        v4l2_plane  plane{};
        v4l2_buffer buf{};
        buf.type     = req.type;
        buf.memory   = req.memory;
        buf.index    = i;
        buf.length   = 1;
        buf.m.planes = &plane;
        if (xioctl(_fd, VIDIOC_QUERYBUF, &buf) == -1)
          throw_errno("VIDIOC_QUERYBUF");

        _slots[i].alloc_size = plane.length;

        v4l2_exportbuffer exp{};
        exp.type  = req.type;
        exp.index = i;
        exp.plane = 0;
        exp.flags = O_CLOEXEC;
        if (xioctl(_fd, VIDIOC_EXPBUF, &exp) == -1)
          throw_errno("VIDIOC_EXPBUF");
        _slots[i].dmabuf_fd = exp.fd;

        v4l2_plane  qplane{};
        v4l2_buffer qbuf{};
        qbuf.type     = req.type;
        qbuf.memory   = req.memory;
        qbuf.index    = i;
        qbuf.length   = 1;
        qbuf.m.planes = &qplane;
        if (xioctl(_fd, VIDIOC_QBUF, &qbuf) == -1)
          throw_errno("VIDIOC_QBUF");
      }

      int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
      if (xioctl(_fd, VIDIOC_STREAMON, &type) == -1)
        throw_errno("VIDIOC_STREAMON");

      _streaming = true;
    }
    catch (...)
    {
      stop();
      throw;
    }
  }

  void V4l2Capture::stop() noexcept
  {
    if (_fd < 0)
    {
      _slots.clear();
      _streaming    = false;
      _bytesperline = 0;
      return;
    }

    if (_streaming)
    {
      int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
      (void) xioctl(_fd, VIDIOC_STREAMOFF, &type);
      _streaming = false;
    }

    for (auto& slot : _slots)
    {
      if (slot.dmabuf_fd >= 0)
      {
        ::close(slot.dmabuf_fd);
        slot.dmabuf_fd = -1;
      }
    }
    _slots.clear();

    ::close(_fd);
    _fd           = -1;
    _bytesperline = 0;
  }

  bool V4l2Capture::dequeue(FrameDescriptor& out)
  {
    if (!_streaming)
      throw std::runtime_error("V4l2Capture::dequeue: not streaming");

    v4l2_plane  plane{};
    v4l2_buffer buf{};
    buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory   = V4L2_MEMORY_MMAP;
    buf.length   = 1;
    buf.m.planes = &plane;

    if (xioctl(_fd, VIDIOC_DQBUF, &buf) == -1)
    {
      if (errno == EAGAIN)
        return false;
      throw_errno("VIDIOC_DQBUF");
    }

    if (buf.index >= _slots.size())
    {
      throw std::runtime_error("V4l2Capture::dequeue: out-of-range buffer index");
    }

    const Slot& slot = _slots[buf.index];

    out                    = FrameDescriptor{};
    out.size.w             = static_cast<int>(_width);
    out.size.h             = static_cast<int>(_height);
    out.fmt                = PixelFormat::NV12;
    out.num_planes         = 2;
    out.v4l2_index         = buf.index;
    out.sequence           = buf.sequence;
    out.capture_ts_mono_ns = 0;
    out.dequeue_ts_mono_ns = 0;

    const size_t bytesused_total = static_cast<size_t>(plane.bytesused);
    const size_t total_alloc     = static_cast<size_t>(slot.alloc_size);
    const size_t y_size    = static_cast<size_t>(_bytesperline) * static_cast<size_t>(_height);
    const size_t uv_offset = (y_size < total_alloc) ? y_size : total_alloc;

    out.planes[0].fd         = slot.dmabuf_fd;
    out.planes[0].stride     = _bytesperline;
    out.planes[0].offset     = 0;
    out.planes[0].alloc_size = uv_offset;
    out.planes[0].bytesused  = (bytesused_total < uv_offset) ? bytesused_total : uv_offset;

    out.planes[1].fd         = slot.dmabuf_fd;
    out.planes[1].stride     = _bytesperline;
    out.planes[1].offset     = static_cast<uint32_t>(uv_offset);
    out.planes[1].alloc_size = (uv_offset < total_alloc) ? (total_alloc - uv_offset) : 0;
    out.planes[1].bytesused  = (bytesused_total > uv_offset) ? (bytesused_total - uv_offset) : 0;

    return true;
  }

  void V4l2Capture::requeue(uint32_t index)
  {
    if (!_streaming)
      throw std::runtime_error("V4l2Capture::requeue: not streaming");
    if (index >= _slots.size())
      throw std::runtime_error("V4l2Capture::requeue: index out of range");

    v4l2_plane  plane{};
    v4l2_buffer buf{};
    buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory   = V4L2_MEMORY_MMAP;
    buf.index    = index;
    buf.length   = 1;
    buf.m.planes = &plane;
    if (xioctl(_fd, VIDIOC_QBUF, &buf) == -1)
      throw_errno("VIDIOC_QBUF");
  }

} // namespace omniseer::vision
