#include "omniseer/vision/rga_preprocess.hpp"

#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <im2d.h>
#include <linux/dma-buf.h>
#include <rga.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>

namespace omniseer::vision
{
  namespace
  {
    int rga_format(PixelFormat fmt)
    {
      switch (fmt)
      {
      case PixelFormat::NV12:
        return RK_FORMAT_YCbCr_420_SP;
      case PixelFormat::RGB888:
        return RK_FORMAT_RGB_888;
      }
      return -1;
    }

    bool wrap_nv12_src(const FrameDescriptor& src, rga_buffer_t& out)
    {
      if (src.fmt != PixelFormat::NV12)
        return false;
      if (src.num_planes != 2)
        return false;
      if (src.size.w <= 0 || src.size.h <= 0)
        return false;

      const auto& y  = src.planes[0];
      const auto& uv = src.planes[1];

      if (y.fd < 0 || uv.fd < 0)
        return false;

      if (y.fd != uv.fd)
        return false;

      if (y.stride == 0)
        return false;

      const uint32_t expected_uv_offset = static_cast<uint32_t>(static_cast<uint64_t>(y.stride) *
                                                                static_cast<uint64_t>(src.size.h));
      if (uv.offset != expected_uv_offset)
        return false;

      const int wstride_px = static_cast<int>(y.stride);
      out = wrapbuffer_fd(y.fd, src.size.w, src.size.h, rga_format(PixelFormat::NV12), wstride_px,
                          src.size.h);
      return true;
    }

    bool wrap_rgb888_dst(const ImageBuffer& dst, int w, int h, rga_buffer_t& out)
    {
      if (w <= 0 || h <= 0)
        return false;

      constexpr int bpp = 3;

      if (dst.num_planes < 1)
        return false;
      const auto& p = dst.planes[0];
      if (p.fd < 0 || p.stride == 0)
        return false;

      const uint32_t stride_bytes = p.stride;
      if (stride_bytes % static_cast<uint32_t>(bpp) != 0)
        return false;

      const int wstride_px = static_cast<int>(stride_bytes / static_cast<uint32_t>(bpp));
      if (wstride_px < w)
        return false;

      out = wrapbuffer_fd(p.fd, w, h, rga_format(PixelFormat::RGB888), wstride_px, h);
      return true;
    }

    void dmabuf_sync(int fd, uint64_t flags)
    {
      dma_buf_sync sync{};
      sync.flags = flags;
      (void) ::ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
    }

    [[noreturn]] void throw_prefill_error(const std::string& message)
    {
      throw std::runtime_error("RgaPreprocess::prefill: " + message);
    }

    void cpu_prefill(ImageBuffer& dst, int w, int h, uint8_t v)
    {
      if (w <= 0 || h <= 0)
        throw_prefill_error("invalid destination size");
      if (dst.fmt != PixelFormat::RGB888)
        throw_prefill_error("destination pixel format must be RGB888");
      if (dst.num_planes < 1)
        throw_prefill_error("destination must expose at least one plane");
      const auto& p = dst.planes[0];
      if (p.fd < 0 || p.stride == 0)
        throw_prefill_error("destination plane fd/stride are invalid");

      const size_t needed = static_cast<size_t>(p.stride) * static_cast<size_t>(h);
      const size_t map_size =
          (p.alloc_size != 0 && p.alloc_size >= needed) ? p.alloc_size : needed;

      dmabuf_sync(p.fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE);
      void* map = ::mmap(nullptr, map_size, PROT_WRITE, MAP_SHARED, p.fd, 0);
      if (map == MAP_FAILED)
      {
        dmabuf_sync(p.fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE);
        const int err = errno;
        throw_prefill_error("mmap failed: " + std::string(std::strerror(err)));
      }

      std::memset(map, v, needed);

      ::munmap(map, map_size);
      dmabuf_sync(p.fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE);
    }
  } // namespace

  RgaPreprocess::RgaPreprocess(RgaPreprocessConfig cfg) : _cfg(cfg)
  {
    _initialized = _init_letterbox();
    if (!_initialized)
    {
      throw std::invalid_argument("RgaPreprocess: invalid source/destination geometry");
    }
  }

  LetterboxMeta RgaPreprocess::_compute_letterbox(int src_w, int src_h, int dst_w, int dst_h)
  {
    LetterboxMeta out{};
    if (src_w <= 0 || src_h <= 0 || dst_w <= 0 || dst_h <= 0)
      return out;

    const float sx = static_cast<float>(dst_w) / static_cast<float>(src_w);
    const float sy = static_cast<float>(dst_h) / static_cast<float>(src_h);
    const float s  = std::min(sx, sy);

    int resized_w = static_cast<int>(std::lround(static_cast<float>(src_w) * s));
    int resized_h = static_cast<int>(std::lround(static_cast<float>(src_h) * s));

    resized_w = std::clamp(resized_w, 1, dst_w);
    resized_h = std::clamp(resized_h, 1, dst_h);

    out.scale     = s;
    out.resized_w = resized_w;
    out.resized_h = resized_h;
    out.pad_x     = (dst_w - resized_w) / 2;
    out.pad_y     = (dst_h - resized_h) / 2;
    return out;
  }

  bool RgaPreprocess::_init_letterbox()
  {
    if (_cfg.src_w <= 0 || _cfg.src_h <= 0)
      return false;
    if (_cfg.dst_w <= 0 || _cfg.dst_h <= 0)
      return false;

    _lb    = _compute_letterbox(_cfg.src_w, _cfg.src_h, _cfg.dst_w, _cfg.dst_h);
    _srect = im_rect{0, 0, _cfg.src_w, _cfg.src_h};
    _drect = im_rect{_lb.pad_x, _lb.pad_y, _lb.resized_w, _lb.resized_h};

    return true;
  }

  void RgaPreprocess::prefill(ImageBuffer& dst) const
  {
    // Normalize dst descriptor for downstream users
    dst.size.w           = _cfg.dst_w;
    dst.size.h           = _cfg.dst_h;
    dst.fmt              = PixelFormat::RGB888;
    dst.num_planes       = 1;
    dst.planes[0].offset = 0;

    // Prefill is intentionally out-of-hot-path; keep run() to a single RGA op per frame.
    cpu_prefill(dst, _cfg.dst_w, _cfg.dst_h, _cfg.pad_value);
  }

  PreprocessResult RgaPreprocess::preflight(const FrameDescriptor& src, ImageBuffer& dst,
                                            LetterboxMeta* meta) const noexcept
  {
    if (!_initialized)
      return {PreprocessStatus::InvalidConfig};
    if (_cfg.dst_w <= 0 || _cfg.dst_h <= 0)
      return {PreprocessStatus::InvalidConfig};
    if (src.size.w != _cfg.src_w || src.size.h != _cfg.src_h)
      return {PreprocessStatus::SourceSizeMismatch};

    // Normalize dst descriptor for downstream users
    dst.size.w           = _cfg.dst_w;
    dst.size.h           = _cfg.dst_h;
    dst.fmt              = PixelFormat::RGB888;
    dst.num_planes       = 1;
    dst.planes[0].offset = 0;

    rga_buffer_t src_buf{}, dst_buf{};
    if (!wrap_nv12_src(src, src_buf))
      return {PreprocessStatus::InvalidSourceDescriptor};
    if (!wrap_rgb888_dst(dst, _cfg.dst_w, _cfg.dst_h, dst_buf))
      return {PreprocessStatus::InvalidDestinationDescriptor};

    const im_rect& srect = _srect;
    const im_rect& drect = _drect;
    if (meta)
      *meta = _lb;

    if (imcheck(src_buf, dst_buf, srect, drect) <= 0)
      return {PreprocessStatus::ImcheckFailed};

    if (improcess(src_buf, dst_buf, {}, srect, drect, {}, IM_SYNC) <= 0)
      return {PreprocessStatus::ImprocessFailed};

    return {PreprocessStatus::Ok};
  }

  PreprocessResult RgaPreprocess::run(const FrameDescriptor& src, ImageBuffer& dst,
                                      LetterboxMeta* meta) const noexcept
  {
    assert(_initialized);
    assert(src.size.w == _cfg.src_w);
    assert(src.size.h == _cfg.src_h);
    assert(src.fmt == PixelFormat::NV12);
    assert(src.num_planes == 2);
    assert(dst.num_planes >= 1);

    const auto& y = src.planes[0];
    const auto& p = dst.planes[0];

    const int src_wstride_px = static_cast<int>(y.stride);
    const int dst_wstride_px = static_cast<int>(p.stride / 3u);
    rga_buffer_t src_buf = wrapbuffer_fd(y.fd, src.size.w, src.size.h, rga_format(PixelFormat::NV12),
                                         src_wstride_px, src.size.h);
    rga_buffer_t dst_buf = wrapbuffer_fd(p.fd, _cfg.dst_w, _cfg.dst_h, rga_format(PixelFormat::RGB888),
                                         dst_wstride_px, _cfg.dst_h);

    if (meta)
      *meta = _lb;

    if (improcess(src_buf, dst_buf, {}, _srect, _drect, {}, IM_SYNC) <= 0)
      return {PreprocessStatus::ImprocessFailed};

    return {PreprocessStatus::Ok};
  }
} // namespace omniseer::vision
