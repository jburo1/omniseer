#include "omniseer/vision/cpu_letterbox_preprocess.hpp"

#include <cerrno>
#include <cstring>
#include <linux/dma-buf.h>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>

namespace omniseer::vision
{
  namespace
  {
    void dmabuf_sync(int fd, uint64_t flags)
    {
      dma_buf_sync sync{};
      sync.flags = flags;
      (void) ::ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
    }
  } // namespace

  void cpu_letterbox_bgr_to_rgb(const cv::Mat& source_bgr, ImageBuffer& destination,
                                const PipelineRemapConfig& remap, uint8_t pad_value)
  {
    if (source_bgr.empty() || source_bgr.type() != CV_8UC3)
      throw std::invalid_argument("replay source frame must be a non-empty BGR8 image");
    if (source_bgr.cols != remap.source_size.w || source_bgr.rows != remap.source_size.h)
      throw std::invalid_argument("replay source frame does not match letterbox source geometry");
    if (destination.fmt != PixelFormat::RGB888 || destination.num_planes != 1 ||
        destination.size.w != remap.model_input_size.w ||
        destination.size.h != remap.model_input_size.h)
    {
      throw std::invalid_argument("replay destination must be a model-sized RGB888 buffer");
    }

    const DMABufPlane& plane = destination.planes[0];
    if (plane.fd < 0 || plane.stride < static_cast<uint32_t>(destination.size.w * 3))
      throw std::invalid_argument("replay destination DMA-BUF descriptor is invalid");
    const size_t required =
        static_cast<size_t>(plane.offset) + static_cast<size_t>(plane.stride) * destination.size.h;
    const size_t map_size = (plane.alloc_size >= required) ? plane.alloc_size : required;

    cv::Mat resized_bgr{};
    cv::resize(source_bgr, resized_bgr, cv::Size(remap.resized_w, remap.resized_h), 0.0, 0.0,
               cv::INTER_LINEAR);
    cv::Mat resized_rgb{};
    cv::cvtColor(resized_bgr, resized_rgb, cv::COLOR_BGR2RGB);

    dmabuf_sync(plane.fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE);
    void* mapping = ::mmap(nullptr, map_size, PROT_READ | PROT_WRITE, MAP_SHARED, plane.fd, 0);
    if (mapping == MAP_FAILED)
    {
      dmabuf_sync(plane.fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE);
      throw std::runtime_error("failed to mmap replay destination DMA-BUF: " +
                               std::string(std::strerror(errno)));
    }

    try
    {
      auto*   data = static_cast<uint8_t*>(mapping) + plane.offset;
      cv::Mat model_rgb(destination.size.h, destination.size.w, CV_8UC3, data, plane.stride);
      model_rgb.setTo(cv::Scalar(pad_value, pad_value, pad_value));
      resized_rgb.copyTo(
          model_rgb(cv::Rect(remap.pad_x, remap.pad_y, remap.resized_w, remap.resized_h)));
    }
    catch (...)
    {
      (void) ::munmap(mapping, map_size);
      dmabuf_sync(plane.fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE);
      throw;
    }

    (void) ::munmap(mapping, map_size);
    dmabuf_sync(plane.fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE);
  }
} // namespace omniseer::vision
