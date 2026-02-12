#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <linux/videodev2.h>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>

#include "omniseer/vision/v4l2_capture.hpp"

namespace
{
  std::string readlink_fd(int fd)
  {
    std::string   path = "/proc/self/fd/" + std::to_string(fd);
    char          buf[256]{};
    const ssize_t n = ::readlink(path.c_str(), buf, sizeof(buf) - 1);
    if (n < 0)
      return std::string();
    buf[n] = '\0';
    return std::string(buf);
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
} // namespace

// Hardware integration test for V4l2Capture using DMA-BUF NV12 frames
TEST(V4l2Capture, NegotiatesAndStreamsDmabufNv12_1280x720)
{
  const char*       dev_env = std::getenv("VISION_V4L2_DEV");
  const std::string dev     = (dev_env && *dev_env) ? dev_env : "/dev/video12";

  if (::access(dev.c_str(), R_OK | W_OK) != 0)
    GTEST_SKIP() << "no access to " << dev << " (" << std::strerror(errno) << ")";

  const uint32_t width   = env_u32("VISION_V4L2_W", 1280);
  const uint32_t height  = env_u32("VISION_V4L2_H", 720);
  const uint32_t buffers = env_u32("VISION_V4L2_BUFFERS", 4);
  const uint32_t count   = env_u32("VISION_V4L2_COUNT", 30);

  omniseer::vision::V4l2Capture cap({
      .device       = dev,
      .width        = width,
      .height       = height,
      .fourcc       = V4L2_PIX_FMT_NV12,
      .buffer_count = buffers,
  });

  ASSERT_NO_THROW(cap.start());

  std::unordered_map<uint32_t, int> slot_fd;
  slot_fd.reserve(buffers);

  for (uint32_t n = 0; n < count; ++n)
  {
    omniseer::vision::FrameDescriptor f{};
    bool                              got = false;
    for (int spins = 0; spins < 2000; ++spins)
    {
      if (cap.dequeue(f))
      {
        got = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_TRUE(got) << "timeout waiting for frame (dequeue kept returning EAGAIN)";

    EXPECT_EQ(f.size.w, static_cast<int>(width));
    EXPECT_EQ(f.size.h, static_cast<int>(height));
    EXPECT_EQ(f.fmt, omniseer::vision::PixelFormat::NV12);
    EXPECT_EQ(f.num_planes, 2u);
    ASSERT_GE(f.planes[0].fd, 0);
    ASSERT_GE(f.planes[1].fd, 0);
    EXPECT_GT(f.capture_ts_real_ns, 0u);

    auto it = slot_fd.find(f.v4l2_index);
    if (it == slot_fd.end())
    {
      slot_fd.emplace(f.v4l2_index, f.planes[0].fd);

      struct stat st{};
      ASSERT_EQ(::fstat(f.planes[0].fd, &st), 0) << std::strerror(errno);

      const std::string link = readlink_fd(f.planes[0].fd);
      EXPECT_FALSE(link.empty());
      EXPECT_NE(link.find("dmabuf"), std::string::npos)
          << "fd=" << f.planes[0].fd << " link=" << link;
    }
    else
    {
      EXPECT_EQ(f.planes[0].fd, it->second) << "DMA-BUF fd changed for the same V4L2 slot index";
    }

    EXPECT_EQ(f.planes[0].offset, 0u);
    EXPECT_GT(f.planes[1].offset, 0u);
    EXPECT_EQ(f.planes[0].fd, f.planes[1].fd);
    EXPECT_GT(f.planes[0].bytesused, 0u);

    ASSERT_NO_THROW(cap.requeue(f.v4l2_index));
  }

  ASSERT_NO_THROW(cap.stop());
}
