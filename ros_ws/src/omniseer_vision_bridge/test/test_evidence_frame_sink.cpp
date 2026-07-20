#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <fstream>
#include <string>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>

#include "omniseer/vision/config.hpp"
#include "omniseer/vision/detections.hpp"
#include "omniseer/vision/types.hpp"
#include "omniseer_vision_bridge/evidence_frame_sink.hpp"

namespace
{
constexpr uint64_t kNsPerSec = 1000000000ULL;

std::filesystem::path make_temp_dir()
{
  auto root = std::filesystem::temp_directory_path() /
    ("omniseer_evidence_test_" + std::to_string(::getpid()) + "_" +
    std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(root);
  return root;
}

struct TempRgbImage
{
  std::filesystem::path path{};
  int fd{-1};
  int width{0};
  int height{0};
  int stride{0};
  std::size_t bytes{0};

  TempRgbImage(const std::filesystem::path & dir, int w, int h)
  : path(dir / "rgb.bin"), width(w), height(h), stride(w * 3),
    bytes(static_cast<std::size_t>(stride) * static_cast<std::size_t>(h))
  {
    fd = ::open(path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0600);
    if (fd < 0) {
      throw std::runtime_error("failed to create temporary RGB file");
    }
    if (::ftruncate(fd, static_cast<off_t>(bytes)) != 0) {
      throw std::runtime_error("failed to size temporary RGB file");
    }
    void * mapped = ::mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (mapped == MAP_FAILED) {
      throw std::runtime_error("failed to mmap temporary RGB file");
    }
    auto * data = static_cast<unsigned char *>(mapped);
    for (std::size_t i = 0; i < bytes; ++i) {
      data[i] = static_cast<unsigned char>(i % 251u);
    }
    ::munmap(mapped, bytes);
  }

  ~TempRgbImage()
  {
    if (fd >= 0) {
      ::close(fd);
    }
  }

  omniseer::vision::ImageBuffer image() const
  {
    omniseer::vision::ImageBuffer out{};
    out.size.w = width;
    out.size.h = height;
    out.fmt = omniseer::vision::PixelFormat::RGB888;
    out.num_planes = 1;
    out.total_alloc_size = bytes;
    out.planes[0].fd = fd;
    out.planes[0].stride = static_cast<uint32_t>(stride);
    out.planes[0].offset = 0;
    out.planes[0].alloc_size = bytes;
    out.planes[0].bytesused = bytes;
    return out;
  }
};

omniseer::vision::DetectionsFrame frame(uint64_t frame_id, uint64_t ts, uint32_t count)
{
  omniseer::vision::DetectionsFrame out{};
  out.frame_id = frame_id;
  out.sequence = static_cast<uint32_t>(1000 + frame_id);
  out.capture_ts_real_ns = ts;
  out.source_size.w = 1280;
  out.source_size.h = 720;
  out.count = count;
  if (count > 0) {
    out.detections[0].class_id = 0;
    out.detections[0].score = 0.92F;
    out.detections[0].x1 = 10.0F;
    out.detections[0].y1 = 20.0F;
    out.detections[0].x2 = 110.0F;
    out.detections[0].y2 = 220.0F;
  }
  return out;
}

omniseer::vision::PipelineRemapConfig remap()
{
  omniseer::vision::PipelineRemapConfig out{};
  out.source_size.w = 1280;
  out.source_size.h = 720;
  out.model_input_size.w = 640;
  out.model_input_size.h = 640;
  out.scale = 0.5F;
  out.pad_x = 0;
  out.pad_y = 140;
  out.resized_w = 640;
  out.resized_h = 360;
  return out;
}

bool wait_for_written(const omniseer_vision_bridge::EvidenceFrameSink & sink, uint64_t expected)
{
  for (int i = 0; i < 100; ++i) {
    if (sink.snapshot().written_count >= expected) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

std::string read_text(const std::filesystem::path & path)
{
  std::ifstream in(path);
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

bool has_jpeg_soi(const std::filesystem::path & path)
{
  std::ifstream in(path, std::ios::binary);
  char bytes[2]{};
  in.read(bytes, sizeof(bytes));
  return in.gcount() == 2 && static_cast<unsigned char>(bytes[0]) == 0xffu &&
         static_cast<unsigned char>(bytes[1]) == 0xd8u;
}
} // namespace

TEST(EvidenceFrameSinkTest, WritesJpegAndMetadataForExactFrame)
{
  const auto root = make_temp_dir();
  TempRgbImage rgb(root, 16, 12);

  omniseer_vision_bridge::EvidenceFrameSinkConfig cfg{};
  cfg.evidence_dir = (root / "evidence").string();
  cfg.class_names = {"person"};
  cfg.interval_sec = 1.0;
  cfg.jpeg_quality = 85;
  cfg.min_free_mb = 0;
  cfg.storage_budget_mb = 64;

  {
    omniseer_vision_bridge::EvidenceFrameSink sink(cfg);
    sink.publish(rgb.image(), frame(7, kNsPerSec, 1), remap());
    ASSERT_TRUE(wait_for_written(sink, 1));
  }

  const auto jpg = root / "evidence" / "frames" / "frame_7.jpg";
  ASSERT_TRUE(std::filesystem::exists(jpg));
  EXPECT_TRUE(has_jpeg_soi(jpg));

  const std::string metadata = read_text(root / "evidence" / "evidence.jsonl");
  EXPECT_NE(metadata.find("\"artifact_type\":\"sampled_frame\""), std::string::npos);
  EXPECT_NE(metadata.find("\"image_path\":\"evidence/frames/frame_7.jpg\""), std::string::npos);
  EXPECT_NE(metadata.find("\"frame_id\":7"), std::string::npos);
  EXPECT_NE(metadata.find("\"class_name\":\"person\""), std::string::npos);
  EXPECT_NE(metadata.find("\"source_image\":{\"width\":1280,\"height\":720}"), std::string::npos);
}

TEST(EvidenceFrameSinkTest, SamplesPeriodicallyAndKeepsEmptyDetectionFrames)
{
  const auto root = make_temp_dir();
  TempRgbImage rgb(root, 8, 8);

  omniseer_vision_bridge::EvidenceFrameSinkConfig cfg{};
  cfg.evidence_dir = (root / "evidence").string();
  cfg.interval_sec = 1.0;
  cfg.jpeg_quality = 80;
  cfg.min_free_mb = 0;
  cfg.storage_budget_mb = 64;

  {
    omniseer_vision_bridge::EvidenceFrameSink sink(cfg);
    sink.publish(rgb.image(), frame(1, kNsPerSec, 0), remap());
    sink.publish(rgb.image(), frame(2, kNsPerSec + 500000000ULL, 1), remap());
    sink.publish(rgb.image(), frame(3, 2 * kNsPerSec, 0), remap());
    ASSERT_TRUE(wait_for_written(sink, 2));
    EXPECT_EQ(sink.snapshot().enqueued_count, 2u);
  }

  EXPECT_TRUE(std::filesystem::exists(root / "evidence" / "frames" / "frame_1.jpg"));
  EXPECT_FALSE(std::filesystem::exists(root / "evidence" / "frames" / "frame_2.jpg"));
  EXPECT_TRUE(std::filesystem::exists(root / "evidence" / "frames" / "frame_3.jpg"));
  const std::string metadata = read_text(root / "evidence" / "evidence.jsonl");
  EXPECT_NE(metadata.find("\"detections\":[]"), std::string::npos);
}

TEST(EvidenceFrameSinkTest, StopsWhenStorageBudgetIsReached)
{
  const auto root = make_temp_dir();
  TempRgbImage rgb(root, 16, 16);

  omniseer_vision_bridge::EvidenceFrameSinkConfig cfg{};
  cfg.evidence_dir = (root / "evidence").string();
  cfg.interval_sec = 1.0;
  cfg.jpeg_quality = 90;
  cfg.min_free_mb = 0;
  cfg.storage_budget_mb = 0;

  {
    omniseer_vision_bridge::EvidenceFrameSink sink(cfg);
    sink.publish(rgb.image(), frame(1, kNsPerSec, 1), remap());
    for (int i = 0; i < 100 && !sink.snapshot().stopped; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(sink.snapshot().stopped);
    EXPECT_EQ(sink.snapshot().stopped_reason, "storage_budget_exceeded");
  }

  const std::string metadata = read_text(root / "evidence" / "evidence.jsonl");
  EXPECT_NE(metadata.find("\"artifact_type\":\"evidence_status\""), std::string::npos);
  EXPECT_NE(metadata.find("\"stopped_reason\":\"storage_budget_exceeded\""), std::string::npos);
}
