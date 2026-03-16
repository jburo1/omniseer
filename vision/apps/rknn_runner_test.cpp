#include <cerrno>
#include <cstring>
#include <fstream>
#include <gtest/gtest.h>
#include <optional>
#include <rknn_api.h>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

#include "omniseer/vision/dma_heap_alloc.hpp"
#include "omniseer/vision/image_buffer_pool.hpp"
#include "omniseer/vision/rknn_runner.hpp"

namespace
{
  constexpr const char* kModelRelPath = "/testdata/rknn_runner/yolo_world_v2s_i8.rknn";

  std::string make_rknn_error(const char* where, int code)
  {
    return std::string(where) + " failed, code=" + std::to_string(code);
  }

  std::vector<uint8_t> read_file(const std::string& path)
  {
    std::ifstream ifs(path, std::ios::binary | std::ios::ate);
    if (!ifs)
      throw std::runtime_error("failed to open file: " + path);

    const std::ifstream::pos_type end = ifs.tellg();
    if (end <= 0)
      throw std::runtime_error("file is empty: " + path);

    std::vector<uint8_t> data(static_cast<size_t>(end));
    ifs.seekg(0, std::ios::beg);
    if (!ifs.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(data.size())))
      throw std::runtime_error("failed to read file: " + path);
    return data;
  }

  size_t tensor_bytes(const rknn_tensor_attr& attr)
  {
    return (attr.size_with_stride != 0) ? attr.size_with_stride : attr.size;
  }

  std::string model_path()
  {
    return std::string(VISION_SOURCE_DIR) + kModelRelPath;
  }

  std::vector<int8_t> make_zero_text_blob(const std::string& model)
  {
    const std::vector<uint8_t> model_data = read_file(model);

    rknn_context ctx = 0;
    int          rc  = rknn_init(&ctx, const_cast<uint8_t*>(model_data.data()),
                                 static_cast<uint32_t>(model_data.size()), 0, nullptr);
    if (rc != RKNN_SUCC)
      throw std::runtime_error(make_rknn_error("rknn_init", rc));

    std::optional<std::vector<int8_t>> blob{};
    try
    {
      rknn_input_output_num io_num{};
      rc = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
      if (rc != RKNN_SUCC)
        throw std::runtime_error(make_rknn_error("rknn_query(RKNN_QUERY_IN_OUT_NUM)", rc));

      for (uint32_t i = 0; i < io_num.n_input; ++i)
      {
        rknn_tensor_attr attr{};
        attr.index = i;
        rc         = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &attr, sizeof(attr));
        if (rc != RKNN_SUCC)
          throw std::runtime_error(make_rknn_error("rknn_query(RKNN_QUERY_INPUT_ATTR)", rc));
        if (std::strcmp(attr.name, "texts") == 0)
          blob = std::vector<int8_t>(tensor_bytes(attr), 0);
      }
    }
    catch (...)
    {
      (void) rknn_destroy(ctx);
      throw;
    }

    (void) rknn_destroy(ctx);
    if (!blob.has_value())
      throw std::runtime_error("model does not expose a texts input");
    return std::move(*blob);
  }

  enum class TestPattern : uint8_t
  {
    Black,
    White,
    Checkerboard,
  };

  struct MappedSlot
  {
    void*  base{nullptr};
    size_t size{0};
  };

  MappedSlot map_slot(const omniseer::vision::ImageBuffer& buffer)
  {
    if (buffer.num_planes < 1)
      throw std::runtime_error("buffer has no planes");

    const auto&  plane = buffer.planes[0];
    const size_t map_len =
        (buffer.total_alloc_size != 0) ? buffer.total_alloc_size : plane.alloc_size;
    if (plane.fd < 0 || map_len == 0)
      throw std::runtime_error("buffer is not backed by a valid DMA-BUF");

    void* mapped = ::mmap(nullptr, map_len, PROT_READ | PROT_WRITE, MAP_SHARED, plane.fd, 0);
    if (mapped == MAP_FAILED)
      throw std::runtime_error("mmap failed: " + std::string(std::strerror(errno)));

    return {
        .base = mapped,
        .size = map_len,
    };
  }

  void unmap_slot(MappedSlot& slot) noexcept
  {
    if (slot.base != nullptr && slot.size > 0)
      (void) ::munmap(slot.base, slot.size);
    slot.base = nullptr;
    slot.size = 0;
  }

  void write_pattern(const omniseer::vision::ImageBuffer& buffer, const MappedSlot& mapped,
                     TestPattern pattern)
  {
    if (mapped.base == nullptr || mapped.size == 0)
      throw std::runtime_error("slot is not mapped");

    const auto& plane = buffer.planes[0];
    auto*       base  = static_cast<uint8_t*>(mapped.base);
    for (int y = 0; y < buffer.size.h; ++y)
    {
      uint8_t* row = base + static_cast<size_t>(y) * plane.stride;
      for (int x = 0; x < buffer.size.w; ++x)
      {
        const size_t pixel = static_cast<size_t>(x) * 3u;
        switch (pattern)
        {
        case TestPattern::Black:
          row[pixel + 0] = 0;
          row[pixel + 1] = 0;
          row[pixel + 2] = 0;
          break;
        case TestPattern::White:
          row[pixel + 0] = 255;
          row[pixel + 1] = 255;
          row[pixel + 2] = 255;
          break;
        case TestPattern::Checkerboard:
        {
          const uint8_t v = (((x / 16) + (y / 16)) % 2 == 0) ? 255 : 0;
          row[pixel + 0]  = v;
          row[pixel + 1]  = static_cast<uint8_t>(255 - v);
          row[pixel + 2]  = v / 2;
          break;
        }
        }
      }
    }
  }

  std::vector<std::vector<uint8_t>> snapshot_outputs(
      const std::vector<omniseer::vision::RknnOutputView>& outputs)
  {
    std::vector<std::vector<uint8_t>> copy{};
    copy.reserve(outputs.size());
    for (const auto& output : outputs)
    {
      const auto* begin = static_cast<const uint8_t*>(output.data);
      copy.emplace_back(begin, begin + output.bytes);
    }
    return copy;
  }

  size_t differing_bytes(const std::vector<std::vector<uint8_t>>& lhs,
                         const std::vector<std::vector<uint8_t>>& rhs)
  {
    if (lhs.size() != rhs.size())
      throw std::runtime_error("output vector sizes differ");

    size_t diff = 0;
    for (size_t i = 0; i < lhs.size(); ++i)
    {
      if (lhs[i].size() != rhs[i].size())
        throw std::runtime_error("output tensor sizes differ");
      for (size_t j = 0; j < lhs[i].size(); ++j)
      {
        if (lhs[i][j] != rhs[i][j])
          ++diff;
      }
    }
    return diff;
  }

  class RknnRunnerSmokeTest : public ::testing::Test
  {
  protected:
    ~RknnRunnerSmokeTest() override
    {
      for (auto& mapped : mapped_slots_)
        unmap_slot(mapped);
    }

    void SetUp() override
    {
      model_path_ = model_path();
      if (::access(model_path_.c_str(), R_OK) != 0)
        GTEST_SKIP() << "missing model asset: " << model_path_;

      text_i8_ = make_zero_text_blob(model_path_);

      try
      {
        omniseer::vision::DmaHeapAllocator allocator;
        pool_.allocate_all(allocator, 640, 640, omniseer::vision::PixelFormat::RGB888);
        mapped_slots_.push_back(map_slot(pool_.buffer_at(0)));
        mapped_slots_.push_back(map_slot(pool_.buffer_at(1)));
        write_pattern(pool_.buffer_at(0), mapped_slots_[0], TestPattern::Black);
        write_pattern(pool_.buffer_at(1), mapped_slots_[1], TestPattern::White);
      }
      catch (const std::exception& e)
      {
        GTEST_SKIP() << e.what();
      }
    }

    std::string                       model_path_{};
    omniseer::vision::ImageBufferPool pool_{};
    std::vector<int8_t>               text_i8_{};
    std::vector<MappedSlot>           mapped_slots_{};
  };
} // namespace

TEST_F(RknnRunnerSmokeTest, PreflightArmsRunner)
{
  omniseer::vision::RknnRunnerConfig cfg{};
  cfg.model_path  = model_path_;
  cfg.warmup_runs = 0;
  omniseer::vision::RknnRunner runner(cfg);

  ASSERT_NO_THROW(runner.preflight(pool_, text_i8_.data(), text_i8_.size()));
  EXPECT_TRUE(runner.is_armed());

  const auto& descs = runner.output_descs();
  const auto& outputs = runner.outputs();
  ASSERT_EQ(descs.size(), outputs.size());
  ASSERT_FALSE(outputs.empty());
  for (size_t i = 0; i < outputs.size(); ++i)
  {
    const auto& desc = descs[i];
    EXPECT_EQ(desc.index, i);
    EXPECT_GT(desc.n_dims, 0u);
    EXPECT_FALSE(desc.name.empty());

    const auto& output = outputs[i];
    EXPECT_NE(output.data, nullptr);
    EXPECT_GT(output.bytes, 0u);
  }
}

TEST_F(RknnRunnerSmokeTest, InferObservesInputChanges)
{
  omniseer::vision::RknnRunnerConfig cfg{};
  cfg.model_path  = model_path_;
  cfg.warmup_runs = 0;
  omniseer::vision::RknnRunner runner(cfg);

  ASSERT_NO_THROW(runner.preflight(pool_, text_i8_.data(), text_i8_.size()));

  const auto first = runner.infer(0);
  ASSERT_EQ(first.status, omniseer::vision::InferStatus::Ok)
      << "rknn_code=" << first.rknn_code << " sys_errno=" << first.sys_errno;
  const auto outputs_black = snapshot_outputs(runner.outputs());

  const auto second = runner.infer(1);
  ASSERT_EQ(second.status, omniseer::vision::InferStatus::Ok)
      << "rknn_code=" << second.rknn_code << " sys_errno=" << second.sys_errno;
  const auto outputs_white = snapshot_outputs(runner.outputs());

  write_pattern(pool_.buffer_at(0), mapped_slots_[0], TestPattern::Checkerboard);

  const auto third = runner.infer(0);
  ASSERT_EQ(third.status, omniseer::vision::InferStatus::Ok)
      << "rknn_code=" << third.rknn_code << " sys_errno=" << third.sys_errno;
  const auto outputs_checker = snapshot_outputs(runner.outputs());

  const size_t diff_slots     = differing_bytes(outputs_black, outputs_white);
  const size_t diff_same_slot = differing_bytes(outputs_black, outputs_checker);

  EXPECT_GT(diff_slots, 0u)
      << "different pool slots with different image bytes produced identical outputs";
  EXPECT_GT(diff_same_slot, 0u) << "rewriting the same mapped slot did not change outputs; this is "
                                   "not a defensible base for ConsumerPipeline";
}

TEST_F(RknnRunnerSmokeTest, InferReturnsOkAfterPreflight)
{
  omniseer::vision::RknnRunnerConfig cfg{};
  cfg.model_path  = model_path_;
  cfg.warmup_runs = 0;
  omniseer::vision::RknnRunner runner(cfg);

  ASSERT_NO_THROW(runner.preflight(pool_, text_i8_.data(), text_i8_.size()));

  const omniseer::vision::InferResult result = runner.infer(0);
  EXPECT_EQ(result.status, omniseer::vision::InferStatus::Ok)
      << "rknn_code=" << result.rknn_code << " sys_errno=" << result.sys_errno;
  EXPECT_EQ(result.rknn_code, RKNN_SUCC);
  EXPECT_EQ(result.sys_errno, 0);

  const auto& outputs = runner.outputs();
  ASSERT_FALSE(outputs.empty());
  for (const auto& output : outputs)
  {
    EXPECT_NE(output.data, nullptr);
    EXPECT_GT(output.bytes, 0u);
  }
}
