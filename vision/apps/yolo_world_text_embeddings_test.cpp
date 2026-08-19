#include <algorithm>
#include <cstddef>
#include <cstring>
#include <gtest/gtest.h>
#include <string>
#include <unistd.h>
#include <vector>

#include "omniseer/vision/yolo_world_text_embeddings.hpp"
#include "omniseer/vision/yolo_world_text_input.hpp"

namespace
{
  constexpr const char* kClipModelRelPath = "/testdata/text_embeddings/clip_text_fp16.rknn";
  constexpr const char* kClipVocabRelPath = "/testdata/text_embeddings/clip_vocab.bpe";
  constexpr const char* kYoloModelRelPath = "/testdata/rknn_runner/yolo_world_v2s_i8.rknn";

  std::string source_path(const char* relpath)
  {
    return std::string(VISION_SOURCE_DIR) + relpath;
  }
} // namespace

TEST(YoloWorldTextEmbeddingsBuilderTest, BuildsQuantizedEmbeddingsFromClassNames)
{
  const std::string clip_model = source_path(kClipModelRelPath);
  const std::string clip_vocab = source_path(kClipVocabRelPath);
  const std::string yolo_model = source_path(kYoloModelRelPath);

  if (::access(clip_model.c_str(), R_OK) != 0)
    GTEST_SKIP() << "missing CLIP model asset: " << clip_model;
  if (::access(clip_vocab.c_str(), R_OK) != 0)
    GTEST_SKIP() << "missing CLIP vocab asset: " << clip_vocab;
  if (::access(yolo_model.c_str(), R_OK) != 0)
    GTEST_SKIP() << "missing YOLO model asset: " << yolo_model;

  omniseer::vision::YoloWorldTextEmbeddingsBuilderConfig cfg{};
  cfg.text_encoder_model_path = clip_model;
  cfg.detector_model_path     = yolo_model;
  cfg.clip_vocab_path         = clip_vocab;

  omniseer::vision::YoloWorldTextEmbeddingsBuilder builder(cfg);
  const auto                                       prepared = builder.build({"person", "bus"});
  const auto                                       view     = prepared.view();

  ASSERT_EQ(prepared.class_names.size(), 2u);
  EXPECT_EQ(prepared.class_names[0], "person");
  EXPECT_EQ(prepared.class_names[1], "bus");
  EXPECT_EQ(view.active_class_count, 2u);
  EXPECT_EQ(view.bytes, 80u * 512u);

  const auto row_begin = [&](size_t row)
  { return prepared.text_bytes.begin() + static_cast<std::ptrdiff_t>(row * 512); };
  const auto row_end = [&](size_t row) { return row_begin(row) + 512; };

  EXPECT_FALSE(std::equal(row_begin(0), row_end(0), row_begin(2), row_end(2)));
  EXPECT_FALSE(std::equal(row_begin(1), row_end(1), row_begin(2), row_end(2)));
  EXPECT_TRUE(std::equal(row_begin(2), row_end(2), row_begin(79), row_end(79)));
}

TEST(YoloWorldTextEmbeddingsBuilderTest, AcceptsFloat32DetectorTextInput)
{
  rknn_tensor_attr attr{};
  attr.type = RKNN_TENSOR_FLOAT32;

  EXPECT_EQ(omniseer::vision::resolve_yolo_world_text_input_format(attr),
            omniseer::vision::YoloWorldTextInputFormat::Float32);

  const std::vector<float>                 clip_embedding{0.25F, -1.5F, 3.0F};
  omniseer::vision::PreparedTextEmbeddings prepared{};
  prepared.text_bytes.resize(clip_embedding.size() * sizeof(float));
  omniseer::vision::copy_float32_yolo_world_text_embedding(prepared.text_bytes, 0,
                                                           clip_embedding.data(),
                                                           clip_embedding.size());

  const auto view = prepared.view();
  ASSERT_EQ(view.bytes, clip_embedding.size() * sizeof(float));
  ASSERT_NE(view.data, nullptr);
  EXPECT_EQ(std::memcmp(view.data, clip_embedding.data(), view.bytes), 0);
}

TEST(YoloWorldTextEmbeddingsBuilderTest, RejectsUnsupportedDetectorTextInputType)
{
  rknn_tensor_attr attr{};
  attr.type = RKNN_TENSOR_FLOAT16;

  try
  {
    (void) omniseer::vision::resolve_yolo_world_text_input_format(attr);
    FAIL() << "FLOAT16 detector texts input unexpectedly accepted";
  }
  catch (const std::runtime_error& error)
  {
    EXPECT_NE(std::string(error.what()).find("affine INT8 and FLOAT32"), std::string::npos);
  }
}

TEST(YoloWorldTextEmbeddingsBuilderTest, AcceptsAffineInt8DetectorTextInput)
{
  rknn_tensor_attr attr{};
  attr.type  = RKNN_TENSOR_INT8;
  attr.scale = 0.125F;

  EXPECT_EQ(omniseer::vision::resolve_yolo_world_text_input_format(attr),
            omniseer::vision::YoloWorldTextInputFormat::AffineInt8);
}
