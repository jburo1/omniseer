#include <cstdint>
#include <gtest/gtest.h>
#include <vector>

#include "omniseer/vision/config.hpp"
#include "omniseer/vision/detections.hpp"
#include "omniseer/vision/rknn_runner.hpp"
#include "omniseer/vision/yolo_world_postprocess.hpp"

namespace
{
  using omniseer::vision::ConsumerPipelineConfig;
  using omniseer::vision::Detection;
  using omniseer::vision::DetectionsFrame;
  using omniseer::vision::PipelineRemapConfig;
  using omniseer::vision::RknnOutputDesc;
  using omniseer::vision::RknnOutputView;
  using omniseer::vision::YoloWorldOutputLayout;

  struct FakeTensor
  {
    RknnOutputDesc       desc{};
    std::vector<int8_t> storage{};
    RknnOutputView       view{};
  };

  size_t nchw_bytes(uint32_t c, uint32_t h, uint32_t w)
  {
    return static_cast<size_t>(c) * h * w;
  }

  FakeTensor make_i8_tensor(uint32_t index, uint32_t channels, uint32_t grid, int32_t zp, float scale)
  {
    FakeTensor tensor{};
    tensor.desc.index      = index;
    tensor.desc.n_dims     = 4;
    tensor.desc.dims[0]    = 1;
    tensor.desc.dims[1]    = channels;
    tensor.desc.dims[2]    = grid;
    tensor.desc.dims[3]    = grid;
    tensor.desc.type       = RKNN_TENSOR_INT8;
    tensor.desc.zero_point = zp;
    tensor.desc.scale      = scale;

    tensor.storage.assign(nchw_bytes(channels, grid, grid), static_cast<int8_t>(zp));
    tensor.view.index = index;
    tensor.view.data  = tensor.storage.data();
    tensor.view.bytes = tensor.storage.size();
    return tensor;
  }

  void set_class_score(FakeTensor& tensor, uint32_t class_id, uint32_t y, uint32_t x, int8_t value)
  {
    const uint32_t grid     = tensor.desc.dims[2];
    const size_t   grid_len = static_cast<size_t>(grid) * grid;
    tensor.storage[static_cast<size_t>(y) * grid + x + static_cast<size_t>(class_id) * grid_len] = value;
  }

  void set_box(FakeTensor& tensor, uint32_t y, uint32_t x, int8_t left, int8_t top, int8_t right, int8_t bottom)
  {
    const uint32_t grid     = tensor.desc.dims[2];
    const size_t   grid_len = static_cast<size_t>(grid) * grid;
    const size_t   cell     = static_cast<size_t>(y) * grid + x;
    tensor.storage[cell + 0u * grid_len] = left;
    tensor.storage[cell + 1u * grid_len] = top;
    tensor.storage[cell + 2u * grid_len] = right;
    tensor.storage[cell + 3u * grid_len] = bottom;
  }
} // namespace

TEST(YoloWorldPostprocessTest, DecodesOneInt8DetectionIntoSourceSpace)
{
  std::vector<FakeTensor> tensors{};
  tensors.reserve(6);
  tensors.push_back(make_i8_tensor(0, 80, 80, -128, 0.01F));
  tensors.push_back(make_i8_tensor(1, 4, 80, -128, 0.50F));
  tensors.push_back(make_i8_tensor(2, 80, 40, -128, 0.01F));
  tensors.push_back(make_i8_tensor(3, 4, 40, -128, 0.50F));
  tensors.push_back(make_i8_tensor(4, 80, 20, -128, 0.01F));
  tensors.push_back(make_i8_tensor(5, 4, 20, -128, 0.50F));

  set_class_score(tensors[4], 2, 8, 8, -48);
  set_box(tensors[5], 8, 8, -124, -126, -122, -120);

  std::vector<RknnOutputDesc> descs{};
  std::vector<RknnOutputView> views{};
  descs.reserve(tensors.size());
  views.reserve(tensors.size());
  for (const FakeTensor& tensor : tensors)
  {
    descs.push_back(tensor.desc);
    views.push_back(tensor.view);
  }

  const YoloWorldOutputLayout layout = omniseer::vision::resolve_yolo_world_output_layout(descs);

  ConsumerPipelineConfig cfg{};
  cfg.score_threshold = 0.25F;
  cfg.nms_iou_threshold = 0.45F;
  cfg.max_detections = 100;

  PipelineRemapConfig remap{};
  remap.source_size      = {.w = 1280, .h = 720};
  remap.model_input_size = {.w = 640, .h = 640};
  remap.scale            = 0.5F;
  remap.pad_x            = 0;
  remap.pad_y            = 140;

  DetectionsFrame frame{};
  frame.source_size        = remap.source_size;
  frame.active_class_count = 3;

  omniseer::vision::decode_yolo_world_detections(views, descs, layout, cfg, remap, 3, frame);

  ASSERT_EQ(frame.count, 1u);
  const Detection& det = frame.detections[0];
  EXPECT_EQ(det.class_id, 2u);
  EXPECT_NEAR(det.score, 0.80F, 1e-4F);
  EXPECT_NEAR(det.x1, 416.0F, 1e-4F);
  EXPECT_NEAR(det.y1, 200.0F, 1e-4F);
  EXPECT_NEAR(det.x2, 736.0F, 1e-4F);
  EXPECT_NEAR(det.y2, 520.0F, 1e-4F);
}
