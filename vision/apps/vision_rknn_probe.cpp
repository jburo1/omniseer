#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "omniseer/vision/class_list.hpp"
#include "omniseer/vision/cpu_letterbox_preprocess.hpp"
#include "omniseer/vision/dma_heap_alloc.hpp"
#include "omniseer/vision/image_buffer_pool.hpp"
#include "omniseer/vision/letterbox.hpp"
#include "omniseer/vision/rknn_runner.hpp"
#include "omniseer/vision/yolo_world_text_embeddings.hpp"

namespace
{
  constexpr omniseer::vision::Size kModelInputSize{640, 640};

  struct Config
  {
    std::string              detector_model{};
    std::string              image{};
    std::string              classes_file{};
    std::vector<std::string> classes{};
    std::string              clip_model{};
    std::string              clip_vocab{};
    std::string              output_dir{};
  };

  const char* usage(const char* argv0)
  {
    static const std::string text =
        std::string("Usage: ") + argv0 +
        " --detector-model <model.rknn> --image <image>\\n"
        "       (--classes <classes.txt> | --class <name> [...])\\n"
        "       --clip-model <clip.rknn> --clip-vocab <vocab.bpe>\\n"
        "       --output-dir <dir>\\n"
        "\\nRuns one CPU-letterboxed image through RKNN and writes raw output_<index>.bin\\n"
        "files plus metadata.json. It does not decode detections or run NMS.\\n";
    return text.c_str();
  }

  std::string require_value(int& index, int argc, char** argv, const char* option)
  {
    if (++index >= argc)
      throw std::invalid_argument(std::string(option) + " requires a value");
    return argv[index];
  }

  Config parse_args(int argc, char** argv, bool& help)
  {
    Config config{};
    help = false;
    for (int i = 1; i < argc; ++i)
    {
      const std::string arg = argv[i];
      if (arg == "--help" || arg == "-h")
      {
        help = true;
        return config;
      }
      if (arg == "--detector-model")
        config.detector_model = require_value(i, argc, argv, "--detector-model");
      else if (arg == "--image")
        config.image = require_value(i, argc, argv, "--image");
      else if (arg == "--classes")
        config.classes_file = require_value(i, argc, argv, "--classes");
      else if (arg == "--class")
        config.classes.push_back(require_value(i, argc, argv, "--class"));
      else if (arg == "--clip-model")
        config.clip_model = require_value(i, argc, argv, "--clip-model");
      else if (arg == "--clip-vocab")
        config.clip_vocab = require_value(i, argc, argv, "--clip-vocab");
      else if (arg == "--output-dir")
        config.output_dir = require_value(i, argc, argv, "--output-dir");
      else
        throw std::invalid_argument("unknown option: " + arg);
    }
    if (config.detector_model.empty() || config.image.empty() || config.clip_model.empty() ||
        config.clip_vocab.empty() || config.output_dir.empty())
      throw std::invalid_argument(
          "detector model, image, CLIP model, CLIP vocab, and output directory are required");
    if (config.classes_file.empty() == config.classes.empty())
      throw std::invalid_argument("provide exactly one of --classes or --class");
    return config;
  }

  const char* dtype_name(rknn_tensor_type type)
  {
    switch (type)
    {
    case RKNN_TENSOR_FLOAT32:
      return "float32";
    case RKNN_TENSOR_FLOAT16:
      return "float16";
    case RKNN_TENSOR_INT8:
      return "int8";
    case RKNN_TENSOR_UINT8:
      return "uint8";
    case RKNN_TENSOR_INT16:
      return "int16";
    case RKNN_TENSOR_UINT16:
      return "uint16";
    case RKNN_TENSOR_INT32:
      return "int32";
    case RKNN_TENSOR_UINT32:
      return "uint32";
    case RKNN_TENSOR_BOOL:
      return "bool";
    default:
      return "unknown";
    }
  }

  const char* quantization_name(rknn_tensor_qnt_type type)
  {
    switch (type)
    {
    case RKNN_TENSOR_QNT_NONE:
      return "none";
    case RKNN_TENSOR_QNT_DFP:
      return "dfp";
    case RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC:
      return "affine_asymmetric";
    default:
      return "unknown";
    }
  }

  float half_to_float(uint16_t value)
  {
    const uint32_t sign     = static_cast<uint32_t>(value & 0x8000U) << 16U;
    const uint32_t exponent = (value >> 10U) & 0x1fU;
    uint32_t       mantissa = value & 0x03ffU;
    uint32_t       bits     = 0;
    if (exponent == 0)
    {
      if (mantissa != 0)
      {
        uint32_t shift = 0;
        while ((mantissa & 0x0400U) == 0)
        {
          mantissa <<= 1U;
          ++shift;
        }
        bits = sign | ((127U - 14U - shift) << 23U) | ((mantissa & 0x03ffU) << 13U);
      }
      else
        bits = sign;
    }
    else if (exponent == 31U)
      bits = sign | 0x7f800000U | (mantissa << 13U);
    else
      bits = sign | ((exponent + 112U) << 23U) | (mantissa << 13U);
    float value_f{};
    std::memcpy(&value_f, &bits, sizeof(value_f));
    return value_f;
  }

  double element_value(const uint8_t* data, size_t index, rknn_tensor_type type)
  {
    switch (type)
    {
    case RKNN_TENSOR_INT8:
      return static_cast<const int8_t*>(static_cast<const void*>(data))[index];
    case RKNN_TENSOR_UINT8:
      return static_cast<double>(data[index]);
    case RKNN_TENSOR_INT16:
      return static_cast<const int16_t*>(static_cast<const void*>(data))[index];
    case RKNN_TENSOR_UINT16:
      return static_cast<const uint16_t*>(static_cast<const void*>(data))[index];
    case RKNN_TENSOR_INT32:
      return static_cast<const int32_t*>(static_cast<const void*>(data))[index];
    case RKNN_TENSOR_UINT32:
      return static_cast<const uint32_t*>(static_cast<const void*>(data))[index];
    case RKNN_TENSOR_BOOL:
      return static_cast<double>(data[index]);
    case RKNN_TENSOR_FLOAT16:
      return half_to_float(static_cast<const uint16_t*>(static_cast<const void*>(data))[index]);
    case RKNN_TENSOR_FLOAT32:
      return static_cast<const float*>(static_cast<const void*>(data))[index];
    default:
      throw std::runtime_error("unsupported output dtype for statistics");
    }
  }

  size_t element_size(rknn_tensor_type type)
  {
    switch (type)
    {
    case RKNN_TENSOR_INT8:
    case RKNN_TENSOR_UINT8:
      return 1;
    case RKNN_TENSOR_INT16:
    case RKNN_TENSOR_UINT16:
    case RKNN_TENSOR_FLOAT16:
      return 2;
    case RKNN_TENSOR_FLOAT32:
      return 4;
    case RKNN_TENSOR_INT32:
    case RKNN_TENSOR_UINT32:
      return 4;
    case RKNN_TENSOR_BOOL:
      return 1;
    default:
      return 0;
    }
  }

  void json_string(std::ostream& out, const std::string& value)
  {
    out << '"';
    for (const char ch : value)
    {
      if (ch == '"' || ch == '\\')
        out << '\\';
      if (static_cast<unsigned char>(ch) < 0x20U)
        out << '?';
      else
        out << ch;
    }
    out << '"';
  }

  void write_outputs(const std::filesystem::path&                         directory,
                     const std::vector<omniseer::vision::RknnOutputView>& outputs,
                     const std::vector<omniseer::vision::RknnOutputDesc>& descriptions)
  {
    if (outputs.size() != descriptions.size())
      throw std::runtime_error("RKNN output views and metadata differ in count");
    std::ofstream metadata(directory / "metadata.json");
    if (!metadata)
      throw std::runtime_error("failed to create output metadata");
    metadata << "{\n  \"outputs\": [\n";
    for (size_t i = 0; i < outputs.size(); ++i)
    {
      const auto&                 output = outputs[i];
      const auto&                 desc   = descriptions[i];
      const std::filesystem::path dump =
          directory / ("output_" + std::to_string(output.index) + ".bin");
      std::ofstream raw(dump, std::ios::binary);
      if (!raw)
        throw std::runtime_error("failed to create raw output dump: " + dump.string());
      raw.write(static_cast<const char*>(output.data), static_cast<std::streamsize>(output.bytes));
      if (!raw)
        throw std::runtime_error("failed to write raw output dump: " + dump.string());

      const size_t bytes_per_element = element_size(desc.type);
      if (bytes_per_element == 0 || output.bytes % bytes_per_element != 0)
        throw std::runtime_error("output byte size is incompatible with its dtype");
      const size_t element_count = output.bytes / bytes_per_element;
      double       minimum       = std::numeric_limits<double>::infinity();
      double       maximum       = -std::numeric_limits<double>::infinity();
      double       sum           = 0.0;
      for (size_t element = 0; element < element_count; ++element)
      {
        double value = element_value(static_cast<const uint8_t*>(output.data), element, desc.type);
        if (desc.quantization == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC)
          value = (value - static_cast<double>(desc.zero_point)) * static_cast<double>(desc.scale);
        minimum = std::min(minimum, value);
        maximum = std::max(maximum, value);
        sum += value;
      }

      metadata << "    {\"index\": " << desc.index << ", \"name\": ";
      json_string(metadata, desc.name);
      metadata << ", \"file\": ";
      json_string(metadata, dump.filename().string());
      metadata << ", \"shape\": [";
      for (uint32_t dim = 0; dim < desc.n_dims; ++dim)
        metadata << (dim == 0 ? "" : ", ") << desc.dims[dim];
      metadata << "], \"dtype\": ";
      json_string(metadata, dtype_name(desc.type));
      metadata << ", \"quantization\": ";
      json_string(metadata, quantization_name(desc.quantization));
      metadata << ", \"scale\": " << desc.scale << ", \"zero_point\": " << desc.zero_point
               << ", \"dequantized_min\": " << minimum << ", \"dequantized_max\": " << maximum
               << ", \"dequantized_mean\": " << (sum / static_cast<double>(element_count)) << "}"
               << (i + 1 == outputs.size() ? "\n" : ",\n");
    }
    metadata << "  ]\n}\n";
  }
} // namespace

int main(int argc, char** argv)
{
  try
  {
    bool         help   = false;
    const Config config = parse_args(argc, argv, help);
    if (help)
    {
      std::fputs(usage(argv[0]), stdout);
      return 0;
    }
    const std::vector<std::string> classes =
        config.classes_file.empty() ? config.classes
                                    : omniseer::vision::load_class_list_file(config.classes_file);
    const cv::Mat source_bgr = cv::imread(config.image, cv::IMREAD_COLOR);
    if (source_bgr.empty())
      throw std::runtime_error("failed to load image: " + config.image);

    omniseer::vision::PreparedTextEmbeddings embeddings =
        omniseer::vision::YoloWorldTextEmbeddingsBuilder(
            {
                .text_encoder_model_path = config.clip_model,
                .detector_model_path     = config.detector_model,
                .clip_vocab_path         = config.clip_vocab,
            })
            .build(classes);
    omniseer::vision::DmaHeapAllocator allocator{};
    omniseer::vision::ImageBufferPool  pool{};
    pool.allocate_all(allocator, kModelInputSize.w, kModelInputSize.h,
                      omniseer::vision::PixelFormat::RGB888);
    omniseer::vision::RknnRunner runner({.model_path = config.detector_model, .warmup_runs = 0});
    runner.preflight(pool, embeddings.text_bytes.data(), embeddings.text_bytes.size());

    auto lease = pool.acquire_write_lease();
    if (!lease)
      throw std::runtime_error("no writable image buffer available");
    const auto remap =
        omniseer::vision::make_letterbox_remap({source_bgr.cols, source_bgr.rows}, kModelInputSize);
    omniseer::vision::cpu_letterbox_bgr_to_rgb(source_bgr, lease->buffer(), remap);
    const int pool_index = lease->index();
    lease->publish();

    const omniseer::vision::InferResult result = runner.infer(pool_index);
    if (!result.ok())
      throw std::runtime_error(
          "RKNN inference failed: status=" + std::to_string(static_cast<int>(result.status)) +
          " rknn_code=" + std::to_string(result.rknn_code));
    const std::filesystem::path output_dir(config.output_dir);
    std::filesystem::create_directories(output_dir);
    write_outputs(output_dir, runner.outputs(), runner.output_descs());
    std::fprintf(stdout, "vision_rknn_probe: wrote %zu raw outputs to %s\n",
                 runner.outputs().size(), output_dir.c_str());
    return 0;
  }
  catch (const std::exception& error)
  {
    std::fprintf(stderr, "vision_rknn_probe: %s\n%s", error.what(), usage(argv[0]));
    return 1;
  }
}
