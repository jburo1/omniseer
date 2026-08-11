#include "omniseer_vision_bridge/resolved_vision_config.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <system_error>

namespace omniseer_vision_bridge
{
  namespace
  {
    std::string yaml_string(const std::string& value)
    {
      std::ostringstream out;
      out << '"';
      for (const char character : value)
      {
        switch (character)
        {
        case '\\':
          out << "\\\\";
          break;
        case '"':
          out << "\\\"";
          break;
        case '\n':
          out << "\\n";
          break;
        case '\r':
          out << "\\r";
          break;
        case '\t':
          out << "\\t";
          break;
        default:
          out << character;
          break;
        }
      }
      out << '"';
      return out.str();
    }

    void set_error(std::string* error, const std::string& value)
    {
      if (error != nullptr)
      {
        *error = value;
      }
    }
  } // namespace

  bool write_resolved_vision_config(const VisionBridgeRuntimeConfig& config, std::string* error)
  {
    if (config.resolved_config_path.empty())
    {
      return true;
    }

    const std::filesystem::path destination(config.resolved_config_path);
    std::error_code             filesystem_error;
    if (!destination.parent_path().empty())
    {
      std::filesystem::create_directories(destination.parent_path(), filesystem_error);
      if (filesystem_error)
      {
        set_error(error, "failed to create provenance directory: " + filesystem_error.message());
        return false;
      }
    }

    const auto    temporary = destination.string() + ".tmp";
    std::ofstream output(temporary, std::ios::out | std::ios::trunc);
    if (!output)
    {
      set_error(error, "failed to open resolved configuration artifact");
      return false;
    }

    output << "camera:\n"
           << "  device: " << yaml_string(config.camera_device) << "\n"
           << "  width: " << config.camera_width << "\n"
           << "  height: " << config.camera_height << "\n"
           << "  buffer_count: " << config.camera_buffer_count << "\n"
           << "pipeline:\n"
           << "  dst_width: " << config.pipeline_dst_width << "\n"
           << "  dst_height: " << config.pipeline_dst_height << "\n"
           << "models:\n"
           << "  detector_model_path: " << yaml_string(config.detector_model_path) << "\n"
           << "  clip_model_path: " << yaml_string(config.clip_model_path) << "\n"
           << "  clip_vocab_path: " << yaml_string(config.clip_vocab_path) << "\n"
           << "classes:\n"
           << "  path: " << yaml_string(config.class_list_path) << "\n"
           << "  pad_token: " << yaml_string(config.pad_token) << "\n"
           << "runner:\n"
           << "  warmup_runs: " << config.runner_warmup_runs << "\n"
           << "postprocess:\n"
           << "  score_threshold: " << config.score_threshold << "\n"
           << "  nms_iou_threshold: " << config.nms_iou_threshold << "\n"
           << "  max_detections: " << config.max_detections << "\n"
           << "frames:\n"
           << "  camera_frame_id: " << yaml_string(config.camera_frame_id) << "\n";
    output.close();
    if (!output)
    {
      std::filesystem::remove(temporary, filesystem_error);
      set_error(error, "failed to write resolved configuration artifact");
      return false;
    }

    std::filesystem::rename(temporary, destination, filesystem_error);
    if (filesystem_error)
    {
      std::filesystem::remove(temporary, filesystem_error);
      set_error(error, "failed to finalize resolved configuration artifact: " +
                           filesystem_error.message());
      return false;
    }
    return true;
  }
} // namespace omniseer_vision_bridge
