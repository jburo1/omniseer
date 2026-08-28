#include "omniseer/vision/runbundle_comparison.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

#include "omniseer/vision/preview_wrap_repair.hpp"

namespace omniseer::vision
{
  namespace
  {
    void append_json_string(std::ostream& out, const std::string& value)
    {
      out.put('"');
      for (const unsigned char ch : value)
      {
        switch (ch)
        {
        case '"':
          out << "\\\"";
          break;
        case '\\':
          out << "\\\\";
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
          if (ch < 0x20U)
            out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                << static_cast<unsigned int>(ch) << std::dec << std::setfill(' ');
          else
            out.put(static_cast<char>(ch));
        }
      }
      out.put('"');
    }
  } // namespace

  double comparison_output_fps(double input_fps) noexcept
  {
    return (std::isfinite(input_fps) && input_fps > 0.0) ? input_fps : 30.0;
  }

  bool comparison_name_is_safe(const std::string& name) noexcept
  {
    if (name.empty() || name.size() > 64 || name == "." || name == "..")
      return false;
    for (const unsigned char ch : name)
    {
      const bool ascii_alphanumeric =
          (ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9');
      if (!(ascii_alphanumeric || ch == '_' || ch == '-'))
        return false;
    }
    return true;
  }

  double comparison_source_timestamp_sec(double position_msec, uint64_t frame_index,
                                         double source_fps) noexcept
  {
    const double timestamp_sec = position_msec / 1000.0;
    if (std::isfinite(timestamp_sec) && timestamp_sec >= 0.0)
      return timestamp_sec;
    return (std::isfinite(source_fps) && source_fps > 0.0)
               ? static_cast<double>(frame_index) / source_fps
               : 0.0;
  }

  ComparisonInputPaths resolve_comparison_input_paths(const std::filesystem::path& run_dir,
                                                      const std::filesystem::path& repo_root,
                                                      const std::filesystem::path& model_dir,
                                                      const std::filesystem::path& class_list_path)
  {
    return {
        .source_path     = run_dir / "video" / "source.ts",
        .model_dir       = model_dir.empty() ? repo_root / "runs" / "model_artifacts" : model_dir,
        .class_list_path = class_list_path.empty() ? run_dir / "classes.txt" : class_list_path,
    };
  }

  std::string comparison_provenance_json(const ComparisonProvenance& provenance)
  {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::fixed << std::setprecision(6);
    out << "{\n  \"schema_version\": 2,\n  \"comparison_name\": ";
    append_json_string(out, provenance.comparison_name);
    out << ",\n  \"source\": {\"path\": ";
    append_json_string(out, provenance.source_path);
    out << ", \"sha256\": ";
    append_json_string(out, provenance.source_sha256);
    out << "},\n  \"repair\": {\"id\": \"rockchip_preview_circular_wrap_v1\", "
           "\"recorded_order\": \"[original x=1272..1279][original x=0..1271]\", "
           "\"width\": "
        << kRockchipPreviewWrapRepairWidth << ", \"height\": " << kRockchipPreviewWrapRepairHeight
        << ", \"rotate_left_px\": " << kRockchipPreviewWrapRepairPixels << "},\n  \"classes\": [";
    for (size_t i = 0; i < provenance.classes.size(); ++i)
    {
      if (i != 0)
        out << ", ";
      append_json_string(out, provenance.classes[i]);
    }
    out << "],\n  \"postprocess\": {\"score_threshold\": " << provenance.score_threshold
        << ", \"nms_iou_threshold\": " << provenance.nms_iou_threshold
        << ", \"max_detections\": " << provenance.max_detections << "},\n  \"models\": [";
    for (size_t i = 0; i < provenance.models.size(); ++i)
    {
      const auto& model = provenance.models[i];
      if (i != 0)
        out << ',';
      out << "\n    {\"label\": ";
      append_json_string(out, model.label);
      out << ", \"artifact_name\": ";
      append_json_string(out, model.artifact_name);
      out << ", \"path\": ";
      append_json_string(out, model.path);
      out << ", \"sha256\": ";
      append_json_string(out, model.sha256);
      out << '}';
    }
    out << "\n  ],\n  \"detection_jsonl\": [";
    for (size_t i = 0; i < provenance.detection_jsonl_paths.size(); ++i)
    {
      if (i != 0)
        out << ", ";
      append_json_string(out, provenance.detection_jsonl_paths[i]);
    }
    out << "],\n  \"output\": {\"path\": ";
    append_json_string(out, provenance.output_path);
    out << ", \"sha256\": ";
    append_json_string(out, provenance.output_sha256);
    out << ", \"fps\": " << provenance.output_fps
        << ", \"source_frames_rendered\": " << provenance.source_frames_rendered
        << ", \"timing_policy\": \"one output frame per decoded source frame at source FPS; "
           "offline processing duration does not change presentation elapsed time\"},\n"
           "  \"git_sha\": ";
    append_json_string(out, provenance.git_sha);
    out << "\n}\n";
    return out.str();
  }
} // namespace omniseer::vision
