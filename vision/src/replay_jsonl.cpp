#include "omniseer/vision/replay_jsonl.hpp"

#include <cmath>
#include <iomanip>
#include <locale>
#include <stdexcept>
#include <utility>

namespace omniseer::vision
{
  namespace
  {
    void write_json_string(std::ostream& out, const std::string& value)
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
        case '\b':
          out << "\\b";
          break;
        case '\f':
          out << "\\f";
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
          {
            const auto flags = out.flags();
            const char fill  = out.fill();
            out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                << static_cast<unsigned int>(ch);
            out.flags(flags);
            out.fill(fill);
          }
          else
          {
            out.put(static_cast<char>(ch));
          }
          break;
        }
      }
      out.put('"');
    }

    void write_number(std::ostream& out, double value)
    {
      if (!std::isfinite(value))
        throw std::invalid_argument("replay JSONL values must be finite");
      out << std::fixed << std::setprecision(9) << value;
    }
  } // namespace

  ReplayJsonlWriter::ReplayJsonlWriter(std::string              output_path,
                                       std::vector<std::string> class_names)
      : _output(output_path, std::ios::out | std::ios::trunc), _class_names(std::move(class_names))
  {
    if (!_output)
      throw std::runtime_error("failed to open replay JSONL output: " + output_path);
    _output.imbue(std::locale::classic());
  }

  void ReplayJsonlWriter::write(uint64_t frame_index, double timestamp_sec,
                                const DetectionsFrame& frame)
  {
    if (frame.count > DetectionsFrame::capacity)
      throw std::invalid_argument("replay detection count exceeds frame capacity");

    _output << "{\"frame_index\":" << frame_index << ",\"timestamp_sec\":";
    write_number(_output, timestamp_sec);
    _output << ",\"detections\":[";

    for (uint32_t i = 0; i < frame.count; ++i)
    {
      if (i != 0)
        _output.put(',');

      const Detection&  detection  = frame.detections[i];
      const std::string class_name = (detection.class_id < _class_names.size())
                                         ? _class_names[detection.class_id]
                                         : "<out-of-range>";
      _output << "{\"class_id\":" << detection.class_id << ",\"class_name\":";
      write_json_string(_output, class_name);
      _output << ",\"score\":";
      write_number(_output, detection.score);
      _output << ",\"bbox\":[";
      write_number(_output, detection.x1);
      _output.put(',');
      write_number(_output, detection.y1);
      _output.put(',');
      write_number(_output, detection.x2);
      _output.put(',');
      write_number(_output, detection.y2);
      _output << "]}";
    }

    _output << "]}\n";
    _output.flush();
    if (!_output)
      throw std::runtime_error("failed to write replay JSONL output");
  }
} // namespace omniseer::vision
