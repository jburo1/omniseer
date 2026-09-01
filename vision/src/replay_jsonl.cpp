#include "omniseer/vision/replay_jsonl.hpp"

#include <cmath>
#include <iomanip>
#include <locale>
#include <regex>
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

  ReplayJsonlReader::ReplayJsonlReader(std::string input_path)
      : _input(input_path), _input_path(std::move(input_path))
  {
    if (!_input)
      throw std::runtime_error("failed to open replay JSONL input: " + _input_path);
    _input.imbue(std::locale::classic());
  }

  DetectionsFrame ReplayJsonlReader::read(uint64_t expected_frame_index)
  {
    std::string line{};
    if (!std::getline(_input, line))
      throw std::runtime_error("replay JSONL ended before source frame " +
                               std::to_string(expected_frame_index) + ": " + _input_path);

    static const std::regex frame_index_pattern(R"("frame_index":([0-9]+))");
    std::smatch             frame_index_match{};
    if (!std::regex_search(line, frame_index_match, frame_index_pattern))
      throw std::runtime_error("replay JSONL record has no frame_index: " + _input_path);
    const uint64_t frame_index = std::stoull(frame_index_match[1].str());
    if (frame_index != expected_frame_index)
      throw std::runtime_error("replay JSONL frame_index does not match source frame in " +
                               _input_path);

    static const std::regex detection_pattern(
        R"(\{"class_id":([0-9]+),"class_name":"(?:\\.|[^"])*","score":([-+0-9.eE]+),"bbox":\[([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)\]\})");
    const size_t detections_start = line.find("\"detections\":[");
    const size_t detections_end   = line.rfind("]}");
    if (detections_start == std::string::npos || detections_end == std::string::npos ||
        detections_end < detections_start)
      throw std::runtime_error("replay JSONL record has no detections array: " + _input_path);

    DetectionsFrame frame{};
    size_t          unparsed_start = detections_start + std::string("\"detections\":[").size();
    for (std::sregex_iterator it(line.begin(), line.end(), detection_pattern), end; it != end; ++it)
    {
      if (frame.count == DetectionsFrame::capacity)
        throw std::runtime_error("replay JSONL detection count exceeds frame capacity: " +
                                 _input_path);
      const std::smatch& match       = *it;
      const size_t       match_start = static_cast<size_t>(match.position());
      if (match_start < unparsed_start || match_start > detections_end)
        throw std::runtime_error("replay JSONL contains an invalid detection: " + _input_path);
      for (size_t i = unparsed_start; i < match_start; ++i)
      {
        if (line[i] != ',')
          throw std::runtime_error("replay JSONL contains an invalid detection: " + _input_path);
      }
      Detection&          detection = frame.detections[frame.count++];
      const unsigned long class_id  = std::stoul(match[1].str());
      if (class_id > UINT32_MAX)
        throw std::runtime_error("replay JSONL class_id is out of range: " + _input_path);
      detection.class_id = static_cast<uint32_t>(class_id);
      detection.score    = std::stof(match[2].str());
      detection.x1       = std::stof(match[3].str());
      detection.y1       = std::stof(match[4].str());
      detection.x2       = std::stof(match[5].str());
      detection.y2       = std::stof(match[6].str());
      if (!std::isfinite(detection.score) || !std::isfinite(detection.x1) ||
          !std::isfinite(detection.y1) || !std::isfinite(detection.x2) ||
          !std::isfinite(detection.y2))
        throw std::runtime_error("replay JSONL contains a non-finite detection: " + _input_path);
      unparsed_start = match_start + static_cast<size_t>(match.length());
    }
    for (size_t i = unparsed_start; i < detections_end; ++i)
    {
      if (line[i] != ',')
        throw std::runtime_error("replay JSONL contains an invalid detection: " + _input_path);
    }
    return frame;
  }
} // namespace omniseer::vision
