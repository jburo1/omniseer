#pragma once

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include "omniseer/vision/detections.hpp"

namespace omniseer::vision
{
  /**
   * @brief Deterministic JSONL writer for source-video replay detections.
   */
  class ReplayJsonlWriter
  {
  public:
    ReplayJsonlWriter(std::string output_path, std::vector<std::string> class_names);

    ReplayJsonlWriter(const ReplayJsonlWriter&)            = delete;
    ReplayJsonlWriter& operator=(const ReplayJsonlWriter&) = delete;

    /**
     * @brief Write exactly one canonical record for a decoded source frame.
     *
     * @throws std::runtime_error if output cannot be written.
     */
    void write(uint64_t frame_index, double timestamp_sec, const DetectionsFrame& frame);

  private:
    std::ofstream            _output{};
    std::vector<std::string> _class_names{};
  };

  /** @brief Sequential reader for ReplayJsonlWriter's canonical source-frame records. */
  class ReplayJsonlReader
  {
  public:
    explicit ReplayJsonlReader(std::string input_path);

    ReplayJsonlReader(const ReplayJsonlReader&)            = delete;
    ReplayJsonlReader& operator=(const ReplayJsonlReader&) = delete;

    /** @brief Return detections for exactly the requested source frame. */
    DetectionsFrame read(uint64_t expected_frame_index);

  private:
    std::ifstream _input{};
    std::string   _input_path{};
  };
} // namespace omniseer::vision
