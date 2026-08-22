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
} // namespace omniseer::vision
