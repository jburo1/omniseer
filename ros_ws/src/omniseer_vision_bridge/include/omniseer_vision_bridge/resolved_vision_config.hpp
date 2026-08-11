#pragma once

#include <string>

#include "omniseer_vision_bridge/vision_bridge_runtime.hpp"

namespace omniseer_vision_bridge
{
  // Writes the effective bridge parameters after ROS has applied defaults, YAML, and launch
  // overrides.
  bool write_resolved_vision_config(const VisionBridgeRuntimeConfig& config,
                                    std::string*                     error = nullptr);
} // namespace omniseer_vision_bridge
