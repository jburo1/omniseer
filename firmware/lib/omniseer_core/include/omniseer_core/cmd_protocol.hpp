#pragma once
#include "omniseer_core/types.hpp"

namespace omniseer_core {

// parses lines: "V vx vy wz"
bool parse_cmd_vel_line(const char* line, CmdVel& out);

} // namespace omniseer_core
