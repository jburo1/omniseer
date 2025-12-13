#include "omniseer_core/cmd_protocol.hpp"
#include <cstdio>

namespace omniseer_core {

bool parse_cmd_vel_line(const char* line, CmdVel& out) {
    float vx, vy, wz;
    int n = std::sscanf(line, "V %f %f %f", &vx, &vy, &wz);
    if (n == 3) {
        out.vx    = vx;
        out.vy    = vy;
        out.wz    = wz;
        return true;
    }
    return false;
}

} // namespace omniseer_core
