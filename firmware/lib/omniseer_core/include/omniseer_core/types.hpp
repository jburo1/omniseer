#pragma once
#include <cstdint>

namespace omniseer_core {

struct CmdVel {
    float vx;     // m/s, +forward
    float vy;     // m/s, +left
    float wz;     // rad/s, +CCW
};

struct WheelSpeeds {
    float fl;
    float fr;
    float rl;
    float rr;
};

struct WheelEncoderCounts {
    int32_t data[4];       // encoder counts per wheel : FR RR FL RL
    uint32_t timestamp_us;
};

} //namespace omniseer_core
