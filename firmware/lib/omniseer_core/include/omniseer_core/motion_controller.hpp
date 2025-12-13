// Given time and last cmd_vel, output wheel speeds

#pragma once
#include <cstdint>
#include "types.hpp"
#include "omniseer_core/mecanum_kinematics.hpp"
#include "omniseer_core/types.hpp"

namespace omniseer_core {

class MotionController {
public:
    MotionController(const MecanumKinematics& kin,
                     uint32_t cmd_timeout_us);

    // called whenever a new command arrives
    void set_cmd_vel(const CmdVel& cmd, uint32_t now_us);

    // called at fixed rate from loop()
    // sets wheel speeds in rad/s into 'out'(from Serial or ROS)
    void update(uint32_t now_us, WheelSpeeds& out);

private:
    MecanumKinematics _kin;
    CmdVel _cmd{0.0f,0.0f,0.0f};
    uint32_t _prev_cmd_time_us{0};
    uint32_t _cmd_timeout_us;
};

} // namespace omniseer_core
