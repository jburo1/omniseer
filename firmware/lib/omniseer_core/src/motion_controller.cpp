#include "omniseer_core/motion_controller.hpp"

namespace omniseer_core {

MotionController::MotionController(const MecanumKinematics& kin,
                                   uint32_t cmd_timeout_us)
    : _kin(kin),
      _cmd_timeout_us(cmd_timeout_us)
{}

void MotionController::set_cmd_vel(const CmdVel& cmd, uint32_t now_us) {
    _cmd = cmd;
    _prev_cmd_time_us = now_us;
}

void MotionController::update(uint32_t now_us, WheelSpeeds& out) {
    CmdVel effective = _cmd;
    if (now_us - _prev_cmd_time_us > _cmd_timeout_us) {
        effective.vx    = 0.0f;
        effective.vy    = 0.0f;
        effective.wz    = 0.0f;
    }

    _kin.compute(effective, out);
}

} // namespace omniseer_core
