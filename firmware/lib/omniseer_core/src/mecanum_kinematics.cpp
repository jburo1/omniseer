#include "omniseer_core/mecanum_kinematics.hpp"

namespace omniseer_core {

MecanumKinematics::MecanumKinematics(float wheel_radius,
                                     float half_length,
                                     float half_width)
    : _R(wheel_radius),
      _k(half_length + half_width)
{}

void MecanumKinematics::compute(const CmdVel& c, WheelSpeeds& w) const {
    // rad/s
    w.fl = ( c.vx - c.vy - _k * c.wz ) / _R;
    w.fr = ( c.vx + c.vy + _k * c.wz ) / _R;
    w.rl = ( c.vx + c.vy - _k * c.wz ) / _R;
    w.rr = ( c.vx - c.vy + _k * c.wz ) / _R;
}

} // namespace omniseer_core
