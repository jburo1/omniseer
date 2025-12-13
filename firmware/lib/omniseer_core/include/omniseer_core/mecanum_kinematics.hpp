#pragma once
#include "types.hpp"

namespace omniseer_core {

class MecanumKinematics {
public:
    MecanumKinematics(float wheel_radius,
                      float half_length,
                      float half_width);

    void compute(const CmdVel& cmd, WheelSpeeds& out) const;

private:
    float _R;
    float _k;    // L + W
};

} // namespace omniseer_core
