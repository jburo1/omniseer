#pragma once

#include <cstdint>
#include <optional>

namespace robot_io_adapters
{

struct WheelCountsSnapshot
{
  double  stamp_s{0.0};
  int64_t front_left{0};
  int64_t front_right{0};
  int64_t rear_left{0};
  int64_t rear_right{0};
};

struct PlanarPose
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

struct BodyTwist
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct EncoderOdometryEstimate
{
  PlanarPose pose{};
  BodyTwist  twist{};
};

double counts_delta_to_wheel_angular_velocity(
  int64_t delta_counts,
  double  dt_s,
  double  encoder_ticks_per_rev);

BodyTwist body_twist_from_wheel_angular_velocities(
  double front_left,
  double front_right,
  double rear_left,
  double rear_right,
  double wheel_radius_m,
  double half_length_m,
  double half_width_m);

PlanarPose integrate_planar_pose(
  const PlanarPose & pose,
  const BodyTwist & twist,
  double            dt_s);

class EncoderOdometryIntegrator
{
public:
  EncoderOdometryIntegrator(
    double wheel_radius_m,
    double half_length_m,
    double half_width_m,
    double encoder_ticks_per_rev);

  std::optional<EncoderOdometryEstimate> update(const WheelCountsSnapshot & sample);

private:
  double _wheel_radius_m{0.0};
  double _half_length_m{0.0};
  double _half_width_m{0.0};
  double _encoder_ticks_per_rev{0.0};
  std::optional<WheelCountsSnapshot> _previous_sample{};
  PlanarPose _pose{};
};

} // namespace robot_io_adapters
