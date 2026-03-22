#include "robot_io_adapters/odometry_math.hpp"

#include <cmath>
#include <stdexcept>

namespace robot_io_adapters
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

} // namespace

double counts_delta_to_wheel_angular_velocity(
  int64_t delta_counts,
  double  dt_s,
  double  encoder_ticks_per_rev)
{
  if (dt_s <= 0.0) {
    throw std::invalid_argument("dt_s must be positive");
  }
  if (encoder_ticks_per_rev <= 0.0) {
    throw std::invalid_argument("encoder_ticks_per_rev must be positive");
  }

  return (static_cast<double>(delta_counts) * kTwoPi) / (encoder_ticks_per_rev * dt_s);
}

BodyTwist body_twist_from_wheel_angular_velocities(
  double front_left,
  double front_right,
  double rear_left,
  double rear_right,
  double wheel_radius_m,
  double half_length_m,
  double half_width_m)
{
  const double k = half_length_m + half_width_m;
  if (wheel_radius_m <= 0.0) {
    throw std::invalid_argument("wheel_radius_m must be positive");
  }
  if (k <= 0.0) {
    throw std::invalid_argument("half_length_m + half_width_m must be positive");
  }

  const double scale = wheel_radius_m * 0.25;
  return BodyTwist{
    scale * (front_left + front_right + rear_left + rear_right),
    scale * (-front_left + front_right + rear_left - rear_right),
    (scale / k) * (-front_left + front_right - rear_left + rear_right),
  };
}

PlanarPose integrate_planar_pose(
  const PlanarPose & pose,
  const BodyTwist & twist,
  double            dt_s)
{
  if (dt_s <= 0.0) {
    throw std::invalid_argument("dt_s must be positive");
  }

  const double cos_theta = std::cos(pose.theta);
  const double sin_theta = std::sin(pose.theta);

  return PlanarPose{
    pose.x + ((twist.vx * cos_theta - twist.vy * sin_theta) * dt_s),
    pose.y + ((twist.vx * sin_theta + twist.vy * cos_theta) * dt_s),
    normalize_angle(pose.theta + (twist.wz * dt_s)),
  };
}

EncoderOdometryIntegrator::EncoderOdometryIntegrator(
  double wheel_radius_m,
  double half_length_m,
  double half_width_m,
  double encoder_ticks_per_rev)
: _wheel_radius_m(wheel_radius_m),
  _half_length_m(half_length_m),
  _half_width_m(half_width_m),
  _encoder_ticks_per_rev(encoder_ticks_per_rev)
{
  if (_wheel_radius_m <= 0.0) {
    throw std::invalid_argument("wheel_radius_m must be positive");
  }
  if ((_half_length_m + _half_width_m) <= 0.0) {
    throw std::invalid_argument("half_length_m + half_width_m must be positive");
  }
  if (_encoder_ticks_per_rev <= 0.0) {
    throw std::invalid_argument("encoder_ticks_per_rev must be positive");
  }
}

std::optional<EncoderOdometryEstimate> EncoderOdometryIntegrator::update(
  const WheelCountsSnapshot & sample)
{
  if (!_previous_sample.has_value()) {
    _previous_sample = sample;
    return std::nullopt;
  }

  const double dt_s = sample.stamp_s - _previous_sample->stamp_s;
  if (dt_s <= 0.0) {
    throw std::invalid_argument("sample timestamp must move forward");
  }

  const double front_left_rate = counts_delta_to_wheel_angular_velocity(
      sample.front_left - _previous_sample->front_left, dt_s, _encoder_ticks_per_rev);
  const double front_right_rate = counts_delta_to_wheel_angular_velocity(
      sample.front_right - _previous_sample->front_right, dt_s, _encoder_ticks_per_rev);
  const double rear_left_rate = counts_delta_to_wheel_angular_velocity(
      sample.rear_left - _previous_sample->rear_left, dt_s, _encoder_ticks_per_rev);
  const double rear_right_rate = counts_delta_to_wheel_angular_velocity(
      sample.rear_right - _previous_sample->rear_right, dt_s, _encoder_ticks_per_rev);

  const BodyTwist twist = body_twist_from_wheel_angular_velocities(
      front_left_rate,
      front_right_rate,
      rear_left_rate,
      rear_right_rate,
      _wheel_radius_m,
      _half_length_m,
      _half_width_m);

  _pose = integrate_planar_pose(_pose, twist, dt_s);
  _previous_sample = sample;

  return EncoderOdometryEstimate{_pose, twist};
}

} // namespace robot_io_adapters
