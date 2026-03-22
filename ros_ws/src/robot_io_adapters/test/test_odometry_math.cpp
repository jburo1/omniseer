#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "robot_io_adapters/odometry_math.hpp"

namespace
{

using robot_io_adapters::BodyTwist;
using robot_io_adapters::EncoderOdometryIntegrator;
using robot_io_adapters::PlanarPose;
using robot_io_adapters::WheelCountsSnapshot;

constexpr double kWheelRadiusM = 0.0485;
constexpr double kHalfLengthM = 0.09735;
constexpr double kHalfWidthM = 0.10886705;
constexpr double kEncoderTicksPerRev = 44.0 * 131.0;
constexpr double kPi = 3.14159265358979323846;

std::array<double, 4> wheel_angular_velocities_from_body_twist(double vx, double vy, double wz)
{
  const double k = kHalfLengthM + kHalfWidthM;
  return {
    (vx - vy - (k * wz)) / kWheelRadiusM,
    (vx + vy + (k * wz)) / kWheelRadiusM,
    (vx + vy - (k * wz)) / kWheelRadiusM,
    (vx - vy + (k * wz)) / kWheelRadiusM,
  };
}

int64_t ticks_for_rate(double angular_velocity_rad_s, double dt_s)
{
  const double revolutions = angular_velocity_rad_s * dt_s / (2.0 * kPi);
  return static_cast<int64_t>(std::llround(revolutions * kEncoderTicksPerRev));
}

} // namespace

TEST(OdometryMath, CountsDeltaMatchesOneRevolutionPerSecond)
{
  const double angular_velocity =
    robot_io_adapters::counts_delta_to_wheel_angular_velocity(
          static_cast<int64_t>(kEncoderTicksPerRev), 1.0, kEncoderTicksPerRev);

  EXPECT_NEAR(angular_velocity, 2.0 * kPi, 1.0e-9);
}

TEST(OdometryMath, BodyTwistInverseMatchesFirmwareKinematics)
{
  constexpr double expected_vx = 0.6;
  constexpr double expected_vy = -0.15;
  constexpr double expected_wz = 0.35;

  const auto wheel_rates =
    wheel_angular_velocities_from_body_twist(expected_vx, expected_vy, expected_wz);

  const BodyTwist twist = robot_io_adapters::body_twist_from_wheel_angular_velocities(
      wheel_rates[0], wheel_rates[1], wheel_rates[2], wheel_rates[3], kWheelRadiusM, kHalfLengthM,
    kHalfWidthM);

  EXPECT_NEAR(twist.vx, expected_vx, 1.0e-9);
  EXPECT_NEAR(twist.vy, expected_vy, 1.0e-9);
  EXPECT_NEAR(twist.wz, expected_wz, 1.0e-9);
}

TEST(OdometryMath, IntegratePlanarPoseUsesRobotHeading)
{
  const PlanarPose pose{0.0, 0.0, kPi / 2.0};
  const BodyTwist twist{1.0, 0.0, 0.0};

  const PlanarPose integrated = robot_io_adapters::integrate_planar_pose(pose, twist, 0.5);

  EXPECT_NEAR(integrated.x, 0.0, 1.0e-9);
  EXPECT_NEAR(integrated.y, 0.5, 1.0e-9);
  EXPECT_NEAR(integrated.theta, pose.theta, 1.0e-9);
}

TEST(OdometryMath, IntegratorProducesPoseAndTwistFromEncoderSamples)
{
  EncoderOdometryIntegrator integrator(kWheelRadiusM, kHalfLengthM, kHalfWidthM,
    kEncoderTicksPerRev);

  const auto first = integrator.update(WheelCountsSnapshot{1.0, 1000, 1000, 1000, 1000});
  EXPECT_FALSE(first.has_value());

  constexpr double expected_vx = 0.4;
  constexpr double expected_vy = 0.05;
  constexpr double expected_wz = -0.2;
  constexpr double dt_s = 0.1;
  const auto wheel_rates = wheel_angular_velocities_from_body_twist(expected_vx, expected_vy,
    expected_wz);

  const auto second = integrator.update(WheelCountsSnapshot{
    1.0 + dt_s,
    1000 + ticks_for_rate(wheel_rates[0], dt_s),
    1000 + ticks_for_rate(wheel_rates[1], dt_s),
    1000 + ticks_for_rate(wheel_rates[2], dt_s),
    1000 + ticks_for_rate(wheel_rates[3], dt_s),
  });

  ASSERT_TRUE(second.has_value());
  EXPECT_NEAR(second->twist.vx, expected_vx, 1.0e-3);
  EXPECT_NEAR(second->twist.vy, expected_vy, 1.0e-3);
  EXPECT_NEAR(second->twist.wz, expected_wz, 1.0e-3);
  EXPECT_NEAR(second->pose.x, expected_vx * dt_s, 1.0e-3);
  EXPECT_NEAR(second->pose.y, expected_vy * dt_s, 1.0e-3);
  EXPECT_NEAR(second->pose.theta, expected_wz * dt_s, 1.0e-3);
}
