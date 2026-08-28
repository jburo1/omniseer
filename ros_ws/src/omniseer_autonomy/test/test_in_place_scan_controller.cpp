#include <cmath>

#include <gtest/gtest.h>

#include "omniseer_autonomy/in_place_scan_controller.hpp"

namespace omniseer_autonomy
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}

TEST(InPlaceScanController, AccumulatesWrappedYawAcrossPiBoundary)
  {
    InPlaceScanController controller({0.20, 1.0});

    controller.update_heading(kPi - 0.01);
    controller.update_heading(-kPi + 0.01);

    EXPECT_NEAR(controller.yaw_travel_rad(), 0.02, 1.0e-9);
}

TEST(InPlaceScanController, CompletesOneRevolutionAndCommandsZero)
  {
    InPlaceScanController controller({0.20, 1.0});

    controller.update_heading(0.0);
    for (int heading = 1; heading <= 8; ++heading) {
    controller.update_heading(static_cast<double>(heading) * kPi / 4.0);
    }

    EXPECT_TRUE(controller.complete());
    EXPECT_GE(controller.yaw_travel_rad(), 2.0 * kPi);
    EXPECT_DOUBLE_EQ(controller.command().linear_x_m_s, 0.0);
    EXPECT_DOUBLE_EQ(controller.command().angular_z_rad_s, 0.0);
}
} // namespace omniseer_autonomy
