#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "robot_io_adapters/scan_range_conversion.hpp"

namespace
{

sensor_msgs::msg::LaserScan make_scan()
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp.sec = 12;
  scan.header.stamp.nanosec = 345;
  scan.header.frame_id = "sonar_frame";
  scan.angle_min = -0.1F;
  scan.angle_max = 0.2F;
  scan.range_min = 0.02F;
  scan.range_max = 4.0F;
  return scan;
}

} // namespace

TEST(ScanRangeConversion, CopiesRangeContractFields)
{
  auto scan = make_scan();
  scan.ranges = {1.5F};

  const auto range = robot_io_adapters::laser_scan_to_range(scan);

  EXPECT_EQ(range.header.stamp.sec, scan.header.stamp.sec);
  EXPECT_EQ(range.header.stamp.nanosec, scan.header.stamp.nanosec);
  EXPECT_EQ(range.header.frame_id, scan.header.frame_id);
  EXPECT_EQ(range.radiation_type, sensor_msgs::msg::Range::ULTRASOUND);
  EXPECT_FLOAT_EQ(range.field_of_view, 0.3F);
  EXPECT_FLOAT_EQ(range.min_range, scan.range_min);
  EXPECT_FLOAT_EQ(range.max_range, scan.range_max);
}

TEST(ScanRangeConversion, ChoosesNearestFiniteInRangeRay)
{
  auto scan = make_scan();
  scan.ranges = {
    std::numeric_limits<float>::quiet_NaN(),
    3.0F,
    std::numeric_limits<float>::infinity(),
    0.01F,
    0.75F,
    4.5F,
  };

  const auto range = robot_io_adapters::laser_scan_to_range(scan);

  EXPECT_FLOAT_EQ(range.range, 0.75F);
}

TEST(ScanRangeConversion, PublishesInfinityWhenNoRaysAreValid)
{
  auto scan = make_scan();
  scan.ranges = {
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::infinity(),
    -1.0F,
    10.0F,
  };

  const auto range = robot_io_adapters::laser_scan_to_range(scan);

  EXPECT_TRUE(std::isinf(range.range));
  EXPECT_GT(range.range, 0.0F);
}

TEST(ScanRangeConversion, ClampsNegativeFieldOfViewToZero)
{
  auto scan = make_scan();
  scan.angle_min = 0.5F;
  scan.angle_max = -0.5F;
  scan.ranges = {1.0F};

  const auto range = robot_io_adapters::laser_scan_to_range(scan);

  EXPECT_FLOAT_EQ(range.field_of_view, 0.0F);
}
