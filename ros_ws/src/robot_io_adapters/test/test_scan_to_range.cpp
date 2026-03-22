#include <gtest/gtest.h>

#include <limits>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>

#include "robot_io_adapters/scan_to_range.hpp"

namespace
{

sensor_msgs::msg::LaserScan make_scan()
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "sonar_link";
  scan.angle_min = -0.2F;
  scan.angle_max = 0.3F;
  scan.range_min = 0.15F;
  scan.range_max = 4.0F;
  return scan;
}

} // namespace

TEST(ScanToRange, SelectsClosestFiniteValidBeam)
{
  auto scan = make_scan();
  scan.ranges = {
    std::numeric_limits<float>::quiet_NaN(),
    3.5F,
    0.4F,
    0.25F,
    std::numeric_limits<float>::infinity(),
  };

  EXPECT_FLOAT_EQ(robot_io_adapters::select_best_range(scan), 0.25F);
}

TEST(ScanToRange, FallsBackToRangeMaxWhenNoValidBeamsExist)
{
  auto scan = make_scan();
  scan.ranges = {
    std::numeric_limits<float>::quiet_NaN(),
    0.01F,
    10.0F,
    std::numeric_limits<float>::infinity(),
  };

  EXPECT_FLOAT_EQ(robot_io_adapters::select_best_range(scan), scan.range_max);
}

TEST(ScanToRange, PreservesScanHeaderAndMetadata)
{
  auto scan = make_scan();
  scan.ranges = {0.6F, 0.4F, 0.8F};

  const auto range = robot_io_adapters::convert_scan_to_range(scan);

  EXPECT_EQ(range.header.frame_id, scan.header.frame_id);
  EXPECT_FLOAT_EQ(range.field_of_view, scan.angle_max - scan.angle_min);
  EXPECT_FLOAT_EQ(range.min_range, scan.range_min);
  EXPECT_FLOAT_EQ(range.max_range, scan.range_max);
  EXPECT_EQ(range.radiation_type, sensor_msgs::msg::Range::ULTRASOUND);
  EXPECT_FLOAT_EQ(range.range, 0.4F);
}
