#include "robot_io_adapters/scan_range_conversion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot_io_adapters
{

namespace
{

float nearest_valid_range(const sensor_msgs::msg::LaserScan & scan)
{
  float nearest = std::numeric_limits<float>::infinity();

  for (const float range : scan.ranges) {
    if (!std::isfinite(range)) {
      continue;
    }
    if (range < scan.range_min || range > scan.range_max) {
      continue;
    }
    nearest = std::min(nearest, range);
  }

  return nearest;
}

} // namespace

sensor_msgs::msg::Range laser_scan_to_range(const sensor_msgs::msg::LaserScan & scan)
{
  sensor_msgs::msg::Range range;
  range.header = scan.header;
  range.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  range.field_of_view = std::max(0.0F, scan.angle_max - scan.angle_min);
  range.min_range = scan.range_min;
  range.max_range = scan.range_max;
  range.range = nearest_valid_range(scan);
  return range;
}

} // namespace robot_io_adapters
