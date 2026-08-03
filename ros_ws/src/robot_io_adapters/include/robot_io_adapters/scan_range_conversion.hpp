#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>

namespace robot_io_adapters
{

[[nodiscard]] sensor_msgs::msg::Range laser_scan_to_range(
  const sensor_msgs::msg::LaserScan & scan);

} // namespace robot_io_adapters
