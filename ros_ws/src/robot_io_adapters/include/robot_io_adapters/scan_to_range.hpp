#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>

namespace robot_io_adapters
{

double select_best_range(const sensor_msgs::msg::LaserScan & scan);

sensor_msgs::msg::Range convert_scan_to_range(const sensor_msgs::msg::LaserScan & scan);

class ScanToRangeNode : public rclcpp::Node
{
public:
  ScanToRangeNode();

private:
  void scan_callback(const sensor_msgs::msg::LaserScan & scan);

  std::string _scan_topic{};
  std::string _range_topic{};
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr _publisher{};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _subscription{};
};

} // namespace robot_io_adapters
