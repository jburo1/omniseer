#include "robot_io_adapters/scan_to_range_node.hpp"

#include "robot_io_adapters/scan_range_conversion.hpp"

namespace robot_io_adapters
{

ScanToRangeNode::ScanToRangeNode()
: rclcpp::Node("scan_to_range")
{
  _scan_topic = this->declare_parameter<std::string>("scan_topic", "/sonar");
  _range_topic = this->declare_parameter<std::string>("range_topic", "/range");

  const auto qos = rclcpp::SensorDataQoS();
  _publisher = this->create_publisher<sensor_msgs::msg::Range>(_range_topic, qos);
  _subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
      _scan_topic, qos, [this](const sensor_msgs::msg::LaserScan & scan) {scan_callback(scan);});

  RCLCPP_INFO(
      this->get_logger(), "ScanToRange: %s -> %s", _scan_topic.c_str(), _range_topic.c_str());
}

void ScanToRangeNode::scan_callback(const sensor_msgs::msg::LaserScan & scan)
{
  _publisher->publish(laser_scan_to_range(scan));
}

} // namespace robot_io_adapters
