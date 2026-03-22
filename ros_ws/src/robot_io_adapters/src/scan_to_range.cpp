#include "robot_io_adapters/scan_to_range.hpp"

#include <cmath>

namespace robot_io_adapters
{

double select_best_range(const sensor_msgs::msg::LaserScan & scan)
{
  double best = scan.range_max;

  for (const float value : scan.ranges) {
    if (!std::isfinite(value)) {
      continue;
    }
    if (value < scan.range_min || value > scan.range_max) {
      continue;
    }
    if (value < best) {
      best = value;
    }
  }

  if (best < scan.range_min) {
    return scan.range_min;
  }
  if (best > scan.range_max) {
    return scan.range_max;
  }
  return best;
}

sensor_msgs::msg::Range convert_scan_to_range(const sensor_msgs::msg::LaserScan & scan)
{
  sensor_msgs::msg::Range message;
  message.header = scan.header;
  message.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  message.field_of_view = scan.angle_max - scan.angle_min;
  message.min_range = scan.range_min;
  message.max_range = scan.range_max;
  message.range = select_best_range(scan);
  return message;
}

ScanToRangeNode::ScanToRangeNode()
: rclcpp::Node("scan_to_range")
{
  _scan_topic = this->declare_parameter<std::string>("scan_topic", "/sonar");
  _range_topic = this->declare_parameter<std::string>("range_topic", "/range");

  const auto qos = rclcpp::SensorDataQoS();
  _publisher = this->create_publisher<sensor_msgs::msg::Range>(_range_topic, qos);
  _subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
    _scan_topic,
    qos,
    [this](const sensor_msgs::msg::LaserScan & scan) {scan_callback(scan);});

  RCLCPP_INFO(this->get_logger(), "ScanToRange: %s -> %s", _scan_topic.c_str(),
    _range_topic.c_str());
}

void ScanToRangeNode::scan_callback(const sensor_msgs::msg::LaserScan & scan)
{
  _publisher->publish(convert_scan_to_range(scan));
}

} // namespace robot_io_adapters
