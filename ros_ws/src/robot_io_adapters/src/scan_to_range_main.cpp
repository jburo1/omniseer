#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robot_io_adapters/scan_to_range.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_io_adapters::ScanToRangeNode>());
  rclcpp::shutdown();
  return 0;
}
