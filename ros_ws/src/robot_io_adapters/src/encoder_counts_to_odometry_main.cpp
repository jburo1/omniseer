#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robot_io_adapters/encoder_counts_to_odometry_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_io_adapters::EncoderCountsToOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
