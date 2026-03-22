#pragma once

#include <array>
#include <memory>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <omniseer_msgs/msg/wheel_encoder_counts.hpp>
#include <rclcpp/rclcpp.hpp>

#include "robot_io_adapters/odometry_math.hpp"

namespace robot_io_adapters
{

class EncoderCountsToOdometryNode : public rclcpp::Node
{
public:
  EncoderCountsToOdometryNode();

private:
  void encoder_callback(const omniseer_msgs::msg::WheelEncoderCounts & message);

  static double time_to_seconds(const builtin_interfaces::msg::Time & stamp);
  static geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw);
  static std::array<double, 36> diagonal_covariance(
    double x,
    double y,
    double z,
    double roll,
    double pitch,
    double yaw);

  std::string _encoder_topic{};
  std::string _odom_topic{};
  std::string _odom_frame_id{};
  std::string _base_frame_id{};
  std::array<double, 36> _pose_covariance{};
  std::array<double, 36> _twist_covariance{};

  std::unique_ptr<EncoderOdometryIntegrator> _integrator{};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _publisher{};
  rclcpp::Subscription<omniseer_msgs::msg::WheelEncoderCounts>::SharedPtr _subscription{};
};

} // namespace robot_io_adapters
