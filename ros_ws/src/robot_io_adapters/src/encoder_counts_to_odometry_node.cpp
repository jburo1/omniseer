#include "robot_io_adapters/encoder_counts_to_odometry_node.hpp"

#include <cmath>
#include <optional>
#include <stdexcept>

namespace robot_io_adapters
{

EncoderCountsToOdometryNode::EncoderCountsToOdometryNode()
: rclcpp::Node("encoder_counts_to_odometry")
{
  _encoder_topic = this->declare_parameter<std::string>("encoder_topic", "/encoder_counts");
  _odom_topic = this->declare_parameter<std::string>("odom_topic",
      "/mecanum_drive_controller/odometry");
  _odom_frame_id = this->declare_parameter<std::string>("odom_frame_id", "odom");
  _base_frame_id = this->declare_parameter<std::string>("base_frame_id", "base_link");

  const double wheel_radius_m = this->declare_parameter<double>("wheel_radius_m", 0.0485);
  const double half_length_m = this->declare_parameter<double>("half_length_m", 0.09735);
  const double half_width_m = this->declare_parameter<double>("half_width_m", 0.10886705);
  const double encoder_ticks_per_rev = this->declare_parameter<double>("encoder_ticks_per_rev",
      44.0 * 131.0);

  _pose_covariance = diagonal_covariance(1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6);
  _twist_covariance = diagonal_covariance(0.05, 0.05, 1.0e6, 1.0e6, 1.0e6, 0.1);

  _integrator = std::make_unique<EncoderOdometryIntegrator>(
      wheel_radius_m, half_length_m, half_width_m, encoder_ticks_per_rev);

  _publisher = this->create_publisher<nav_msgs::msg::Odometry>(_odom_topic, 10);
  _subscription = this->create_subscription<omniseer_msgs::msg::WheelEncoderCounts>(
      _encoder_topic,
      10,
    [this](const omniseer_msgs::msg::WheelEncoderCounts & message) {encoder_callback(message);});

  RCLCPP_INFO(
      this->get_logger(), "EncoderCountsToOdometry: %s -> %s", _encoder_topic.c_str(),
      _odom_topic.c_str());
}

void EncoderCountsToOdometryNode::encoder_callback(
  const omniseer_msgs::msg::WheelEncoderCounts & message)
{
  builtin_interfaces::msg::Time header_stamp = message.stamp;
  double stamp_s = time_to_seconds(message.stamp);

  if (stamp_s <= 0.0) {
    const auto now = this->get_clock()->now();
    stamp_s = now.seconds();
    const auto now_ns = now.nanoseconds();
    header_stamp.sec = static_cast<int32_t>(now_ns / 1000000000LL);
    header_stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);
  }

  const WheelCountsSnapshot sample{
    stamp_s,
    message.front_left,
    message.front_right,
    message.rear_left,
    message.rear_right,
  };

  std::optional<EncoderOdometryEstimate> estimate;
  try {
    estimate = _integrator->update(sample);
  } catch (const std::invalid_argument & error) {
    RCLCPP_WARN(this->get_logger(), "Dropping encoder sample: %s", error.what());
    return;
  }

  if (!estimate.has_value()) {
    return;
  }

  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = header_stamp;
  odometry.header.frame_id = _odom_frame_id;
  odometry.child_frame_id = _base_frame_id;
  odometry.pose.pose.position.x = estimate->pose.x;
  odometry.pose.pose.position.y = estimate->pose.y;
  odometry.pose.pose.orientation = quaternion_from_yaw(estimate->pose.theta);
  odometry.pose.covariance = _pose_covariance;
  odometry.twist.twist.linear.x = estimate->twist.vx;
  odometry.twist.twist.linear.y = estimate->twist.vy;
  odometry.twist.twist.angular.z = estimate->twist.wz;
  odometry.twist.covariance = _twist_covariance;

  _publisher->publish(odometry);
}

double EncoderCountsToOdometryNode::time_to_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + (static_cast<double>(stamp.nanosec) * 1.0e-9);
}

geometry_msgs::msg::Quaternion EncoderCountsToOdometryNode::quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

std::array<double, 36> EncoderCountsToOdometryNode::diagonal_covariance(
  double x,
  double y,
  double z,
  double roll,
  double pitch,
  double yaw)
{
  std::array<double, 36> covariance{};
  covariance[0] = x;
  covariance[7] = y;
  covariance[14] = z;
  covariance[21] = roll;
  covariance[28] = pitch;
  covariance[35] = yaw;
  return covariance;
}

} // namespace robot_io_adapters
