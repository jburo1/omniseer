#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "omniseer_autonomy/in_place_scan_controller.hpp"
#include "rclcpp/rclcpp.hpp"

namespace omniseer_autonomy
{
namespace
{
using namespace std::chrono_literals;

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & orientation)
{
  const auto siny_cosp =
    2.0 * ((orientation.w * orientation.z) + (orientation.x * orientation.y));
  const auto cosy_cosp =
    1.0 - (2.0 * ((orientation.y * orientation.y) + (orientation.z * orientation.z)));
  return std::atan2(siny_cosp, cosy_cosp);
}
}   // namespace

class InPlaceScanNode : public rclcpp::Node
{
public:
  explicit InPlaceScanNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("in_place_scan_node", options), _started_at(std::chrono::steady_clock::now()),
    _controller(make_config())
  {
    const auto odometry_topic = declare_parameter<std::string>("odometry_topic",
        "/odometry/filtered");
    const auto command_topic = declare_parameter<std::string>("command_topic", "/cmd_vel_autonomy");
    _frame_id = declare_parameter<std::string>("cmd_frame_id", "base_link");
    _odometry_timeout_sec = declare_parameter<double>("odometry_timeout_sec", 5.0);
    _odometry_stale_sec = declare_parameter<double>("odometry_stale_sec", 0.5);
    if (!std::isfinite(_odometry_timeout_sec) || _odometry_timeout_sec <= 0.0 ||
      !std::isfinite(_odometry_stale_sec) || _odometry_stale_sec <= 0.0)
    {
      throw std::invalid_argument("odometry timeouts must be finite and positive");
    }

    _publisher = create_publisher<geometry_msgs::msg::TwistStamped>(command_topic, 10);
    _odometry_subscription = create_subscription<nav_msgs::msg::Odometry>(
          odometry_topic, 10, [this](const nav_msgs::msg::Odometry & msg) {on_odometry(msg);});
    _timer = create_wall_timer(50ms, [this]() {on_timer();});

    RCLCPP_INFO(get_logger(), "perception scan started; yaw_rate_rad_s=%.3f revolutions=%.3f",
                  _yaw_rate_rad_s, _revolutions);
  }

  ~InPlaceScanNode() override
  {
    publish_command(0.0);
  }

  bool succeeded() const noexcept
  {
    return _succeeded;
  }

private:
  InPlaceScanConfig make_config()
  {
    InPlaceScanConfig config{};
    config.yaw_rate_rad_s = declare_parameter<double>("yaw_rate_rad_s", 0.20);
    config.revolutions = declare_parameter<double>("revolutions", 1.0);
    _yaw_rate_rad_s = config.yaw_rate_rad_s;
    _revolutions = config.revolutions;
    return config;
  }

  void on_odometry(const nav_msgs::msg::Odometry & msg)
  {
    if (_terminal) {
      return;
    }
    const auto heading_rad = yaw_from_quaternion(msg.pose.pose.orientation);
    if (!std::isfinite(heading_rad)) {
      finish(false, "invalid_odometry_yaw");
      return;
    }

    _last_odometry_at = std::chrono::steady_clock::now();
    const auto output = _controller.update_heading(heading_rad);
    publish_command(output.angular_z_rad_s);
    if (_controller.complete()) {
      finish(true, "complete");
    }
  }

  void on_timer()
  {
    if (_terminal) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (!_last_odometry_at.has_value()) {
      if (std::chrono::duration<double>(now - _started_at).count() > _odometry_timeout_sec) {
        finish(false, "odometry_not_received");
      }
      return;
    }
    if (std::chrono::duration<double>(now - *_last_odometry_at).count() > _odometry_stale_sec) {
      finish(false, "stale_odometry");
      return;
    }
    publish_command(_controller.command().angular_z_rad_s);
  }

  void publish_command(double angular_z_rad_s)
  {
    geometry_msgs::msg::TwistStamped msg{};
    msg.header.stamp = now();
    msg.header.frame_id = _frame_id;
    msg.twist.linear.x = 0.0;
    msg.twist.angular.z = angular_z_rad_s;
    _publisher->publish(msg);
  }

  void finish(bool success, const std::string & reason)
  {
    if (_terminal) {
      return;
    }
    _terminal = true;
    _succeeded = success;
    publish_command(0.0);
    _timer->cancel();
    if (success) {
      RCLCPP_INFO(get_logger(), "perception scan complete; yaw_travel_rad=%.3f",
                    _controller.yaw_travel_rad());
    } else {
      RCLCPP_ERROR(get_logger(), "perception scan failed; reason=%s", reason.c_str());
    }
    rclcpp::shutdown();
  }

  std::chrono::steady_clock::time_point _started_at{};
  InPlaceScanController _controller;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _publisher{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_subscription{};
  rclcpp::TimerBase::SharedPtr _timer{};
  std::optional<std::chrono::steady_clock::time_point> _last_odometry_at{};
  std::string _frame_id{"base_link"};
  double _yaw_rate_rad_s{0.20};
  double _revolutions{1.0};
  double _odometry_timeout_sec{5.0};
  double _odometry_stale_sec{0.5};
  bool _terminal{false};
  bool _succeeded{false};
};
} // namespace omniseer_autonomy

#ifndef OMNISEER_AUTONOMY_TESTING
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<omniseer_autonomy::InPlaceScanNode>();
  rclcpp::spin(node);
  const auto exit_code = node->succeeded() ? 0 : 1;
  rclcpp::shutdown();
  return exit_code;
}
#endif
