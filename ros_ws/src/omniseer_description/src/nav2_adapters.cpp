#include "omniseer/nav2_adapters.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace omniseer
{

  GridU8 costmapToGrid(const nav2_msgs::msg::Costmap& costmap)
  {
    GridU8 grid;
    grid.width      = static_cast<int>(costmap.metadata.size_x);
    grid.height     = static_cast<int>(costmap.metadata.size_y);
    grid.resolution = costmap.metadata.resolution;
    grid.origin_x   = costmap.metadata.origin.position.x;
    grid.origin_y   = costmap.metadata.origin.position.y;

    const std::size_t expected =
        static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height);
    if (expected != costmap.data.size())
    {
      throw std::runtime_error("costmap data size mismatch");
    }

    grid.data.resize(expected);
    std::copy(costmap.data.begin(), costmap.data.end(), grid.data.begin());
    return grid;
  }

  std::optional<Pose2D> lookupRobotPose(tf2_ros::Buffer& buffer, const std::string& global_frame,
                                        const std::string&    robot_frame,
                                        const rclcpp::Logger& logger)
  {
    try
    {
      const auto tf = buffer.lookupTransform(global_frame, robot_frame, tf2::TimePointZero);
      Pose2D     pose;
      pose.x = tf.transform.translation.x;
      pose.y = tf.transform.translation.y;
      return pose;
    }
    catch (const tf2::TransformException& e)
    {
      RCLCPP_WARN(logger, "TF lookup %s -> %s failed: %s", global_frame.c_str(),
                  robot_frame.c_str(), e.what());
      return std::nullopt;
    }
  }

  std::optional<double> computePathLength(
      rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>& client, const Pose2D& start,
      const Pose2D& goal, const std::string& frame_id, std::chrono::milliseconds timeout,
      const rclcpp::Logger& logger)
  {
    if (!client.wait_for_action_server(timeout))
    {
      RCLCPP_WARN(logger, "Planner action not available");
      return std::nullopt;
    }

    nav2_msgs::action::ComputePathToPose::Goal goal_msg;
    goal_msg.start.header.frame_id = frame_id;
    goal_msg.goal.header.frame_id  = frame_id;

    goal_msg.start.pose.position.x    = start.x;
    goal_msg.start.pose.position.y    = start.y;
    goal_msg.start.pose.orientation.w = 1.0;

    goal_msg.goal.pose.position.x    = goal.x;
    goal_msg.goal.pose.position.y    = goal.y;
    goal_msg.goal.pose.orientation.w = 1.0;

    auto future_handle = client.async_send_goal(goal_msg);
    if (future_handle.wait_for(timeout) != std::future_status::ready)
    {
      RCLCPP_WARN(logger, "Planner goal send timeout");
      return std::nullopt;
    }
    auto handle = future_handle.get();
    if (!handle)
    {
      RCLCPP_WARN(logger, "Planner goal handle null");
      return std::nullopt;
    }

    auto result_future = client.async_get_result(handle);
    if (result_future.wait_for(timeout) != std::future_status::ready)
    {
      RCLCPP_WARN(logger, "Planner result wait timeout");
      return std::nullopt;
    }

    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_DEBUG(logger, "Planner result code %d", static_cast<int>(result.code));
      return std::nullopt;
    }

    const auto& poses = result.result->path.poses;
    if (poses.size() < 2)
    {
      return 0.0;
    }

    double length = 0.0;
    for (std::size_t i = 1; i < poses.size(); ++i)
    {
      const auto& a = poses[i - 1].pose.position;
      const auto& b = poses[i].pose.position;
      length += std::hypot(b.x - a.x, b.y - a.y);
    }

    return length;
  }

} // namespace omniseer
