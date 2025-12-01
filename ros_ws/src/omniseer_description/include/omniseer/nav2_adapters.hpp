#pragma once

#include <chrono>
#include <cmath>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <optional>
#include <rclcpp/logger.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <tf2_ros/buffer.h>

#include "omniseer/frontier.hpp"
#include "omniseer/grid_io.hpp"

namespace omniseer
{
  inline double pathLength(const nav_msgs::msg::Path& path)
  {
    const auto& poses = path.poses;
    if (poses.size() < 2)
      return 0.0;

    double total = 0.0;
    auto   prev  = poses.front().pose.position;

    for (std::size_t i = 1; i < poses.size(); ++i)
    {
      const auto&  p    = poses[i].pose.position;
      const double step = std::hypot(p.x - prev.x, p.y - prev.y);
      if (std::isfinite(step))
        total += step;
      prev = p;
    }
    return total;
  }

  /**
   * \brief Convert a Nav2 costmap message into a GridU8.
   * \throws std::runtime_error if metadata dimensions do not match data size.
   */
  GridU8 costmapToGrid(const nav2_msgs::msg::Costmap& costmap);

  /**
   * \brief Lookup the robot pose in the specified frame.
   * \return Pose if lookup succeeded, std::nullopt otherwise (warnings logged).
   */
  std::optional<Pose2D> lookupRobotPose(tf2_ros::Buffer& buffer, const std::string& global_frame,
                                        const std::string&    robot_frame,
                                        const rclcpp::Logger& logger);

  /**
   * \brief Invoke ComputePathToPose to obtain the path length between poses.
   * \return Path length in meters or std::nullopt if planner is unavailable or fails.
   */
  std::optional<double> computePathLength(
      rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>& client, const Pose2D& start,
      const Pose2D& goal, const std::string& frame_id, std::chrono::milliseconds timeout,
      const rclcpp::Logger& logger);

} // namespace omniseer
