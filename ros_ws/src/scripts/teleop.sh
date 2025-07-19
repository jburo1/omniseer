#!/usr/bin/env bash
source /opt/ros/kilted/setup.bash
source /home/ws/ros_ws/install/setup.bash
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
     --ros-args -r cmd_vel:=/cmd_vel_keyboard
