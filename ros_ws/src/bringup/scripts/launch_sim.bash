#!/bin/bash

source ./install/setup.bash

PATTERN='(^|[[:space:]/])(ros2|rviz2|gazebo(server|client)?|gz([[:space:]]+|-)?(sim|gui)|gz|ros_gz(_(sim|bridge))?|nav2|slam_toolbox|ekf_filter|gz_ros_control|scan_to_range|robot_state_publisher|controller_manager(/spawner)?|parameter_bridge|image_bridge|mecanum_drive_controller|joint_state_publisher|jsb|lifecycle_manager)\b'

ros2 daemon stop >/dev/null 2>&1 || true

echo "=== ROS/GZ processes that may be lingering ==="
pgrep -fa -l -f "$PATTERN" || true

kill_phase() {
  local sig="$1" delay="${2:-1}"
  pkill "$sig" -f "$PATTERN" 2>/dev/null || true
  sleep "$delay"
}

kill_phase -INT 1
kill_phase -TERM 1
kill_phase -KILL 1

echo "=== After cleanup (should be empty) ==="
pgrep -fa -l -f "$PATTERN" || echo "(none)"

ros2 daemon start >/dev/null 2>&1 || true

echo "=== ROS graph (nodes/topics) ==="
ros2 node list || true
ros2 topic list || true

exec ros2 launch bringup orchestrate_sim.launch.py "$@"