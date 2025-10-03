#!/bin/bash
echo 'ros2 launch bringup orchestrate_sim.launch.py ' "$@"

source ./install/setup.bash

PATTERN='(^|[^[:alnum:]_])(ros2|rviz2|gazebo(server|client)?|gz([[:space:]]+|-)?(sim|gui)|gz|ros_gz(_(sim|bridge))?|nav2|slam_toolbox|ekf_filter|gz_ros_control|scan_to_range|robot_state_publisher|controller_manager(/spawner)?|parameter_bridge|image_bridge|mecanum_drive_controller|joint_state_publisher|jsb|lifecycle_manager|lifecycle_manager_slam|rf2o|yolo(_ros|_bringup)?|yolo_node|yolov[0-9]+|ultralytics)([^[:alnum:]_]|$)'

ros2 daemon stop >/dev/null 2>&1 || true

echo "=== Initializing simulation ==="

echo "=== Lingering ROS/GZ processes ==="
pgrep -fa -l -f "$PATTERN" || echo "(none)"

kill_phase() {
  local sig="$1" delay="${2:-1}"
  pkill -e "$sig" -f "$PATTERN" 2>/dev/null || true
  sleep "$delay"
}

kill_phase -INT 0.5
kill_phase -TERM 0.5
kill_phase -KILL 0.5

echo "=== After cleanup ==="
pgrep -fa -l -f "$PATTERN" || echo "(clean)"

ros2 daemon start >/dev/null 2>&1 || true

echo "=== Current ROS graph (nodes/topics) ==="
ros2 node list || true
ros2 topic list || true

exec ros2 launch bringup orchestrate_sim.launch.py "$@"
