#!/bin/bash
echo 'ros2 launch bringup orchestrate_sim.launch.py ' "$@"

source ./install/setup.bash

echo "=== Initializing simulation ==="

cleanup_script="$(ros2 pkg prefix bringup)/share/bringup/scripts/pre_launch_cleanup.sh"
bash "${cleanup_script}" sim

echo "=== Current ROS graph (nodes/topics) ==="
ros2 node list || true
ros2 topic list || true

exec ros2 launch bringup orchestrate_sim.launch.py "$@"
