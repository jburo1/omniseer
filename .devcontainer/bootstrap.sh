#!/usr/bin/env bash
WS="${REMOTE_CONTAINERS_WORKSPACE_FOLDER:-$PWD}"
cd "$WS/ros_ws"

source /opt/ros/${ROS_DISTRO}/setup.bash
source /opt/venv/bin/activate

sudo apt-get update

rosdep update
rosdep install --from-paths src --rosdistro "${ROS_DISTRO}" --ignore-src -r -y --as-root apt:true --as-root pip:false

python -m colcon build --symlink-install --merge-install \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DPython3_EXECUTABLE=/opt/venv/bin/python \
  -DPython3_ROOT_DIR=/opt/venv
