#!/usr/bin/env bash

WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." >/dev/null 2>&1 && pwd)"

# Common environment
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "/opt/venv/bin/activate"

sudo apt-get update
rosdep update

build_ws() {
  local ws_path="$1"

  echo "=== Building workspace: ${ws_path} ==="
  cd "${ws_path}"

  rosdep install --from-paths src \
    --rosdistro "${ROS_DISTRO}" \
    --ignore-src -r -y \
    --as-root apt:true --as-root pip:false

  python -m colcon build --symlink-install --merge-install \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DPython3_EXECUTABLE=/opt/venv/bin/python \
      -DPython3_ROOT_DIR=/opt/venv \
      -DPython3_FIND_VIRTUALENV=ONLY \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

  source "install/setup.bash"
}

# ROS2 application workspace
build_ws "${WS}/ros_ws"

# micro-ROS setup workspace (provides micro_ros_setup)
build_ws "${WS}/uros_ws"

grep -q "Omniseer ROS overlays" ~/.bashrc || cat >> ~/.bashrc <<'EOF'

# Omniseer ROS overlays
source /opt/ros/${ROS_DISTRO}/setup.bash
source /omniseer/uros_ws/install/setup.bash
source /omniseer/ros_ws/install/setup.bash
EOF
