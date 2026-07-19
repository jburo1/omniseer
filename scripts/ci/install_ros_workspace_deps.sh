#!/usr/bin/env bash

set -euo pipefail

if [ "$#" -eq 0 ]; then
  echo "usage: $0 <package_path> [<package_path> ...]" >&2
  exit 64
fi

: "${ROS_DISTRO:=kilted}"
export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y --no-install-recommends \
  git \
  libgrpc++-dev \
  libprotobuf-dev \
  protobuf-compiler-grpc \
  python3-colcon-common-extensions \
  python3-grpcio \
  python3-protobuf \
  python3-pytest \
  python3-rosdep \
  python3-tk

# ROS setup scripts may inspect optional environment variables while nounset is
# enabled. Keep strict mode for this script, but do not impose it on vendor code.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init
fi

rosdep update
# These runtime packages are provided by repository overlays excluded from the
# portable CI package set.
rosdep install --from-paths "$@" --ignore-src -y --rosdistro "${ROS_DISTRO}" \
  --skip-keys "analysis micro_ros_agent rf2o_laser_odometry robot_diag_control_cpp yolo_bringup"
