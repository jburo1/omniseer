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
  python3-ruff \
  python3-tk

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init
fi

rosdep update
rosdep install --from-paths "$@" --ignore-src -y --rosdistro "${ROS_DISTRO}"
