#!/usr/bin/env bash
set -euo pipefail

ros_distro="${ROS_DISTRO:-kilted}"
export ROS_DISTRO="${ros_distro}"

if [[ -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ros_distro}/setup.bash"
  set -u
fi

if [[ -f "${OMNISEER_WS_SETUP:-/ros_ws/install/setup.bash}" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "${OMNISEER_WS_SETUP:-/ros_ws/install/setup.bash}"
  set -u
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

if [[ "${1:-}" == "bash" || "${1:-}" == "sh" || "${1:-}" == /* ]]; then
  exec "$@"
fi

exec /opt/omniseer/scripts/omni "$@"
