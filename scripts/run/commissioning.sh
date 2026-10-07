#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source "${script_dir}/../lib/common.sh"
# shellcheck disable=SC1091
source "${script_dir}/../lib/ros.sh"

usage() {
  cat <<'EOF'
Usage:
  scripts/omni run commissioning [--dry-run]

Runs one non-repeating physical motion sequence through /cmd_vel_autonomy.
Start the recorded real bringup separately before executing it. --dry-run
prints the resolved sequence without initializing ROS or publishing commands.
EOF
}

if [[ "${1:-}" =~ ^(-h|--help|help)$ ]]; then
  usage
  exit 0
fi

omni_source_ros_workspace
exec ros2 run omniseer_experiments commissioning_motion "$@"
