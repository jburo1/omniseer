#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source "${script_dir}/../lib/log.sh"
# shellcheck disable=SC1091
source "${script_dir}/../lib/common.sh"
# shellcheck disable=SC1091
source "${script_dir}/../lib/ros.sh"

usage() {
  cat <<'EOF'
Usage:
  scripts/omni run monitor --host <robot-ip> [monitor args...]

Launches the laptop Tk monitor. The preview host defaults to the gateway host.
EOF
}

if [[ "${1:-}" =~ ^(-h|--help|help)$ ]]; then
  usage
  exit 0
fi

omni_source_ros_workspace
omni_info "Starting laptop operator monitor"
exec ros2 run robot_diag_control robot_monitor_gui --refresh-on-start "$@"
