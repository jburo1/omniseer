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
  scripts/omni run monitor [monitor args...]

Launches the laptop Tk monitor. The default robot target is radxa@192.168.1.178.
The preview host defaults to the gateway host. Override with --host and --ssh-user.
EOF
}

if [[ "${1:-}" =~ ^(-h|--help|help)$ ]]; then
  usage
  exit 0
fi

omni_source_ros_workspace
omni_info "Starting laptop operator monitor"
exec ros2 run robot_diag_control robot_monitor_gui --refresh-on-start "$@"
