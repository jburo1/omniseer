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
  scripts/omni run sim [launch args...]

Launches the existing simulation bringup helper path.
EOF
}

if [[ "${1:-}" =~ ^(-h|--help|help)$ ]]; then
  usage
  exit 0
fi

omni_source_ros_workspace

cleanup_script="$(ros2 pkg prefix bringup)/share/bringup/scripts/pre_launch_cleanup.sh"
if [[ -f "${cleanup_script}" ]]; then
  omni_info "Running simulation pre-launch cleanup"
  bash "${cleanup_script}" sim
fi

omni_info "Launching simulation bringup"
exec ros2 launch bringup orchestrate_sim.launch.py "$@"
