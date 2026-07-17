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
  scripts/omni build ros [colcon args...]

Builds the default portable ROS package set documented in CI.
EOF
}

if [[ "${1:-}" =~ ^(-h|--help|help)$ ]]; then
  usage
  exit 0
fi

omni_require_command colcon
omni_source_ros

ros_ws_root="$(omni_ros_ws_root)"
declare -a packages
declare -a ignored_packages
omni_read_lines_into_array packages omni_ros_core_packages
omni_read_lines_into_array ignored_packages omni_ros_core_ignore_packages

omni_info "Building ROS workspace packages: ${packages[*]}"
cd "${ros_ws_root}"
exec colcon build --merge-install \
  --packages-ignore "${ignored_packages[@]}" \
  --packages-select "${packages[@]}" \
  "$@"
