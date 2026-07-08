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
  scripts/omni test smoke-sim

Builds the CI-safe simulation package subset and runs the headless Gazebo smoke
test.
EOF
}

if [[ "${1:-}" =~ ^(-h|--help|help)$ ]]; then
  usage
  exit 0
fi

omni_require_command colcon
omni_require_command python3
omni_source_ros

ros_ws_root="$(omni_ros_ws_root)"
declare -a packages
declare -a ignored_packages
omni_read_lines_into_array packages omni_ros_smoke_packages
omni_read_lines_into_array ignored_packages omni_ros_smoke_ignore_packages

omni_info "Building simulation smoke-test package subset"
cd "${ros_ws_root}"
colcon build --merge-install \
  --packages-ignore "${ignored_packages[@]}" \
  --packages-select "${packages[@]}"

omni_source_workspace
omni_info "Running headless simulation smoke test"
OMNISEER_RUN_SIM_SMOKE=1 exec python3 -m pytest -vv -s src/bringup/test/test_sim_launch_smoke.py "$@"
