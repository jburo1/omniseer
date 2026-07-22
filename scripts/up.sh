#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source "${script_dir}/lib/log.sh"
# shellcheck disable=SC1091
source "${script_dir}/lib/common.sh"

usage() {
  cat <<'EOF'
Usage:
  scripts/omni up sim [launch args...]
  scripts/omni up real [--profile <name>] [mode] [launch args...]

Builds the needed ROS workspace, then runs the selected profile.

Examples:
  scripts/omni up sim headless:=true
  scripts/omni up real
  scripts/omni up real smoke
EOF
}

profile="${1:-help}"
if [[ $# -gt 0 ]]; then
  shift
fi

case "${profile}" in
  sim)
    omni_warn "scripts/up.sh is deprecated; run 'scripts/omni build ros' then 'scripts/omni run sim'"
    omni_info "Building ROS workspace for sim profile"
    "${script_dir}/build/ros.sh"
    exec "${script_dir}/run/sim.sh" "$@"
    ;;
  real)
    omni_warn "scripts/up.sh is deprecated; run 'scripts/omni build ros' then 'scripts/omni run real'"
    omni_info "Building ROS workspace for real profile"
    "${script_dir}/build/ros.sh"
    exec "${script_dir}/run/real.sh" "$@"
    ;;
  help|-h|--help)
    usage
    ;;
  *)
    omni_die "unknown up profile: ${profile}"
    ;;
esac
