#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source "${script_dir}/../lib/log.sh"
# shellcheck disable=SC1091
source "${script_dir}/../lib/common.sh"

usage() {
  cat <<'EOF'
Usage:
  scripts/omni setup ros-deps [package_path ...]

Without explicit package paths, installs the default ROS dependency set used by
the portable local ROS workflow.
EOF
}

subcommand="${1:-help}"
if [[ $# -gt 0 ]]; then
  shift
fi

case "${subcommand}" in
  ros-deps)
    if [[ "${1:-}" =~ ^(-h|--help|help)$ ]]; then
      usage
      exit 0
    fi

    dependency_paths=()
    if [[ $# -eq 0 ]]; then
      omni_read_lines_into_array dependency_paths omni_ros_dep_paths_core
    else
      dependency_paths=("$@")
    fi

    omni_info "Installing ROS workspace dependencies"
    exec bash "${script_dir}/../ci/install_ros_workspace_deps.sh" "${dependency_paths[@]}"
    ;;
  help|-h|--help)
    usage
    ;;
  *)
    omni_die "unknown setup command: ${subcommand}"
    ;;
esac
