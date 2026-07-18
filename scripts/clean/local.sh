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
  scripts/omni clean [ros]
  scripts/omni clean vision
  scripts/omni clean docs
  scripts/omni clean all

Targets:
  ros     Remove ROS workspace build/install/log trees.
  vision  Remove vision build directories.
  docs    Remove generated site and Doxygen outputs.
  all     Remove all of the above.
EOF
}

clean_ros() {
  local repo_root="$1"

  omni_info "Removing ROS workspace build artifacts"
  rm -rf "${repo_root}/ros_ws/build" "${repo_root}/ros_ws/install" "${repo_root}/ros_ws/log"
}

clean_vision() {
  local repo_root="$1"

  omni_info "Removing vision build directories"
  rm -rf "${repo_root}"/vision/build*
}

clean_docs() {
  local repo_root="$1"

  omni_info "Removing generated docs outputs"
  rm -rf "${repo_root}/site" "${repo_root}/build/docs"
}

target="${1:-ros}"
if [[ $# -gt 0 ]]; then
  shift
fi

if [[ "${target}" =~ ^(-h|--help|help)$ ]]; then
  usage
  exit 0
fi

if [[ $# -gt 0 ]]; then
  omni_die "clean ${target} does not accept extra arguments: $*"
fi

repo_root="$(omni_repo_root)"

case "${target}" in
  ros)
    clean_ros "${repo_root}"
    ;;
  vision)
    clean_vision "${repo_root}"
    ;;
  docs)
    clean_docs "${repo_root}"
    ;;
  all)
    clean_ros "${repo_root}"
    clean_vision "${repo_root}"
    clean_docs "${repo_root}"
    ;;
  *)
    omni_die "unknown clean target: ${target}"
    ;;
esac
