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
  scripts/omni build ros [--with-vision|--without-vision] [colcon args...]

Builds the default ROS package set documented in CI. If RKNN/RGA development files
are available, the real-hardware omniseer_vision_bridge package is included.

Options:
  --with-vision     Force building the real-hardware omniseer_vision_bridge package.
                 Requires RKNN and RGA SDK/runtime development files.
  --without-vision  Build only the portable ROS package set.
EOF
}

vision_mode=auto
declare -a colcon_args
while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-vision)
      vision_mode=force
      shift
      ;;
    --without-vision)
      vision_mode=skip
      shift
      ;;
    help|-h|--help)
      usage
      exit 0
      ;;
    *)
      colcon_args+=("$1")
      shift
      ;;
  esac
done

omni_require_command colcon
omni_source_ros

ros_ws_root="$(omni_ros_ws_root)"
declare -a packages
declare -a ignored_packages
omni_read_lines_into_array packages omni_ros_core_packages
omni_read_lines_into_array ignored_packages omni_ros_core_ignore_packages

case "${vision_mode}" in
  force)
    packages+=(omniseer_vision_bridge)
    ;;
  auto)
    if omni_vision_bridge_deps_available; then
      omni_info "Detected RKNN/RGA development files; including omniseer_vision_bridge"
      packages+=(omniseer_vision_bridge)
    else
      omni_info "RKNN/RGA development files not detected; skipping omniseer_vision_bridge"
    fi
    ;;
  skip)
    ;;
esac

omni_info "Building ROS workspace packages: ${packages[*]}"
cd "${ros_ws_root}"
exec colcon build --merge-install \
  --packages-ignore "${ignored_packages[@]}" \
  --packages-select "${packages[@]}" \
  "${colcon_args[@]}"
