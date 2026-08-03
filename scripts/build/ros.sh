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
  scripts/omni build ros [--with-vision|--without-vision] [--with-yolo] [colcon args...]

Builds the default ROS package set documented in CI. If RKNN/RGA development files
are available, the real-hardware omniseer_vision_bridge package is included.

Options:
  --with-vision     Force building the real-hardware omniseer_vision_bridge package.
                 Requires RKNN and RGA SDK/runtime development files.
  --without-vision  Build only the portable ROS package set.
  --with-yolo       Also build the optional Python yolo_ros sim provider packages.
EOF
}

vision_mode=auto
with_yolo=false
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
    --with-yolo)
      with_yolo=true
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
colcon_python="python3"
declare -a packages
declare -a ignored_packages
declare -a colcon_command
declare -a build_python_cmake_args
omni_read_lines_into_array packages omni_ros_core_packages
omni_read_lines_into_array ignored_packages omni_ros_core_ignore_packages

if [[ -n "${VIRTUAL_ENV:-}" && -x "${VIRTUAL_ENV}/bin/python" ]]; then
  colcon_python="${VIRTUAL_ENV}/bin/python"
elif [[ -x "/opt/venv/bin/python" ]]; then
  colcon_python="/opt/venv/bin/python"
fi

if "${colcon_python}" -m colcon --help >/dev/null 2>&1; then
  colcon_command=("${colcon_python}" -m colcon)
else
  colcon_command=(colcon)
fi

if [[ "${with_yolo}" == true ]]; then
  if ! "${colcon_python}" -c "import torch; import ultralytics" >/dev/null 2>&1; then
    omni_die \
      "YOLO sim support requires Python 'torch' and 'ultralytics' in ${colcon_python};" \
      "rebuild the NVIDIA devcontainer or install them into that environment"
  fi

  packages+=(yolo_ros yolo_bringup)
  build_python_cmake_args=(
    --cmake-args
    "-DPython3_EXECUTABLE=${colcon_python}"
    -DPython3_FIND_VIRTUALENV=ONLY
  )
  declare -a filtered_ignored_packages=()
  for ignored_package in "${ignored_packages[@]}"; do
    case "${ignored_package}" in
      yolo_ros|yolo_bringup)
        ;;
      *)
        filtered_ignored_packages+=("${ignored_package}")
        ;;
    esac
  done
  ignored_packages=("${filtered_ignored_packages[@]}")
fi

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
exec "${colcon_command[@]}" build --merge-install \
  --packages-ignore "${ignored_packages[@]}" \
  --packages-select "${packages[@]}" \
  "${build_python_cmake_args[@]}" \
  "${colcon_args[@]}"
