#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source "${script_dir}/lib/log.sh"
# shellcheck disable=SC1091
source "${script_dir}/lib/common.sh"
# shellcheck disable=SC1091
source "${script_dir}/lib/ros.sh"

usage() {
  cat <<'EOF'
Usage:
  scripts/omni doctor

Reports local Omniseer build, runtime, and robot dependency state.
EOF
}

status_line() {
  local label="$1"
  local state="$2"
  local detail="${3:-}"
  if [[ -n "${detail}" ]]; then
    printf '%-34s %s  %s\n' "${label}" "${state}" "${detail}"
  else
    printf '%-34s %s\n' "${label}" "${state}"
  fi
}

file_status() {
  local label="$1"
  local path="$2"
  if [[ -e "${path}" ]]; then
    status_line "${label}" "ok" "${path}"
  else
    status_line "${label}" "missing" "${path}"
  fi
}

command_status() {
  local label="$1"
  local command_name="$2"
  if omni_command_available "${command_name}"; then
    status_line "${label}" "ok" "$(command -v "${command_name}")"
  else
    status_line "${label}" "missing" "${command_name}"
  fi
}

ros_package_status() {
  local package_name="$1"
  if omni_command_available ros2 && ros2 pkg prefix "${package_name}" >/dev/null 2>&1; then
    status_line "ROS package ${package_name}" "ok" "$(ros2 pkg prefix "${package_name}")"
  else
    status_line "ROS package ${package_name}" "missing"
  fi
}

vision_config_paths() {
  local config_path="$1"
  awk -F': ' '
    /^[[:space:]]*(models\.detector_model_path|models\.clip_model_path|models\.clip_vocab_path|classes\.path):/ {
      gsub(/^[[:space:]]+/, "", $2)
      gsub(/["'\'']/, "", $2)
      print $1 "|" $2
    }
  ' "${config_path}"
}

if [[ "${1:-}" =~ ^(-h|--help|help)$ ]]; then
  usage
  exit 0
fi

if [[ $# -gt 0 ]]; then
  omni_die "doctor does not accept arguments: $*"
fi

repo_root="$(omni_repo_root)"
ros_setup="$(omni_default_ros_setup)"
ws_setup="$(omni_default_ws_setup)"
uros_ws_setup="$(omni_default_uros_ws_setup)"
vision_paths_config="${repo_root}/ros_ws/src/bringup/config/vision_bridge.real.paths.yaml"

printf 'Omniseer doctor\n'
printf '%s\n' '==============='
status_line "repo root" "ok" "${repo_root}"
file_status "ROS setup" "${ros_setup}"
if [[ -n "${ws_setup}" ]]; then
  file_status "ROS workspace setup" "${ws_setup}"
else
  status_line "ROS workspace setup" "missing" "build ros_ws first"
fi
if [[ -n "${uros_ws_setup}" ]]; then
  file_status "micro-ROS workspace setup" "${uros_ws_setup}"
else
  status_line "micro-ROS workspace setup" "missing"
fi

printf '\nTools\n'
printf '%s\n' '-----'
command_status "colcon" "colcon"
command_status "cmake" "cmake"
if platformio_bin="$(omni_platformio_bin)"; then
  status_line "PlatformIO" "ok" "${platformio_bin}"
else
  status_line "PlatformIO" "missing" "firmware build will be skipped by build all"
fi

printf '\nHardware SDKs\n'
printf '%s\n' '-------------'
if omni_command_available pkg-config && pkg-config --exists librga; then
  status_line "RGA pkg-config" "ok" "$(pkg-config --modversion librga 2>/dev/null || true)"
else
  status_line "RGA pkg-config" "missing" "librga"
fi
if printf '#include <rknn_api.h>\n' | "${CXX:-c++}" -E -x c++ - >/dev/null 2>&1; then
  status_line "RKNN header" "ok" "rknn_api.h"
else
  status_line "RKNN header" "missing" "rknn_api.h"
fi
if omni_vision_bridge_deps_available; then
  status_line "vision bridge build deps" "ok"
else
  status_line "vision bridge build deps" "missing" "omniseer_vision_bridge will be skipped unless forced"
fi

printf '\nROS Packages\n'
printf '%s\n' '------------'
if [[ -f "${ros_setup}" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "${ros_setup}"
  if [[ -n "${ws_setup}" && -f "${ws_setup}" ]]; then
    # shellcheck disable=SC1090
    source "${ws_setup}"
  fi
  set -u
fi
ros_package_status bringup
ros_package_status omniseer_msgs
ros_package_status yolo_msgs
ros_package_status omniseer_vision_bridge

printf '\nCamera Devices\n'
printf '%s\n' '--------------'
shopt -s nullglob
camera_devices=(/dev/video*)
if [[ ${#camera_devices[@]} -eq 0 ]]; then
  status_line "/dev/video*" "missing"
else
  for camera_device in "${camera_devices[@]}"; do
    status_line "camera device" "ok" "${camera_device}"
  done
fi
shopt -u nullglob

printf '\nVision Assets\n'
printf '%s\n' '-------------'
file_status "vision paths config" "${vision_paths_config}"
if [[ -f "${vision_paths_config}" ]]; then
  while IFS='|' read -r key path; do
    [[ -n "${path}" ]] || continue
    file_status "${key}" "${path}"
  done < <(vision_config_paths "${vision_paths_config}")
fi
