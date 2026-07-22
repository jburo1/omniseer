#!/usr/bin/env bash

_omni_common_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_omni_scripts_dir="$(cd "${_omni_common_dir}/.." && pwd)"
OMNI_REPO_ROOT="$(cd "${_omni_scripts_dir}/.." && pwd)"
readonly OMNI_REPO_ROOT

omni_repo_root() {
  printf '%s\n' "${OMNI_REPO_ROOT}"
}

omni_scripts_root() {
  printf '%s\n' "${OMNI_REPO_ROOT}/scripts"
}

omni_require_command() {
  local command_name="$1"
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    omni_die "${command_name} not found in PATH"
  fi
}

omni_command_available() {
  command -v "$1" >/dev/null 2>&1
}

omni_platformio_bin() {
  if [[ -n "${PLATFORMIO_BIN:-}" ]]; then
    [[ -x "${PLATFORMIO_BIN}" ]] || return 1
    printf '%s\n' "${PLATFORMIO_BIN}"
    return 0
  fi

  if [[ -x "${HOME}/.platformio/penv/bin/platformio" ]]; then
    printf '%s\n' "${HOME}/.platformio/penv/bin/platformio"
    return 0
  fi

  if omni_command_available platformio; then
    command -v platformio
    return 0
  fi

  if omni_command_available pio; then
    command -v pio
    return 0
  fi

  return 1
}

omni_vision_bridge_deps_available() {
  omni_command_available pkg-config || return 1
  pkg-config --exists librga || return 1

  local cxx="${CXX:-c++}"
  omni_command_available "${cxx}" || return 1
  printf '#include <rknn_api.h>\n' | "${cxx}" -E -x c++ - >/dev/null 2>&1 || return 1

  if omni_command_available ldconfig && ldconfig -p | grep -q 'librknnrt\.so'; then
    return 0
  fi

  [[ -e /usr/lib/librknnrt.so ]] && return 0
  [[ -e /usr/lib/aarch64-linux-gnu/librknnrt.so ]] && return 0
  [[ -e /usr/local/lib/librknnrt.so ]] && return 0
  [[ -e /lib/librknnrt.so ]] && return 0
  find /usr /opt -name 'librknnrt.so*' -print -quit 2>/dev/null | grep -q .
}

omni_default_real_profile() {
  printf '%s\n' "${OMNISEER_DEFAULT_REAL_PROFILE:-current}"
}

omni_current_real_profile() {
  printf '%s\n' "${OMNISEER_CURRENT_REAL_PROFILE:-operator}"
}

omni_supported_real_profiles() {
  printf '%s\n' "current" "operator" "perception" "legacy-teleop"
}

omni_ros_dep_paths_core() {
  cat <<'EOF'
ros_ws/src/omniseer_gz_assets
ros_ws/src/omniseer_msgs
ros_ws/src/yolo_ros/yolo_msgs
ros_ws/src/omniseer_description
ros_ws/src/analysis
ros_ws/src/omniseer_experiments
ros_ws/src/bringup
ros_ws/src/robot_io_adapters
ros_ws/src/robot_diag_control
ros_ws/src/robot_diag_control_cpp
EOF
}

omni_ros_dep_paths_smoke() {
  cat <<'EOF'
ros_ws/src/omniseer_gz_assets
ros_ws/src/omniseer_msgs
ros_ws/src/omniseer_description
ros_ws/src/bringup
ros_ws/src/robot_io_adapters
EOF
}

omni_ros_core_packages() {
  cat <<'EOF'
omniseer_gz_assets
omniseer_msgs
yolo_msgs
omniseer_description
analysis
omniseer_experiments
bringup
robot_io_adapters
robot_diag_control
robot_diag_control_cpp
EOF
}

omni_ros_core_ignore_packages() {
  cat <<'EOF'
rf2o_laser_odometry
yolo_bringup
yolo_ros
EOF
}

omni_ros_test_packages() {
  cat <<'EOF'
omniseer_description
analysis
omniseer_experiments
bringup
robot_io_adapters
robot_diag_control
robot_diag_control_cpp
EOF
}

omni_ros_smoke_packages() {
  cat <<'EOF'
omniseer_gz_assets
omniseer_msgs
omniseer_description
bringup
robot_io_adapters
EOF
}

omni_ros_smoke_ignore_packages() {
  cat <<'EOF'
analysis
rf2o_laser_odometry
robot_diag_control_cpp
yolo_bringup
yolo_ros
EOF
}

omni_read_lines_into_array() {
  local destination_name="$1"
  local producer="$2"
  local -n destination_ref="${destination_name}"
  mapfile -t destination_ref < <("${producer}")
}
