#!/usr/bin/env bash

omni_default_ros_setup() {
  printf '%s\n' "${OMNISEER_ROS_SETUP:-/opt/ros/${OMNISEER_ROS_DISTRO:-kilted}/setup.bash}"
}

omni_default_ws_setup() {
  if [[ -n "${OMNISEER_WS_SETUP:-}" ]]; then
    printf '%s\n' "${OMNISEER_WS_SETUP}"
    return
  fi

  if [[ -f "/ros_ws/install/setup.bash" ]]; then
    printf '%s\n' "/ros_ws/install/setup.bash"
    return
  fi

  if [[ -f "${OMNI_REPO_ROOT}/ros_ws/install/setup.bash" ]]; then
    printf '%s\n' "${OMNI_REPO_ROOT}/ros_ws/install/setup.bash"
    return
  fi

  printf '\n'
}

omni_default_uros_ws_setup() {
  if [[ -n "${OMNISEER_UROS_WS_SETUP:-}" ]]; then
    printf '%s\n' "${OMNISEER_UROS_WS_SETUP}"
    return
  fi

  if [[ -f "/uros_ws/install/setup.bash" ]]; then
    printf '%s\n' "/uros_ws/install/setup.bash"
    return
  fi

  if [[ -f "${OMNI_REPO_ROOT}/uros_ws/install/setup.bash" ]]; then
    printf '%s\n' "${OMNI_REPO_ROOT}/uros_ws/install/setup.bash"
    return
  fi

  printf '\n'
}

omni_source_file() {
  local setup_path="$1"
  if [[ ! -f "${setup_path}" ]]; then
    omni_die "required setup file not found at ${setup_path}"
  fi

  set +u
  # shellcheck disable=SC1090
  source "${setup_path}"
  set -u
}

omni_source_ros() {
  local ros_setup
  ros_setup="$(omni_default_ros_setup)"
  omni_source_file "${ros_setup}"
}

omni_source_workspace() {
  local ws_setup
  ws_setup="$(omni_default_ws_setup)"
  if [[ -z "${ws_setup}" ]]; then
    omni_die "workspace setup.bash not found; set OMNISEER_WS_SETUP or build ros_ws first"
  fi
  omni_source_file "${ws_setup}"
}

omni_source_ros_workspace() {
  omni_source_ros
  omni_source_workspace
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
}

omni_ros_ws_root() {
  if [[ -d "/ros_ws/src" ]]; then
    printf '%s\n' "/ros_ws"
    return
  fi

  printf '%s\n' "${OMNI_REPO_ROOT}/ros_ws"
}
