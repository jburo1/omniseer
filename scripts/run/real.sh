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
  scripts/omni run real [--phase <number>] [--mode <mode>] [mode] [launch args...]

Modes:
  operator  Phase 2 only: start bringup, then open keyboard teleop.
  smoke    Start bringup, run the phase verifier, then stop.
  bringup  Start only the selected real bringup in the foreground.
  teleop   Start only the stamped keyboard teleop publisher.
  verify   Run the phase verifier against an existing ROS graph.

Examples:
  scripts/omni run real --phase 2
  scripts/omni run real --phase 2 smoke
  scripts/omni run real --phase 2 bringup camera_device:=/dev/video11
  scripts/omni run real --phase 2 operator micro_ros_serial_device:=/dev/serial/by-id/usb-Teensyduino_USB_Serial_16634450-if00
  scripts/omni run real --phase 3
EOF
}

operator_launch_args() {
  printf '%s\n' \
    "start_nav:=false" \
    "start_slam:=false" \
    "start_rf2o:=false" \
    "start_gateway:=false" \
    "start_vision:=true" \
    "wait_for_boundary_topics:=false" \
    "vision_params_file:=${OMNISEER_VISION_PARAMS_FILE:-vision_bridge.real.paths.yaml}" \
    "detector_model_path:=__from_config__" \
    "clip_model_path:=__from_config__" \
    "clip_vocab_path:=__from_config__" \
    "classes_path:=__from_config__"
}

phase3_launch_args() {
  printf '%s\n' \
    "start_nav:=false" \
    "start_slam:=false" \
    "start_rf2o:=false" \
    "start_gateway:=true" \
    "start_vision:=true" \
    "wait_for_boundary_topics:=false" \
    "vision_params_file:=${OMNISEER_VISION_PARAMS_FILE:-vision_bridge.real.paths.yaml}" \
    "detector_model_path:=__from_config__" \
    "clip_model_path:=__from_config__" \
    "clip_vocab_path:=__from_config__" \
    "classes_path:=__from_config__"
}

launch_starts_vision() {
  local start_vision="true"
  local arg
  for arg in "$@"; do
    if [[ "${arg}" == start_vision:=* ]]; then
      start_vision="${arg#start_vision:=}"
    fi
  done

  [[ "${start_vision}" == "true" ]]
}

require_vision_bridge_package() {
  if ! launch_starts_vision "$@"; then
    return
  fi

  if ros2 pkg prefix omniseer_vision_bridge >/dev/null 2>&1; then
    return
  fi

  omni_die "ROS package omniseer_vision_bridge is not installed in the sourced workspace; run 'scripts/omni build ros' on a robot image with RKNN/RGA development files, force it with 'scripts/omni build ros --with-vision', or launch with start_vision:=false"
}

run_teleop() {
  omni_source_ros_workspace
  exec "${script_dir}/teleop.sh"
}

run_verify() {
  omni_source_ros_workspace
  exec "${script_dir}/../check/real_teleop_perception.sh"
}

run_operator_bringup() {
  local extra_args=("$@")
  local launch_args=()
  omni_read_lines_into_array launch_args operator_launch_args
  omni_source_ros_workspace
  require_vision_bridge_package "${launch_args[@]}" "${extra_args[@]}"
  exec ros2 launch bringup real.launch.py "${launch_args[@]}" "${extra_args[@]}"
}

run_background_operator_bringup() {
  local extra_args=("$@")
  local launch_args=()
  omni_read_lines_into_array launch_args operator_launch_args
  omni_source_ros_workspace
  require_vision_bridge_package "${launch_args[@]}" "${extra_args[@]}"

  local bringup_delay_sec="${OMNISEER_BRINGUP_DELAY_SEC:-5}"
  local bringup_log="${OMNISEER_BRINGUP_LOG:-/tmp/omniseer-operator-bringup-$(date -u +%Y%m%dT%H%M%SZ).log}"

  ros2 launch bringup real.launch.py "${launch_args[@]}" "${extra_args[@]}" >"${bringup_log}" 2>&1 &
  bringup_pid=$!

  sleep "${bringup_delay_sec}"

  if ! kill -0 "${bringup_pid}" 2>/dev/null; then
    omni_error "bringup exited early; recent log output follows"
    tail -n 40 "${bringup_log}" >&2 || true
    exit 1
  fi

  omni_info "bringup running with pid ${bringup_pid}"
  omni_info "bringup log: ${bringup_log}"
}

cleanup_background_bringup() {
  if [[ -n "${bringup_pid:-}" ]] && kill -0 "${bringup_pid}" 2>/dev/null; then
    kill "${bringup_pid}" 2>/dev/null || true
    wait "${bringup_pid}" 2>/dev/null || true
  fi
}

run_operator_default() {
  run_background_operator_bringup "$@"
  trap cleanup_background_bringup EXIT INT TERM
  omni_info "Starting keyboard teleop in this terminal"
  omni_info "Use q or Ctrl-C to stop teleop; the background bringup will be cleaned up"
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -p stamped:=true -r cmd_vel:=/cmd_vel_keyboard
}

run_operator_smoke() {
  run_background_operator_bringup "$@"
  trap cleanup_background_bringup EXIT INT TERM
  "${script_dir}/../check/real_teleop_perception.sh"
}

run_phase3_bringup() {
  local extra_args=("$@")
  local launch_args=()
  omni_read_lines_into_array launch_args phase3_launch_args
  omni_source_ros_workspace
  require_vision_bridge_package "${launch_args[@]}" "${extra_args[@]}"
  exec ros2 launch bringup real.launch.py "${launch_args[@]}" "${extra_args[@]}"
}

run_background_phase3_bringup() {
  local extra_args=("$@")
  local launch_args=()
  omni_read_lines_into_array launch_args phase3_launch_args
  omni_source_ros_workspace
  require_vision_bridge_package "${launch_args[@]}" "${extra_args[@]}"

  local bringup_delay_sec="${OMNISEER_BRINGUP_DELAY_SEC:-5}"
  local bringup_log="${OMNISEER_BRINGUP_LOG:-/tmp/omniseer-phase3-bringup-$(date -u +%Y%m%dT%H%M%SZ).log}"

  ros2 launch bringup real.launch.py "${launch_args[@]}" "${extra_args[@]}" >"${bringup_log}" 2>&1 &
  bringup_pid=$!

  sleep "${bringup_delay_sec}"

  if ! kill -0 "${bringup_pid}" 2>/dev/null; then
    omni_error "bringup exited early; recent log output follows"
    tail -n 40 "${bringup_log}" >&2 || true
    exit 1
  fi

  omni_info "bringup running with pid ${bringup_pid}"
  omni_info "bringup log: ${bringup_log}"
}

run_phase3_verify() {
  "${script_dir}/../check/phase3_operator.sh"
}

run_phase3_smoke() {
  run_background_phase3_bringup "$@"
  trap cleanup_background_bringup EXIT INT TERM
  run_phase3_verify
}

run_real_phase_2() {
  local mode="$1"
  shift

  case "${mode}" in
    operator|all)
      run_operator_default "$@"
      ;;
    smoke)
      run_operator_smoke "$@"
      ;;
    bringup)
      run_operator_bringup "$@"
      ;;
    teleop)
      run_teleop
      ;;
    verify|check)
      run_verify
      ;;
    help|-h|--help)
      usage
      ;;
    *)
      omni_die "unknown real mode: ${mode}"
      ;;
  esac
}

run_real_phase_3() {
  local mode="$1"
  shift

  case "${mode}" in
    bringup|demo|all)
      run_phase3_bringup "$@"
      ;;
    smoke)
      run_phase3_smoke "$@"
      ;;
    verify|check)
      omni_source_ros_workspace
      run_phase3_verify
      ;;
    help|-h|--help)
      usage
      ;;
    *)
      omni_die "unknown Phase 3 real mode: ${mode}"
      ;;
  esac
}

resolved_phase=""
mode=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --phase)
      [[ $# -ge 2 ]] || omni_die "--phase requires a numeric argument"
      resolved_phase="$2"
      shift 2
      ;;
    --mode)
      [[ $# -ge 2 ]] || omni_die "--mode requires an argument"
      mode="$2"
      shift 2
      ;;
    help|-h|--help)
      usage
      exit 0
      ;;
    *)
      break
      ;;
  esac
done

if [[ $# -gt 0 ]] && [[ "$1" != *=* && "$1" != --* ]]; then
  mode="$1"
  shift
fi

if [[ -z "${resolved_phase}" ]]; then
  resolved_phase="$(omni_latest_stable_real_phase)"
  omni_info "No real phase specified; defaulting to latest stable phase ${resolved_phase}"
else
  omni_info "Using real phase ${resolved_phase}"
fi

case "${resolved_phase}" in
  2)
    mode="${mode:-operator}"
    run_real_phase_2 "${mode}" "$@"
    ;;
  3)
    mode="${mode:-bringup}"
    run_real_phase_3 "${mode}" "$@"
    ;;
  *)
    omni_die "unsupported real phase ${resolved_phase}; supported phases: $(omni_supported_real_phases | paste -sd, -)"
    ;;
esac
