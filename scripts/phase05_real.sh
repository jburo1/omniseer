#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat <<'EOF'
Usage:
  scripts/phase05_real.sh [phase05|smoke|bringup|teleop|verify|help] [launch args...]

Modes:
  phase05  Start Phase 0.5 real bringup in the background, then open keyboard teleop.
  smoke    Start Phase 0.5 real bringup, run the passive teleop/vision verifier, then stop.
  bringup  Start only the Phase 0.5 real bringup in the foreground.
  teleop   Start only the stamped keyboard teleop publisher.
  verify   Run only the passive teleop/vision verifier against an existing ROS graph.
  help     Show this help text.

Environment overrides:
  OMNISEER_ROS_SETUP=/opt/ros/kilted/setup.bash
  OMNISEER_WS_SETUP=/omniseer/ros_ws/install/setup.bash
  OMNISEER_VISION_PARAMS_FILE=vision_bridge.real.paths.yaml
  OMNISEER_BRINGUP_DELAY_SEC=5
  OMNISEER_BRINGUP_LOG=/tmp/omniseer-phase05-bringup.log
  OMNISEER_REQUIRE_DETECTIONS=0

Examples:
  scripts/phase05_real.sh
  scripts/phase05_real.sh smoke
  scripts/phase05_real.sh bringup camera_device:=/dev/video11
  scripts/phase05_real.sh phase05 micro_ros_serial_device:=/dev/omniseer_teensy
EOF
}

source_runtime() {
  local ros_setup="${OMNISEER_ROS_SETUP:-/opt/ros/kilted/setup.bash}"
  local ws_setup="${OMNISEER_WS_SETUP:-}"

  if [[ -z "${ws_setup}" ]]; then
    if [[ -f "/ros_ws/install/setup.bash" ]]; then
      ws_setup="/ros_ws/install/setup.bash"
    elif [[ -f "${repo_root}/ros_ws/install/setup.bash" ]]; then
      ws_setup="${repo_root}/ros_ws/install/setup.bash"
    else
      echo "workspace setup.bash not found; set OMNISEER_WS_SETUP" >&2
      exit 2
    fi
  fi

  if [[ ! -f "${ros_setup}" ]]; then
    echo "ROS setup.bash not found at ${ros_setup}" >&2
    exit 2
  fi

  if [[ ! -f "${ws_setup}" ]]; then
    echo "workspace setup.bash not found at ${ws_setup}" >&2
    exit 2
  fi

  set +u
  # shellcheck disable=SC1090
  source "${ros_setup}"
  # shellcheck disable=SC1090
  source "${ws_setup}"
  set -u

  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
}

phase05_launch_args() {
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

run_bringup() {
  local extra_args=("$@")
  local launch_args=()
  mapfile -t launch_args < <(phase05_launch_args)
  source_runtime
  exec ros2 launch bringup real.launch.py "${launch_args[@]}" "${extra_args[@]}"
}

run_teleop() {
  source_runtime
  exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -p stamped:=true -r cmd_vel:=/cmd_vel_keyboard
}

run_verify() {
  source_runtime
  exec "${repo_root}/scripts/check_real_teleop_perception.sh"
}

run_background_bringup() {
  local extra_args=("$@")
  local launch_args=()
  mapfile -t launch_args < <(phase05_launch_args)
  source_runtime

  local bringup_delay_sec="${OMNISEER_BRINGUP_DELAY_SEC:-5}"
  local bringup_log="${OMNISEER_BRINGUP_LOG:-/tmp/omniseer-phase05-bringup-$(date -u +%Y%m%dT%H%M%SZ).log}"

  ros2 launch bringup real.launch.py "${launch_args[@]}" "${extra_args[@]}" >"${bringup_log}" 2>&1 &
  bringup_pid=$!

  sleep "${bringup_delay_sec}"

  if ! kill -0 "${bringup_pid}" 2>/dev/null; then
    echo "bringup exited early; recent log output:" >&2
    tail -n 40 "${bringup_log}" >&2 || true
    exit 1
  fi

  echo "bringup running with pid ${bringup_pid}"
  echo "bringup log: ${bringup_log}"
}

cleanup_background_bringup() {
  if [[ -n "${bringup_pid:-}" ]] && kill -0 "${bringup_pid}" 2>/dev/null; then
    kill "${bringup_pid}" 2>/dev/null || true
    wait "${bringup_pid}" 2>/dev/null || true
  fi
}

run_phase05() {
  run_background_bringup "$@"
  trap cleanup_background_bringup EXIT INT TERM
  echo "keyboard teleop is starting in this terminal"
  echo "use q or Ctrl-C to stop teleop; the background bringup will be cleaned up"
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -p stamped:=true -r cmd_vel:=/cmd_vel_keyboard
}

run_smoke() {
  run_background_bringup "$@"
  trap cleanup_background_bringup EXIT INT TERM
  "${repo_root}/scripts/check_real_teleop_perception.sh"
}

mode="${1:-phase05}"
if [[ $# -gt 0 ]]; then
  shift
fi

case "${mode}" in
  phase05|all)
    run_phase05 "$@"
    ;;
  smoke)
    run_smoke "$@"
    ;;
  bringup)
    run_bringup "$@"
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
    echo "unknown mode: ${mode}" >&2
    usage >&2
    exit 2
    ;;
esac
