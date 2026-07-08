#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  pre_launch_cleanup.sh [real|sim]
EOF
}

profile="${1:-}"
if [[ "${profile}" != "real" && "${profile}" != "sim" ]]; then
  usage >&2
  exit 2
fi

readonly profile

declare -a shared_patterns=(
  'micro_ros_agent'
  'rplidar_composition'
  'encoder_counts_to_odometry'
  'vision_bridge(_node)?'
  'robot_diag_control_cpp(_node)?'
  'twist_mux'
  'robot_state_publisher'
  'ekf_(node|filter)'
  'rf2o(_laser_odometry(_node)?)?'
  'slam_toolbox|async_slam_toolbox_node'
  'lifecycle_manager_(slam|navigation)|lifecycle_manager'
  'nav2_container|component_container(_mt)?'
  'controller_server'
  'planner_server'
  'bt_navigator'
  'smoother_server'
  'behavior_server'
  'velocity_smoother'
  'collision_monitor'
  'waypoint_follower'
  'docking_server'
  'scan_to_range'
  'ros2_control_node'
  'controller_manager'
  '(^|[^[:alnum:]_])spawner([^[:alnum:]_]|$)'
  '(^|[^[:alnum:]_])jsb([^[:alnum:]_]|$)'
  'joint_state_broadcaster'
  'mecanum_drive_controller'
  'parameter_bridge'
  'image_bridge'
  'ros2[[:space:]]+launch[[:space:]].*(bringup[[:space:]]+.*(real|sim)\.launch\.py|real\.launch\.py|sim\.launch\.py|orchestrate_sim\.launch\.py)'
)

declare -a sim_only_patterns=(
  '(^|[^[:alnum:]_])create([^[:alnum:]_]|$)'
  '(^|[^[:alnum:]_])rviz2([^[:alnum:]_]|$)'
  '(^|[^[:alnum:]_])gz([^[:alnum:]_]|$)'
  'gz[ _-](sim|server|client|gui)'
  'gazebo(server|client)?'
  'ros_gz(_(sim|bridge|image))?'
)

declare -a patterns=("${shared_patterns[@]}")
if [[ "${profile}" == "sim" ]]; then
  patterns+=("${sim_only_patterns[@]}")
fi

declare -A excluded_pids=()
current_pid="$$"
while [[ -n "${current_pid}" && "${current_pid}" != "0" ]]; do
  excluded_pids["${current_pid}"]=1
  current_pid="$(ps -o ppid= -p "${current_pid}" 2>/dev/null | tr -d '[:space:]')"
done

list_matching_pids() {
  local pattern=""
  for pattern in "${patterns[@]}"; do
    while IFS= read -r pid; do
      [[ -z "${pid}" ]] && continue
      [[ -n "${excluded_pids[${pid}]:-}" ]] && continue
      printf '%s\n' "${pid}"
    done < <(pgrep -f -- "${pattern}" 2>/dev/null || true)
  done | sort -u
}

print_matches() {
  local label="$1"
  local pids=()
  mapfile -t pids < <(list_matching_pids)

  echo "=== ${label} (${profile}) ==="
  if [[ "${#pids[@]}" -eq 0 ]]; then
    echo "(none)"
    return
  fi

  local pid=""
  for pid in "${pids[@]}"; do
    ps -o pid= -o args= -p "${pid}" 2>/dev/null || true
  done
}

kill_phase() {
  local signal_name="$1"
  local sleep_seconds="$2"
  local pids=()
  mapfile -t pids < <(list_matching_pids)
  if [[ "${#pids[@]}" -eq 0 ]]; then
    return
  fi

  echo "[cleanup] sending SIG${signal_name} to: ${pids[*]}"
  kill "-${signal_name}" "${pids[@]}" 2>/dev/null || true
  sleep "${sleep_seconds}"
}

print_matches "Lingering bringup-owned processes before cleanup"

echo "[cleanup] stopping ROS 2 daemon"
ros2 daemon stop >/dev/null 2>&1 || true

kill_phase INT 0.5
kill_phase TERM 0.5
kill_phase KILL 0.5

print_matches "Remaining bringup-owned processes after cleanup"

echo "[cleanup] restarting ROS 2 daemon"
ros2 daemon start >/dev/null 2>&1 || true

exit 0
