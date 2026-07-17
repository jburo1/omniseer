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

declare -A runtime_executables=(
  [micro_ros_agent]=1
  [rplidar_composition]=1
  [encoder_counts_to_odometry]=1
  [vision_bridge]=1
  [vision_bridge_node]=1
  [robot_diag_control_cpp]=1
  [robot_diag_control_cpp_node]=1
  [twist_mux]=1
  [robot_state_publisher]=1
  [ekf_node]=1
  [ekf_filter]=1
  [rf2o]=1
  [rf2o_laser_odometry]=1
  [rf2o_laser_odometry_node]=1
  [slam_toolbox]=1
  [async_slam_toolbox_node]=1
  [lifecycle_manager]=1
  [lifecycle_manager_slam]=1
  [lifecycle_manager_navigation]=1
  [nav2_container]=1
  [component_container]=1
  [component_container_mt]=1
  [controller_server]=1
  [planner_server]=1
  [bt_navigator]=1
  [smoother_server]=1
  [behavior_server]=1
  [velocity_smoother]=1
  [collision_monitor]=1
  [waypoint_follower]=1
  [docking_server]=1
  [scan_to_range]=1
  [ros2_control_node]=1
  [controller_manager]=1
  [spawner]=1
  [jsb]=1
  [joint_state_broadcaster]=1
  [mecanum_drive_controller]=1
  [parameter_bridge]=1
  [image_bridge]=1
)

if [[ "${profile}" == "sim" ]]; then
  runtime_executables[create]=1
  runtime_executables[rviz2]=1
  runtime_executables[gz]=1
  runtime_executables[gz_sim]=1
  runtime_executables[gzserver]=1
  runtime_executables[gzclient]=1
  runtime_executables[gazebo]=1
  runtime_executables[gazeboserver]=1
  runtime_executables[gazeboclient]=1
  runtime_executables[ros_gz_sim]=1
  runtime_executables[ros_gz_bridge]=1
  runtime_executables[ros_gz_image]=1
fi

readonly launch_pattern='ros2[[:space:]]+launch[[:space:]].*(bringup[[:space:]]+.*(real|sim)\.launch\.py|real\.launch\.py|sim\.launch\.py|orchestrate_sim\.launch\.py)'

declare -A excluded_pids=()
current_pid="$$"
while [[ -n "${current_pid}" && "${current_pid}" != "0" ]]; do
  excluded_pids["${current_pid}"]=1
  current_pid="$(ps -o ppid= -p "${current_pid}" 2>/dev/null | tr -d '[:space:]')"
done

list_matching_pids() {
  local proc_path=""
  local pid=""
  local argument=""
  local executable=""
  local -a arguments=()

  for proc_path in /proc/[0-9]*; do
    pid="${proc_path##*/}"
    [[ -n "${excluded_pids[${pid}]:-}" ]] && continue
    [[ -r "${proc_path}/cmdline" ]] || continue

    arguments=()
    mapfile -d '' -t arguments <"${proc_path}/cmdline" 2>/dev/null || continue
    for argument in "${arguments[@]:0:2}"; do
      [[ -z "${argument}" ]] && continue
      executable="${argument##*/}"
      if [[ -n "${runtime_executables[${executable}]:-}" ]]; then
        printf '%s\n' "${pid}"
        break
      fi
    done
  done

  while IFS= read -r pid; do
    [[ -z "${pid}" ]] && continue
    [[ -n "${excluded_pids[${pid}]:-}" ]] && continue
    printf '%s\n' "${pid}"
  done < <(pgrep -f -- "${launch_pattern}" 2>/dev/null || true)
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
