#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
original_args=("$@")

# shellcheck disable=SC1091
source "${script_dir}/../lib/log.sh"
# shellcheck disable=SC1091
source "${script_dir}/../lib/common.sh"
# shellcheck disable=SC1091
source "${script_dir}/../lib/ros.sh"

usage() {
  cat <<'EOF'
Usage:
  scripts/omni run real [--profile <name>] [recording flags] [--mode <mode>] [mode] [launch args...]

Profiles:
  current        Default alias for the current supported real robot runtime.
  operator       Gateway, native vision, preview, bounded teleop, and recording.
  perception     Native vision and recording without the gateway operator surface.

Modes:
  bringup  Start the selected real profile in the foreground.
  smoke    Start the selected profile, run its verifier, then stop.
  verify   Run the selected profile verifier against an existing ROS graph.
  teleop   Start only the stamped keyboard teleop publisher.
  operator  Legacy-teleop profile only: start bringup, then open keyboard teleop.

Examples:
  scripts/omni run real
  scripts/omni run real smoke
  scripts/omni run real verify
  scripts/omni run real --record-run demo_001
  scripts/omni run real --profile perception --record-run demo_001
  scripts/omni run real --profile operator micro_ros_serial_device:=/dev/serial/by-id/usb-Teensyduino_USB_Serial_16634450-if00

Recording flags:
  --record                         Record a timestamped perception run bundle.
  --record-run <run_id>            Record a named perception run bundle.
  --record-out <path>              Override output directory; defaults to runs/<run_id>.
  --record-system-interval-sec <s> Sample system/platform telemetry every s seconds. Default: 1.0.
  --record-notes <text>            Store notes in manifest.yaml.
  --record-classes <text>          Store configured class names in manifest.yaml.
  --record-model-family <text>     Store explicit detector family in manifest.yaml.
  --record-model-variant <text>    Store explicit detector variant in manifest.yaml.
  --record-model-precision <text>  Store explicit detector precision in manifest.yaml.
  --record-model-backend <text>    Store explicit detector backend in manifest.yaml.
  --record-container-image-ref <ref>
                                   Store container image reference in manifest.yaml.
  --record-container-image-digest <digest>
                                   Store container image digest in manifest.yaml.
  --record-experiment-config <text>
                                   Store experiment config identifier or path in manifest.yaml.
  --record-experiment-parameters <items>
                                   Store experiment parameters as JSON or key=value pairs.
  --record-experiment-parameter <key=value>
                                   Store one experiment parameter; may be repeated.
  --record-comparison-id <text>    Store an optional comparison experiment ID.
  --record-trial <text>            Store an optional comparison trial ID.
  --record-workload-id <text>      Store an optional comparison workload ID.
  --record-overwrite               Replace an existing output directory.
  --record-video                   Record gateway camera video to video/source.ts.
  --record-rosbag                  Record the configured ROS topic allowlist to rosbag/.
EOF
}

legacy_usage() {
  cat <<'EOF'
Legacy diagnostic profile:
  scripts/omni run real --profile legacy-teleop [operator|bringup|smoke]

This profile is retained only for older keyboard teleop hardware diagnostics.
Prefer --profile operator or --profile perception for current real robot work.
EOF
}

legacy_teleop_launch_args() {
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

perception_launch_args() {
  legacy_teleop_launch_args
}

operator_launch_args() {
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

recording_requested() {
  [[ "${record_enabled}" == "true" ]]
}

shell_join() {
  local parts=()
  local arg
  for arg in "$@"; do
    parts+=("$(printf '%q' "${arg}")")
  done
  local IFS=" "
  printf '%s\n' "${parts[*]}"
}

recording_launch_command() {
  if [[ -n "${OMNISEER_RUNTIME_CONTAINER_COMMAND:-}" ]]; then
    printf '%s\n' "${OMNISEER_RUNTIME_CONTAINER_COMMAND}"
    return
  fi

  printf 'scripts/omni run real'
  if [[ ${#original_args[@]} -gt 0 ]]; then
    printf ' %s' "$(shell_join "${original_args[@]}")"
  fi
  printf '\n'
}

resolve_recording_defaults() {
  if ! recording_requested; then
    return
  fi

  if [[ -z "${record_run_id}" ]]; then
    record_run_id="$(date -u +%Y%m%dT%H%M%SZ)"
  fi

  if [[ -z "${record_out_dir}" ]]; then
    record_out_dir="runs/${record_run_id}"
  fi
}

prepare_recording_run_dir() {
  if ! recording_requested; then
    return
  fi

  resolve_recording_defaults
  if [[ "${record_out_dir}" == "/" || -z "${record_out_dir}" ]]; then
    omni_die "refusing unsafe run bundle output directory: ${record_out_dir}"
  fi

  if [[ "${record_overwrite}" == "true" ]]; then
    rm -rf -- "${record_out_dir}"
  fi
  mkdir -p -- "${record_out_dir}"
  if [[ -n "${record_classes}" ]]; then
    printf '%s\n' "${record_classes}" \
      | tr ',' '\n' \
      | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//' -e '/^$/d' \
      >"${record_out_dir}/classes.txt"
  fi
  if [[ "${record_video}" == "true" ]]; then
    mkdir -p -- "${record_out_dir}/video"
  fi
}

recording_bringup_log_path() {
  resolve_recording_defaults
  printf '%s/logs/bringup.log\n' "${record_out_dir}"
}

bringup_log_path() {
  local profile_name="$1"
  if recording_requested; then
    recording_bringup_log_path
    return
  fi

  printf '%s\n' "${OMNISEER_BRINGUP_LOG:-/tmp/omniseer-${profile_name}-bringup-$(date -u +%Y%m%dT%H%M%SZ).log}"
}

prepare_bringup_log_file() {
  local bringup_log="$1"
  mkdir -p -- "$(dirname "${bringup_log}")"
  touch -- "${bringup_log}"
}

require_no_recording() {
  if recording_requested; then
    omni_die "recording flags require a mode that launches real bringup"
  fi
}

append_recording_launch_args() {
  local -n target_args="$1"
  shift
  local extra_args=("$@")

  if ! recording_requested; then
    return
  fi

  resolve_recording_defaults
  local launch_command launch_args_text
  launch_command="$(recording_launch_command)"
  launch_args_text="$(shell_join "${target_args[@]}" "${extra_args[@]}")"

  target_args+=(
    "start_experiment_recording:=true"
    "experiment_run_id:=${record_run_id}"
    "experiment_out_dir:=${record_out_dir}"
    "experiment_system_interval_sec:=${record_system_interval_sec}"
    "experiment_overwrite:=${record_overwrite}"
    "pipeline_telemetry_path:=${record_out_dir}/pipeline_telemetry.jsonl"
    "evidence_dir:=${record_out_dir}/evidence"
    "resolved_vision_config_path:=${record_out_dir}/provenance/resolved_vision_config.yaml"
    "experiment_launch_command:=${launch_command}"
    "experiment_launch_profile:=${resolved_profile}"
    "experiment_launch_mode:=${mode}"
    "experiment_launch_args:=${launch_args_text}"
  )
  if [[ "${record_video}" == "true" ]]; then
    target_args+=(
      "start_gateway:=true"
      "gateway_preview_record_path:=${record_out_dir}/video/source.ts"
    )
  fi
  if [[ "${record_rosbag}" == "true" ]]; then
    target_args+=("start_rosbag_recording:=true")
  fi

  if [[ -n "${record_notes}" ]]; then
    target_args+=("experiment_notes:=${record_notes}")
  fi

  if [[ -n "${record_classes}" ]]; then
    target_args+=("experiment_classes:=${record_classes}")
  fi

  if [[ -n "${record_model_family}" ]]; then
    target_args+=("experiment_model_family:=${record_model_family}")
  fi
  if [[ -n "${record_model_variant}" ]]; then
    target_args+=("experiment_model_variant:=${record_model_variant}")
  fi
  if [[ -n "${record_model_precision}" ]]; then
    target_args+=("experiment_model_precision:=${record_model_precision}")
  fi
  if [[ -n "${record_model_backend}" ]]; then
    target_args+=("experiment_model_backend:=${record_model_backend}")
  fi
  if [[ -n "${record_comparison_id}" ]]; then
    target_args+=("experiment_comparison_id:=${record_comparison_id}")
  fi
  if [[ -n "${record_trial}" ]]; then
    target_args+=("experiment_trial:=${record_trial}")
  fi
  if [[ -n "${record_workload_id}" ]]; then
    target_args+=("experiment_workload_id:=${record_workload_id}")
  fi

  local container_image_ref="${record_container_image_ref:-${OMNISEER_CONTAINER_IMAGE_REF:-}}"
  local container_image_digest="${record_container_image_digest:-${OMNISEER_CONTAINER_IMAGE_DIGEST:-}}"
  local experiment_config="${record_experiment_config:-${OMNISEER_EXPERIMENT_CONFIG:-}}"
  local experiment_parameters="${OMNISEER_EXPERIMENT_PARAMETERS:-}"
  local parameter_item
  if [[ ${#record_experiment_parameters[@]} -gt 0 ]]; then
    experiment_parameters=""
    for parameter_item in "${record_experiment_parameters[@]}"; do
      if [[ -z "${experiment_parameters}" ]]; then
        experiment_parameters="${parameter_item}"
      else
        experiment_parameters="${experiment_parameters},${parameter_item}"
      fi
    done
  fi

  if [[ -n "${container_image_ref}" ]]; then
    target_args+=("experiment_container_image_ref:=${container_image_ref}")
  fi

  if [[ -n "${container_image_digest}" ]]; then
    target_args+=("experiment_container_image_digest:=${container_image_digest}")
  fi

  if [[ -n "${experiment_config}" ]]; then
    target_args+=("experiment_config:=${experiment_config}")
  fi

  if [[ -n "${experiment_parameters}" ]]; then
    target_args+=("experiment_parameters:=${experiment_parameters}")
  fi
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

run_profile_bringup() {
  local launch_args_producer="$1"
  shift
  local extra_args=("$@")
  local launch_args=()
  omni_read_lines_into_array launch_args "${launch_args_producer}"
  prepare_recording_run_dir
  append_recording_launch_args launch_args "${extra_args[@]}"
  omni_source_ros_workspace
  require_vision_bridge_package "${launch_args[@]}" "${extra_args[@]}"
  if recording_requested; then
    local bringup_log
    bringup_log="$(recording_bringup_log_path)"
    prepare_bringup_log_file "${bringup_log}"
    omni_info "bringup log: ${bringup_log}"
    ros2 launch bringup real.launch.py "${launch_args[@]}" "${extra_args[@]}" 2>&1 | tee -a "${bringup_log}"
    return "${PIPESTATUS[0]}"
  fi

  exec ros2 launch bringup real.launch.py "${launch_args[@]}" "${extra_args[@]}"
}

run_background_profile_bringup() {
  local profile_name="$1"
  local launch_args_producer="$2"
  shift 2
  local extra_args=("$@")
  local launch_args=()
  omni_read_lines_into_array launch_args "${launch_args_producer}"
  prepare_recording_run_dir
  append_recording_launch_args launch_args "${extra_args[@]}"
  omni_source_ros_workspace
  require_vision_bridge_package "${launch_args[@]}" "${extra_args[@]}"

  local bringup_delay_sec="${OMNISEER_BRINGUP_DELAY_SEC:-5}"
  local bringup_log
  bringup_log="$(bringup_log_path "${profile_name}")"
  prepare_bringup_log_file "${bringup_log}"

  (
    # Bash backgrounds asynchronous commands with SIGINT ignored. Restore the
    # default before exec so cleanup_background_bringup can request the normal
    # ROS 2 graceful shutdown path.
    trap - INT TERM
    exec ros2 launch bringup real.launch.py "${launch_args[@]}" "${extra_args[@]}"
  ) >>"${bringup_log}" 2>&1 &
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
    # ros2 launch handles SIGINT as an orderly shutdown and gives recorder nodes
    # a chance to flush their final RunBundle manifest and summary.
    kill -INT "${bringup_pid}" 2>/dev/null || true
    wait "${bringup_pid}" 2>/dev/null || true
  fi

  wait_for_recording_finalization
}

wait_for_recording_finalization() {
  if ! recording_requested; then
    return
  fi

  resolve_recording_defaults
  local summary_path="${record_out_dir}/summary.json"
  local timeout_sec="${OMNISEER_RECORD_FINALIZE_TIMEOUT_SEC:-10}"
  local deadline=$((SECONDS + timeout_sec))
  while [[ ! -f "${summary_path}" && ${SECONDS} -lt ${deadline} ]]; do
    sleep 0.1
  done

  if [[ ! -f "${summary_path}" ]]; then
    omni_warn "RunBundle did not finalize within ${timeout_sec}s: ${summary_path}"
  fi
}

run_legacy_teleop_default() {
  run_background_profile_bringup "legacy-teleop" legacy_teleop_launch_args "$@"
  trap cleanup_background_bringup EXIT INT TERM
  omni_info "Starting keyboard teleop in this terminal"
  omni_info "Use q or Ctrl-C to stop teleop; the background bringup will be cleaned up"
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -p stamped:=true -r cmd_vel:=/cmd_vel_keyboard
}

run_legacy_teleop_smoke() {
  run_background_profile_bringup "legacy-teleop" legacy_teleop_launch_args "$@"
  trap cleanup_background_bringup EXIT INT TERM
  "${script_dir}/../check/real_teleop_perception.sh"
}

run_integrated_verify() {
  "${script_dir}/../check/real_operator.sh"
}

run_operator_smoke() {
  run_background_profile_bringup "operator" operator_launch_args "$@"
  trap cleanup_background_bringup EXIT INT TERM
  run_integrated_verify
}

run_perception_smoke() {
  run_background_profile_bringup "perception" perception_launch_args "$@"
  trap cleanup_background_bringup EXIT INT TERM
  "${script_dir}/../check/real_teleop_perception.sh"
}

run_real_profile_legacy_teleop() {
  local mode="$1"
  shift

  case "${mode}" in
    operator|all)
      run_legacy_teleop_default "$@"
      ;;
    smoke)
      run_legacy_teleop_smoke "$@"
      ;;
    bringup)
      run_profile_bringup legacy_teleop_launch_args "$@"
      ;;
    teleop)
      require_no_recording
      run_teleop
      ;;
    verify|check)
      require_no_recording
      run_verify
      ;;
    help|-h|--help)
      usage
      legacy_usage
      ;;
    *)
      omni_die "unknown real mode: ${mode}"
      ;;
  esac
}

run_real_profile_operator() {
  local mode="$1"
  shift

  case "${mode}" in
    bringup|demo|all)
      run_profile_bringup operator_launch_args "$@"
      ;;
    smoke)
      run_operator_smoke "$@"
      ;;
    verify|check)
      require_no_recording
      omni_source_ros_workspace
      run_integrated_verify
      ;;
    teleop)
      require_no_recording
      run_teleop
      ;;
    help|-h|--help)
      usage
      ;;
    *)
      omni_die "unknown real mode: ${mode}"
      ;;
  esac
}

run_real_profile_perception() {
  local mode="$1"
  shift

  case "${mode}" in
    bringup|demo|all)
      run_profile_bringup perception_launch_args "$@"
      ;;
    smoke)
      run_perception_smoke "$@"
      ;;
    verify|check)
      require_no_recording
      omni_source_ros_workspace
      "${script_dir}/../check/real_teleop_perception.sh"
      ;;
    teleop)
      require_no_recording
      run_teleop
      ;;
    help|-h|--help)
      usage
      ;;
    *)
      omni_die "unknown real mode: ${mode}"
      ;;
  esac
}

resolve_real_profile() {
  local profile="$1"
  case "${profile}" in
    current)
      local current_profile
      current_profile="$(omni_current_real_profile)"
      [[ "${current_profile}" != "current" ]] || omni_die "OMNISEER_CURRENT_REAL_PROFILE must not be current"
      resolve_real_profile "${current_profile}"
      ;;
    operator|integrated|operator-integrated)
      printf '%s\n' "operator"
      ;;
    perception)
      printf '%s\n' "perception"
      ;;
    legacy-teleop|legacy|phase2)
      printf '%s\n' "legacy-teleop"
      ;;
    *)
      omni_die "unsupported real profile ${profile}; supported profiles: $(omni_supported_real_profiles | paste -sd, -)"
      ;;
  esac
}

requested_profile=""
mode=""
record_enabled=false
record_run_id=""
record_out_dir=""
record_system_interval_sec="1.0"
record_notes=""
record_classes=""
record_model_family=""
record_model_variant=""
record_model_precision=""
record_model_backend=""
record_container_image_ref=""
record_container_image_digest=""
record_experiment_config=""
record_experiment_parameters=()
record_comparison_id=""
record_trial=""
record_workload_id=""
record_overwrite="false"
record_video="false"
record_rosbag="false"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)
      [[ $# -ge 2 ]] || omni_die "--profile requires a profile name"
      [[ -z "${requested_profile}" ]] || omni_die "real profile specified more than once"
      requested_profile="$2"
      shift 2
      ;;
    --phase)
      [[ $# -ge 2 ]] || omni_die "--phase requires a numeric argument"
      [[ -z "${requested_profile}" ]] || omni_die "use either --profile or --phase, not both"
      case "$2" in
        2)
          requested_profile="legacy-teleop"
          ;;
        3)
          requested_profile="operator"
          ;;
        *)
          omni_die "unsupported real phase $2; use --profile instead"
          ;;
      esac
      omni_warn "--phase is deprecated; use --profile ${requested_profile}"
      shift 2
      ;;
    --mode)
      [[ $# -ge 2 ]] || omni_die "--mode requires an argument"
      mode="$2"
      shift 2
      ;;
    --record)
      record_enabled=true
      shift
      ;;
    --record-run)
      [[ $# -ge 2 ]] || omni_die "--record-run requires a run id"
      record_enabled=true
      record_run_id="$2"
      shift 2
      ;;
    --record-out)
      [[ $# -ge 2 ]] || omni_die "--record-out requires a path"
      record_enabled=true
      record_out_dir="$2"
      shift 2
      ;;
    --record-system-interval-sec)
      [[ $# -ge 2 ]] || omni_die "--record-system-interval-sec requires a numeric argument"
      record_enabled=true
      record_system_interval_sec="$2"
      shift 2
      ;;
    --record-notes)
      [[ $# -ge 2 ]] || omni_die "--record-notes requires text"
      record_enabled=true
      record_notes="$2"
      shift 2
      ;;
    --record-classes)
      [[ $# -ge 2 ]] || omni_die "--record-classes requires text"
      record_enabled=true
      record_classes="$2"
      shift 2
      ;;
    --record-model-family)
      [[ $# -ge 2 ]] || omni_die "--record-model-family requires text"
      record_enabled=true
      record_model_family="$2"
      shift 2
      ;;
    --record-model-variant)
      [[ $# -ge 2 ]] || omni_die "--record-model-variant requires text"
      record_enabled=true
      record_model_variant="$2"
      shift 2
      ;;
    --record-model-precision)
      [[ $# -ge 2 ]] || omni_die "--record-model-precision requires text"
      record_enabled=true
      record_model_precision="$2"
      shift 2
      ;;
    --record-model-backend)
      [[ $# -ge 2 ]] || omni_die "--record-model-backend requires text"
      record_enabled=true
      record_model_backend="$2"
      shift 2
      ;;
    --record-container-image-ref)
      [[ $# -ge 2 ]] || omni_die "--record-container-image-ref requires an image reference"
      record_enabled=true
      record_container_image_ref="$2"
      shift 2
      ;;
    --record-container-image-digest)
      [[ $# -ge 2 ]] || omni_die "--record-container-image-digest requires an image digest"
      record_enabled=true
      record_container_image_digest="$2"
      shift 2
      ;;
    --record-experiment-config)
      [[ $# -ge 2 ]] || omni_die "--record-experiment-config requires text"
      record_enabled=true
      record_experiment_config="$2"
      shift 2
      ;;
    --record-experiment-parameters)
      [[ $# -ge 2 ]] || omni_die "--record-experiment-parameters requires JSON or key=value pairs"
      record_enabled=true
      record_experiment_parameters+=("$2")
      shift 2
      ;;
    --record-experiment-parameter)
      [[ $# -ge 2 ]] || omni_die "--record-experiment-parameter requires key=value"
      record_enabled=true
      record_experiment_parameters+=("$2")
      shift 2
      ;;
    --record-comparison-id)
      [[ $# -ge 2 ]] || omni_die "--record-comparison-id requires text"
      record_enabled=true
      record_comparison_id="$2"
      shift 2
      ;;
    --record-trial)
      [[ $# -ge 2 ]] || omni_die "--record-trial requires text"
      record_enabled=true
      record_trial="$2"
      shift 2
      ;;
    --record-workload-id)
      [[ $# -ge 2 ]] || omni_die "--record-workload-id requires text"
      record_enabled=true
      record_workload_id="$2"
      shift 2
      ;;
    --record-overwrite)
      record_enabled=true
      record_overwrite="true"
      shift
      ;;
    --record-video)
      record_enabled=true
      record_video="true"
      shift
      ;;
    --record-rosbag)
      record_enabled=true
      record_rosbag="true"
      shift
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

if [[ -z "${requested_profile}" ]]; then
  requested_profile="$(omni_default_real_profile)"
fi

resolved_profile="$(resolve_real_profile "${requested_profile}")"
if [[ "${requested_profile}" == "current" ]]; then
  omni_info "Using real profile current (${resolved_profile})"
else
  omni_info "Using real profile ${resolved_profile}"
fi

case "${resolved_profile}" in
  legacy-teleop)
    mode="${mode:-operator}"
    run_real_profile_legacy_teleop "${mode}" "$@"
    ;;
  operator)
    mode="${mode:-bringup}"
    run_real_profile_operator "${mode}" "$@"
    ;;
  perception)
    mode="${mode:-bringup}"
    run_real_profile_perception "${mode}" "$@"
    ;;
  *)
    omni_die "unsupported real profile ${resolved_profile}"
    ;;
esac
