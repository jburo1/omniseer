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
  scripts/omni run sim [--autonomy] [--yolo] [--yolo-device <device>] [--yolo-model <path>] [launch args...]

Launches the existing simulation bringup helper path.

Options:
  --autonomy             Start slim visual target-centering autonomy with the sim YOLO provider.
  --yolo                 Start the sim YOLO provider and /yolo/dbg_image overlay.
  --yolo-device <device> Override the YOLO inference device, for example cpu or cuda:0.
  --yolo-model <path>    Override the YOLO-World model path.
EOF
}

if [[ "${1:-}" =~ ^(-h|--help|help)$ ]]; then
  usage
  exit 0
fi

omni_source_ros_workspace

declare -a launch_args=()
start_autonomy=false
start_yolo=false
has_start_autonomy_arg=false
has_start_yolo_arg=false
has_yolo_model_arg=false
has_yolo_image_reliability_arg=false
has_start_slam_arg=false
has_start_rf2o_arg=false
has_start_nav_arg=false
has_start_rviz_arg=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --autonomy)
      start_autonomy=true
      start_yolo=true
      shift
      ;;
    --yolo)
      start_yolo=true
      shift
      ;;
    --yolo-device)
      if [[ $# -lt 2 ]]; then
        omni_die "--yolo-device requires a value"
      fi
      start_yolo=true
      launch_args+=("yolo_device:=$2")
      shift 2
      ;;
    --yolo-device=*)
      start_yolo=true
      launch_args+=("yolo_device:=${1#--yolo-device=}")
      shift
      ;;
    --yolo-model)
      if [[ $# -lt 2 ]]; then
        omni_die "--yolo-model requires a value"
      fi
      start_yolo=true
      has_yolo_model_arg=true
      launch_args+=("yolo_model:=$2")
      shift 2
      ;;
    --yolo-model=*)
      start_yolo=true
      has_yolo_model_arg=true
      launch_args+=("yolo_model:=${1#--yolo-model=}")
      shift
      ;;
    --yolo-cpu)
      start_yolo=true
      launch_args+=("yolo_device:=cpu")
      shift
      ;;
    start_yolo:=*)
      has_start_yolo_arg=true
      case "${1#start_yolo:=}" in
        true|True|TRUE|1)
          start_yolo=true
          ;;
        *)
          start_yolo=false
          ;;
      esac
      launch_args+=("$1")
      shift
      ;;
    start_autonomy:=*)
      has_start_autonomy_arg=true
      case "${1#start_autonomy:=}" in
        true|True|TRUE|1)
          start_autonomy=true
          ;;
        *)
          start_autonomy=false
          ;;
      esac
      launch_args+=("$1")
      shift
      ;;
    start_slam:=*)
      has_start_slam_arg=true
      launch_args+=("$1")
      shift
      ;;
    start_rf2o:=*)
      has_start_rf2o_arg=true
      launch_args+=("$1")
      shift
      ;;
    start_nav:=*)
      has_start_nav_arg=true
      launch_args+=("$1")
      shift
      ;;
    start_rviz:=*)
      has_start_rviz_arg=true
      launch_args+=("$1")
      shift
      ;;
    yolo_model:=*)
      has_yolo_model_arg=true
      launch_args+=("$1")
      shift
      ;;
    yolo_image_reliability:=*)
      has_yolo_image_reliability_arg=true
      launch_args+=("$1")
      shift
      ;;
    *)
      launch_args+=("$1")
      shift
      ;;
  esac
done

if [[ "${start_yolo}" == true ]]; then
  if ! ros2 pkg prefix yolo_bringup >/dev/null 2>&1 || ! ros2 pkg prefix yolo_ros >/dev/null 2>&1; then
    omni_die \
      "YOLO sim support is not installed in the sourced workspace;" \
      "run 'scripts/omni build ros --with-yolo' first"
  fi

  yolo_runtime_python="python3"
  yolo_prefix="$(ros2 pkg prefix yolo_ros 2>/dev/null || true)"
  yolo_entrypoint="${yolo_prefix}/lib/yolo_ros/yolo_node"
  if [[ -f "${yolo_entrypoint}" ]]; then
    IFS= read -r yolo_shebang <"${yolo_entrypoint}"
    if [[ "${yolo_shebang}" == "#!"* ]]; then
      yolo_runtime_python="${yolo_shebang#\#!}"
    fi
  fi
  read -r -a yolo_runtime_python_cmd <<<"${yolo_runtime_python}"

  if ! "${yolo_runtime_python_cmd[@]}" -c "import torch; import ultralytics" >/dev/null 2>&1; then
    omni_die \
      "YOLO sim support needs Python 'torch' and 'ultralytics' available to" \
      "the installed yolo_ros runtime interpreter (${yolo_runtime_python});" \
      "rebuild it with 'scripts/omni build ros --with-yolo'"
  fi
  if [[ "${has_start_yolo_arg}" == false ]]; then
    launch_args+=("start_yolo:=true")
  fi
  workspace_yolo_model="$(omni_ros_ws_root)/yolov8s-worldv2.pt"
  if [[ "${has_yolo_model_arg}" == false && -f "${workspace_yolo_model}" ]]; then
    launch_args+=("yolo_model:=${workspace_yolo_model}")
  fi
  if [[ "${has_yolo_image_reliability_arg}" == false ]]; then
    launch_args+=("yolo_image_reliability:=2")
  fi
fi

if [[ "${start_autonomy}" == true && "${has_start_autonomy_arg}" == false ]]; then
  launch_args+=("start_autonomy:=true")
fi

if [[ "${start_autonomy}" == true ]]; then
  if [[ "${has_start_slam_arg}" == false ]]; then
    launch_args+=("start_slam:=false")
  fi
  if [[ "${has_start_rf2o_arg}" == false ]]; then
    launch_args+=("start_rf2o:=false")
  fi
  if [[ "${has_start_nav_arg}" == false ]]; then
    launch_args+=("start_nav:=false")
  fi
  if [[ "${has_start_rviz_arg}" == false ]]; then
    launch_args+=("start_rviz:=false")
  fi
fi

omni_info "Launching simulation bringup"
exec ros2 launch bringup sim.launch.py "${launch_args[@]}"
