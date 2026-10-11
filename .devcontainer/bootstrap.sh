#!/usr/bin/env bash
set -eo pipefail

WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." >/dev/null 2>&1 && pwd)"

with_micro_ros=false
ros_args=()
for arg in "$@"; do
  case "${arg}" in
    --with-micro-ros) with_micro_ros=true ;;
    --with-yolo|--with-vision|--without-vision) ros_args+=("${arg}") ;;
    *) echo "Unknown bootstrap option: ${arg}" >&2; exit 64 ;;
  esac
done

# Common environment
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "/opt/venv/bin/activate"
source "${WS}/scripts/lib/common.sh"

# Match the supported portable dependency set, with optional providers added.
omni_read_lines_into_array dependency_paths omni_ros_dep_paths_core
vision_mode=auto
for arg in "${ros_args[@]}"; do
  case "${arg}" in
    --with-yolo) dependency_paths+=(ros_ws/src/yolo_ros/yolo_ros ros_ws/src/yolo_ros/yolo_bringup) ;;
    --with-vision) vision_mode=force ;;
    --without-vision) vision_mode=skip ;;
  esac
done
if [[ "${vision_mode}" == force ]] || { [[ "${vision_mode}" == auto ]] && omni_vision_bridge_deps_available; }; then
  dependency_paths+=(ros_ws/src/omniseer_vision_bridge)
fi
cd "${WS}"
sudo -E "${WS}/scripts/omni" setup ros-deps "${dependency_paths[@]}"

# The checked-in Python gRPC/protobuf stubs target newer runtimes than Ubuntu noble
# ships, so install matching wheels into the workspace venv before ROS package use.
python -m pip install --upgrade \
  "grpcio>=1.71.2,<2" \
  "protobuf>=5.29.0,<6"

build_micro_ros_ws() {
  local ws_path="$1"

  echo "=== Building workspace: ${ws_path} ==="
  cd "${ws_path}"

  rosdep install --from-paths src \
    --rosdistro "${ROS_DISTRO}" \
    --ignore-src -r -y \
    --as-root apt:true --as-root pip:false

  # Reuse the existing colcon layout without deleting workspace artifacts.
  local layout=isolated
  local layout_args=()
  if [[ -f install/.colcon_install_layout ]]; then
    layout="$(cat install/.colcon_install_layout)"
  fi
  case "${layout}" in
    merged) layout_args+=(--merge-install) ;;
    isolated) ;;
    *) echo "Unsupported micro-ROS install layout: ${layout}" >&2; return 1 ;;
  esac

  python -m colcon build --symlink-install "${layout_args[@]}" \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DPython3_EXECUTABLE=/opt/venv/bin/python \
      -DPython3_ROOT_DIR=/opt/venv \
      -DPython3_FIND_VIRTUALENV=ONLY \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

  source "install/setup.bash"
}

# Use SDK detection and the portable package selection owned by scripts/omni.
"${WS}/scripts/omni" build ros "${ros_args[@]}" --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DPython3_EXECUTABLE=/opt/venv/bin/python \
    -DPython3_ROOT_DIR=/opt/venv \
    -DPython3_FIND_VIRTUALENV=ONLY \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source "${WS}/ros_ws/install/setup.bash"

# Only needed for direct micro-ROS/Teensy development or robot bringup.
if [[ "${with_micro_ros}" == true ]]; then
  build_micro_ros_ws "${WS}/uros_ws"
fi

grep -q "Omniseer ROS overlays" ~/.bashrc || cat >> ~/.bashrc <<'EOF'

# Omniseer ROS overlays
source /opt/ros/${ROS_DISTRO}/setup.bash
[ ! -f /omniseer/uros_ws/install/setup.bash ] || source /omniseer/uros_ws/install/setup.bash
source /omniseer/ros_ws/install/setup.bash
EOF
