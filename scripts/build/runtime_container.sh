#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source "${script_dir}/../lib/log.sh"
# shellcheck disable=SC1091
source "${script_dir}/../lib/common.sh"

usage() {
  cat <<'EOF'
Usage:
  scripts/omni build runtime-container [options] [docker build args...]

Builds the Omniseer robot runtime container image. The robot target uses Docker
BuildKit named contexts to inject local RKNN SDK files without adding them to the
repository.

Options:
  --image <name>       Image tag to build. Default: omniseer/robot-runtime:v2
  --target <target>    Docker target: robot-runtime or portable-runtime. Default: robot-runtime
  --ros-distro <name>  ROS distro build arg. Default: kilted
  --micro-ros-agent-ref <ref>
                       micro-ROS-Agent git commit/ref. Default: upstream Kilted commit.
  --rknn-include <p>   RKNN header path for robot-runtime. Default: /usr/include/rknn_api.h
  --rknn-lib <p>       RKNN runtime library for robot-runtime. Auto-detected from ldconfig.

Examples:
  scripts/omni build runtime-container
  scripts/omni build runtime-container --target portable-runtime --image omniseer/robot-runtime:portable
EOF
}

image="omniseer/robot-runtime:v2"
target="robot-runtime"
ros_distro="${OMNISEER_ROS_DISTRO:-kilted}"
micro_ros_agent_ref="${OMNISEER_MICRO_ROS_AGENT_REF:-f0d809138de19a61fe1a640d5c5a3076cd648360}"
rknn_include="${OMNISEER_RKNN_INCLUDE:-/usr/include/rknn_api.h}"
rknn_lib="${OMNISEER_RKNN_LIB:-}"
declare -a docker_args

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image)
      [[ $# -ge 2 ]] || omni_die "--image requires a value"
      image="$2"
      shift 2
      ;;
    --target)
      [[ $# -ge 2 ]] || omni_die "--target requires a value"
      target="$2"
      shift 2
      ;;
    --ros-distro)
      [[ $# -ge 2 ]] || omni_die "--ros-distro requires a value"
      ros_distro="$2"
      shift 2
      ;;
    --micro-ros-agent-ref)
      [[ $# -ge 2 ]] || omni_die "--micro-ros-agent-ref requires a value"
      micro_ros_agent_ref="$2"
      shift 2
      ;;
    --rknn-include)
      [[ $# -ge 2 ]] || omni_die "--rknn-include requires a value"
      rknn_include="$2"
      shift 2
      ;;
    --rknn-lib)
      [[ $# -ge 2 ]] || omni_die "--rknn-lib requires a value"
      rknn_lib="$2"
      shift 2
      ;;
    help|-h|--help)
      usage
      exit 0
      ;;
    *)
      docker_args+=("$1")
      shift
      ;;
  esac
done

case "${target}" in
  robot-runtime|portable-runtime)
    ;;
  *)
    omni_die "unsupported runtime container target: ${target}"
    ;;
esac

omni_require_command docker

repo_root="$(omni_repo_root)"
declare -a build_context_args

if [[ "${target}" == "robot-runtime" ]]; then
  [[ -f "${rknn_include}" ]] || omni_die "RKNN header not found at ${rknn_include}; pass --rknn-include"

  if [[ -z "${rknn_lib}" ]]; then
    if omni_command_available ldconfig; then
      rknn_lib="$(ldconfig -p | awk '/librknnrt\.so/ { print $NF; exit }')"
    fi
  fi

  [[ -n "${rknn_lib}" && -f "${rknn_lib}" ]] || omni_die "RKNN runtime library not found; pass --rknn-lib"

  tmp_context="$(mktemp -d)"
  cleanup() {
    rm -rf "${tmp_context}"
  }
  trap cleanup EXIT

  cp "${rknn_include}" "${tmp_context}/rknn_api.h"
  cp "${rknn_lib}" "${tmp_context}/librknnrt.so"
  build_context_args+=(--build-context "rknn_include=${tmp_context}" --build-context "rknn_lib=${tmp_context}")
fi

omni_info "Building ${image} from docker/runtime/Dockerfile target ${target}"
cd "${repo_root}"
exec env DOCKER_BUILDKIT="${DOCKER_BUILDKIT:-1}" docker build \
  --file docker/runtime/Dockerfile \
  --target "${target}" \
  --tag "${image}" \
  --build-arg "ROS_DISTRO=${ros_distro}" \
  --build-arg "MICRO_ROS_AGENT_REF=${micro_ros_agent_ref}" \
  "${build_context_args[@]}" \
  "${docker_args[@]}" \
  .
