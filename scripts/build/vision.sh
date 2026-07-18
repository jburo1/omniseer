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
  scripts/omni build vision [cmake --build args...]

Configures and builds the native vision workspace at `vision/build`.
EOF
}

if [[ "${1:-}" =~ ^(-h|--help|help)$ ]]; then
  usage
  exit 0
fi

omni_require_command cmake

repo_root="$(omni_repo_root)"
build_dir="${OMNISEER_VISION_BUILD_DIR:-${repo_root}/vision/build}"
source_dir="${repo_root}/vision"

cache_path="${build_dir}/CMakeCache.txt"
if [[ -f "${cache_path}" ]]; then
  cached_source_dir="$(awk -F= '/^CMAKE_HOME_DIRECTORY:INTERNAL=/ { print $2 }' "${cache_path}")"
  if [[ -n "${cached_source_dir}" && "${cached_source_dir}" != "${source_dir}" ]]; then
    omni_warn "Vision build cache points at ${cached_source_dir}; removing stale build directory ${build_dir}"
    rm -rf "${build_dir}"
  fi
fi

omni_info "Configuring vision build at ${build_dir}"
cmake -S "${source_dir}" -B "${build_dir}"
omni_info "Building vision targets"
exec cmake --build "${build_dir}" "$@"
