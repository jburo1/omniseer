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

omni_info "Configuring vision build at ${build_dir}"
cmake -S "${repo_root}/vision" -B "${build_dir}"
omni_info "Building vision targets"
exec cmake --build "${build_dir}" "$@"
