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
  scripts/omni test vision

Builds and runs the portable native vision tests documented in CI.
EOF
}

if [[ "${1:-}" =~ ^(-h|--help|help)$ ]]; then
  usage
  exit 0
fi

omni_require_command cmake
omni_require_command ctest

repo_root="$(omni_repo_root)"
build_dir="${OMNISEER_VISION_VERIFY_BUILD_DIR:-${repo_root}/vision/build-verify}"

omni_info "Configuring portable vision verification build at ${build_dir}"
cmake -S "${repo_root}/vision" -B "${build_dir}" -DVISION_BUILD_HARNESS=OFF

omni_info "Building portable vision verification targets"
cmake --build "${build_dir}" \
  --target image_buffer_pool_test jsonl_telemetry_test rolling_telemetry_test

omni_info "Running portable vision verification tests"
exec ctest --test-dir "${build_dir}" \
  -R 'image_buffer_pool_test|jsonl_telemetry_test|rolling_telemetry_test' \
  --output-on-failure
