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
  scripts/omni build [all]
  scripts/omni build strict
  scripts/omni build all [--strict]

Builds all available product artifacts. ROS is required. Native vision and
firmware are built when their toolchains are available, or required in strict mode.
EOF
}

strict=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --strict)
      strict=true
      shift
      ;;
    help|-h|--help)
      usage
      exit 0
      ;;
    *)
      omni_die "unknown build all argument: $1"
      ;;
  esac
done

repo_root="$(omni_repo_root)"

if [[ "${strict}" == "true" ]]; then
  omni_command_available cmake || omni_die "cmake not found; cannot build native vision workspace in strict mode"
  omni_platformio_bin >/dev/null || omni_die "platformio not found; cannot build firmware in strict mode"
fi

omni_info "Building ROS workspace"
"${script_dir}/ros.sh"

if omni_command_available cmake; then
  omni_info "Building native vision workspace"
  "${script_dir}/vision.sh"
elif [[ "${strict}" == "true" ]]; then
  omni_die "cmake not found; cannot build native vision workspace in strict mode"
else
  omni_warn "cmake not found; skipping native vision workspace"
fi

if omni_platformio_bin >/dev/null; then
  omni_info "Building firmware"
  "${script_dir}/firmware.sh"
elif [[ "${strict}" == "true" ]]; then
  omni_die "platformio not found; cannot build firmware in strict mode"
else
  omni_warn "platformio not found; skipping firmware"
fi

omni_info "Product build complete at ${repo_root}"
