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
  scripts/omni build firmware [platformio args...]

Builds the default Teensy 4.1 firmware environment.
EOF
}

if [[ "${1:-}" =~ ^(-h|--help|help)$ ]]; then
  usage
  exit 0
fi

if ! platformio_bin="$(omni_platformio_bin)"; then
  omni_die "platformio not found; set PLATFORMIO_BIN or install PlatformIO"
fi

omni_info "Building firmware environment ${PIO_ENV:-teensy41}"
exec "${platformio_bin}" run -d "${script_dir}/../../firmware" -e "${PIO_ENV:-teensy41}" "$@"
