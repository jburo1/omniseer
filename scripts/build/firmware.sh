#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source "${script_dir}/../lib/log.sh"

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

platformio_bin="${PLATFORMIO_BIN:-}"
if [[ -z "${platformio_bin}" ]]; then
  if [[ -x "${HOME}/.platformio/penv/bin/platformio" ]]; then
    platformio_bin="${HOME}/.platformio/penv/bin/platformio"
  elif command -v platformio >/dev/null 2>&1; then
    platformio_bin="$(command -v platformio)"
  elif command -v pio >/dev/null 2>&1; then
    platformio_bin="$(command -v pio)"
  else
    omni_die "platformio not found; set PLATFORMIO_BIN or install PlatformIO"
  fi
fi

omni_info "Building firmware environment ${PIO_ENV:-teensy41}"
exec "${platformio_bin}" run -d "${script_dir}/../../firmware" -e "${PIO_ENV:-teensy41}" "$@"
