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

firmware_dir="$(cd "${script_dir}/../../firmware" && pwd)"
pio_env="${PIO_ENV:-teensy41}"

omni_info "Resolving firmware dependencies for environment ${pio_env}"
"${platformio_bin}" pkg install -d "${firmware_dir}" -e "${pio_env}"

omni_info "Patching micro_ros_platformio for ${pio_env}"
python3 "${firmware_dir}/scripts/patch_micro_ros_platformio.py" \
  --project-dir "${firmware_dir}" \
  --pioenv "${pio_env}"

omni_info "Building firmware environment ${pio_env}"
exec "${platformio_bin}" run -d "${firmware_dir}" -e "${pio_env}" "$@"
