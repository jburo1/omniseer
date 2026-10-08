#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
firmware_dir="$(cd "${script_dir}/.." && pwd)"
repo_root="$(cd "${firmware_dir}/.." && pwd)"

# shellcheck disable=SC1091
source "${repo_root}/scripts/lib/common.sh"

pio_env="${PIO_ENV:-teensy41}"
pio_bin="${PIO_BIN:-$(omni_platformio_bin || true)}"
penv_python="${PLATFORMIO_PYTHON:-${HOME}/.platformio/penv/bin/python}"
teensy_ports_bin="${HOME}/.platformio/packages/tool-teensy/teensy_ports"
teensy_loader_cli_bin="${HOME}/.platformio/packages/tool-teensy/teensy_loader_cli"
firmware_hex="${firmware_dir}/.pio/build/${pio_env}/firmware.hex"
teensy_loader_wait_seconds="${TEENSY_LOADER_WAIT_SECONDS:-30}"

if [[ -f /opt/ros/kilted/setup.bash ]]; then
  # Expose the host ROS packages needed by the micro-ROS PlatformIO helper.
  set +u
  source /opt/ros/kilted/setup.bash
  set -u
fi

if [[ -f /opt/venv/bin/activate ]]; then
  set +u
  source /opt/venv/bin/activate
  set -u
fi

if [[ -z "${pio_bin}" ]]; then
  echo "PlatformIO not found; set PIO_BIN or PLATFORMIO_BIN, or install PlatformIO" >&2
  exit 2
fi

if [[ ! -x "${penv_python}" ]]; then
  echo "PlatformIO Python not found at ${penv_python}; install PlatformIO and its micro-ROS Python dependencies, or set PLATFORMIO_PYTHON" >&2
  exit 2
fi

if ! "${penv_python}" -c \
  'import catkin_pkg, colcon_common_extensions, em, importlib_resources, lark, markupsafe, pytz, yaml'; then
  echo "PlatformIO micro-ROS Python dependencies are missing." >&2
  echo "Install them with: ${penv_python} -m pip install catkin_pkg lark-parser colcon-common-extensions importlib-resources pyyaml pytz markupsafe==2.0.1 empy==3.3.4" >&2
  exit 2
fi

if [[ ! -x "${teensy_loader_cli_bin}" ]]; then
  echo "teensy_loader_cli not found at ${teensy_loader_cli_bin}." >&2
  echo "Install the project dependencies with: ${pio_bin} pkg install -d ${firmware_dir} -e ${pio_env}" >&2
  exit 2
fi

if ! command -v timeout >/dev/null 2>&1; then
  echo "timeout command not found; install coreutils so the Teensy bootloader wait is bounded" >&2
  exit 2
fi

if [[ ! "${teensy_loader_wait_seconds}" =~ ^[1-9][0-9]*$ ]]; then
  echo "TEENSY_LOADER_WAIT_SECONDS must be a positive integer, got: ${teensy_loader_wait_seconds}" >&2
  exit 2
fi

echo "Patching micro_ros_platformio kilted build helper..."
python3 "${script_dir}/patch_micro_ros_platformio.py" \
  --project-dir "${firmware_dir}" \
  --pioenv "${pio_env}"

if [[ -x "${teensy_ports_bin}" ]]; then
  echo "Detected Teensy ports before upload:"
  "${teensy_ports_bin}" -L || true
fi

rm -f "${firmware_hex}"

echo "Building ${pio_env} firmware..."
"${pio_bin}" run -d "${firmware_dir}" -e "${pio_env}"

if [[ ! -f "${firmware_hex}" ]]; then
  echo "PlatformIO completed without producing ${firmware_hex}; refusing to upload" >&2
  exit 1
fi

upload_firmware() {
  timeout --foreground --kill-after=5s "${teensy_loader_wait_seconds}s" \
    "${teensy_loader_cli_bin}" -mmcu=imxrt1062 -w -v "$@" "${firmware_hex}"
}

echo "Uploading freshly built ${pio_env} firmware with teensy_loader_cli..."
if upload_firmware -s; then
  :
else
  echo "Initial direct upload failed after entering the bootloader; retrying once without another reboot..." >&2
  if upload_firmware; then
    :
  else
    upload_status=$?
    if [[ ${upload_status} -eq 124 || ${upload_status} -eq 137 ]]; then
      echo "Timed out waiting for the Teensy bootloader after ${teensy_loader_wait_seconds}s; press the program button and rerun" >&2
    fi
    exit "${upload_status}"
  fi
fi

if [[ -x "${teensy_ports_bin}" ]]; then
  echo "Detected Teensy ports after upload:"
  "${teensy_ports_bin}" -L || true
fi

echo "Headless Teensy flash complete."
