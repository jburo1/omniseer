#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
firmware_dir="$(cd "${script_dir}/.." && pwd)"
repo_root="$(cd "${firmware_dir}/.." && pwd)"

pio_env="${PIO_ENV:-teensy41}"
pio_bin="${PIO_BIN:-$(command -v pio || true)}"
penv_python="${PLATFORMIO_PYTHON:-${HOME}/.platformio/penv/bin/python}"
teensy_ports_bin="${HOME}/.platformio/packages/tool-teensy/teensy_ports"
teensy_loader_cli_bin="${HOME}/.platformio/packages/tool-teensy/teensy_loader_cli"
firmware_hex="${firmware_dir}/.pio/build/${pio_env}/firmware.hex"

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
  echo "pio not found in PATH" >&2
  exit 2
fi

if [[ ! -x "${penv_python}" ]]; then
  echo "PlatformIO penv python not found at ${penv_python}" >&2
  exit 2
fi

echo "Preparing PlatformIO Python environment for micro-ROS..."
"${penv_python}" -m pip install --disable-pip-version-check \
  catkin_pkg \
  lark-parser \
  colcon-common-extensions \
  importlib-resources \
  pyyaml \
  pytz \
  "markupsafe==2.0.1" \
  "empy==3.3.4"

echo "Resolving PlatformIO dependencies..."
"${pio_bin}" pkg install -d "${firmware_dir}" -e "${pio_env}"

echo "Patching micro_ros_platformio kilted build helper..."
python3 "${script_dir}/patch_micro_ros_platformio.py" \
  --project-dir "${firmware_dir}" \
  --pioenv "${pio_env}"

if [[ -x "${teensy_ports_bin}" ]]; then
  echo "Detected Teensy ports before upload:"
  "${teensy_ports_bin}" -L || true
fi

echo "Building and flashing ${pio_env} with headless teensy-cli upload..."
if ! "${pio_bin}" run -d "${firmware_dir}" -e "${pio_env}" -t upload; then
  if [[ ! -x "${teensy_loader_cli_bin}" || ! -f "${firmware_hex}" ]]; then
    echo "pio upload failed and no direct teensy_loader_cli fallback is available" >&2
    exit 1
  fi

  echo "pio upload failed; retrying direct HalfKay write with teensy_loader_cli..."
  "${teensy_loader_cli_bin}" -mmcu=imxrt1062 -v "${firmware_hex}"
fi

if [[ -x "${teensy_ports_bin}" ]]; then
  echo "Detected Teensy ports after upload:"
  "${teensy_ports_bin}" -L || true
fi

echo "Headless Teensy flash complete."
