#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this installer as root, for example with sudo." >&2
  exit 1
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
rule_src="${script_dir}/udev/99-omniseer-teensy.rules"
rule_dst="/etc/udev/rules.d/99-omniseer-teensy.rules"

if [[ ! -f "${rule_src}" ]]; then
  echo "Missing udev rule source: ${rule_src}" >&2
  exit 1
fi

install -D -m 0644 "${rule_src}" "${rule_dst}"
udevadm control --reload-rules
udevadm trigger --action=add --subsystem-match=tty

echo "Installed ${rule_dst}."
echo "Reconnect the Teensy if /dev/omniseer_teensy does not appear immediately."
