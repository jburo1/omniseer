#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${1:-}" =~ ^(-h|--help|help)$ ]]; then
  cat <<'EOF'
Usage:
  scripts/omni flash teensy

Runs the existing headless Teensy flashing helper.
EOF
  exit 0
fi

exec bash "${script_dir}/../../firmware/scripts/flash_teensy_headless.sh" "$@"
