#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source "${script_dir}/../lib/log.sh"
# shellcheck disable=SC1091
source "${script_dir}/../lib/common.sh"

if [[ "${1:-}" =~ ^(-h|--help|help)$ ]]; then
  cat <<'EOF'
Usage:
  scripts/omni docs build

Builds MkDocs with strict mode enabled.
EOF
  exit 0
fi

omni_require_command mkdocs
repo_root="$(omni_repo_root)"
cd "${repo_root}"
omni_info "Building documentation site with mkdocs --strict"
exec mkdocs build --strict "$@"
