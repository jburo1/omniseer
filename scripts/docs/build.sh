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

Checks diagram assets, builds MkDocs with strict mode enabled, and validates built
diagram links.
EOF
  exit 0
fi

omni_require_command mkdocs
repo_root="$(omni_repo_root)"
cd "${repo_root}"
omni_info "Checking diagram SVG assets before docs build"
"${script_dir}/diagrams.sh" --check

omni_info "Building documentation site with mkdocs --strict --clean"
mkdocs build --strict --clean "$@"

omni_info "Checking built SVG diagram links"
"${script_dir}/check_diagram_links.py" --site-dir site
