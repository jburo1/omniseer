#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source "${script_dir}/../lib/log.sh"
# shellcheck disable=SC1091
source "${script_dir}/../lib/common.sh"

expected_d2_version="v0.7.1"
d2_layout="dagre"
d2_theme="0"

sources=(
  "docs/diagrams/explorer/system-explorer.d2"
)

outputs=(
  "docs/assets/diagrams/explorer/system-explorer.svg"
)

usage() {
  cat <<'EOF'
Usage:
  scripts/omni docs diagrams
  scripts/omni docs diagrams --check

Renders committed D2 diagram sources into MkDocs SVG assets.

Options:
  --check   Render to a temporary directory and fail if committed SVG assets are stale.
EOF
}

resolve_d2() {
  if [[ -n "${OMNISEER_D2_BIN:-}" ]]; then
    [[ -x "${OMNISEER_D2_BIN}" ]] || omni_die "OMNISEER_D2_BIN is not executable: ${OMNISEER_D2_BIN}"
    printf '%s\n' "${OMNISEER_D2_BIN}"
    return
  fi

  omni_require_command d2
  command -v d2
}

check_d2_version() {
  local d2_bin="$1"
  local actual_version

  actual_version="$(env -u DEBUG "${d2_bin}" --version)"
  if [[ "${actual_version}" != "${expected_d2_version}" ]]; then
    omni_die "expected d2 ${expected_d2_version}, found ${actual_version}; install the pinned renderer or set OMNISEER_D2_BIN"
  fi
}

render_one() {
  local d2_bin="$1"
  local source="$2"
  local output="$3"

  [[ -f "${source}" ]] || omni_die "diagram source missing: ${source}"

  mkdir -p "$(dirname "${output}")"
  env -u DEBUG "${d2_bin}" --layout "${d2_layout}" --theme "${d2_theme}" "${source}" "${output}"
  normalize_svg_links "${output}"
  chmod 0644 "${output}"
}

normalize_svg_links() {
  local output="$1"

  python3 - "${output}" <<'PY'
from pathlib import Path
import re
import sys
from urllib.parse import urlparse


def should_target_top(href: str) -> bool:
    parsed = urlparse(href)
    return bool(href) and not href.startswith("#") and not parsed.scheme and not parsed.netloc


path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
anchor = re.compile(r'<a href="([^"]+)" xlink:href="\1">')


def add_target(match: re.Match[str]) -> str:
    href = match.group(1)
    if not should_target_top(href):
        return match.group(0)
    return f'<a href="{href}" xlink:href="{href}" target="_top">'


updated = anchor.sub(add_target, text)
if updated != text:
    path.write_text(updated, encoding="utf-8")
PY
}

render_all() {
  local d2_bin="$1"
  local output_root="${2:-}"
  local index
  local output

  for index in "${!sources[@]}"; do
    output="${outputs[${index}]}"
    if [[ -n "${output_root}" ]]; then
      output="${output_root}/${output}"
    fi
    render_one "${d2_bin}" "${sources[${index}]}" "${output}"
  done
}

check_no_orphaned_outputs() {
  local expected_output
  local published_output
  local matched

  while IFS= read -r published_output; do
    matched=0
    for expected_output in "${outputs[@]}"; do
      if [[ "${published_output}" == "${expected_output}" ]]; then
        matched=1
        break
      fi
    done

    if [[ "${matched}" -eq 0 ]]; then
      omni_die "published diagram SVG has no source mapping: ${published_output}"
    fi
  done < <(find docs/assets/diagrams -type f -name '*.svg' -print 2>/dev/null | sort)
}

check_all() {
  local d2_bin="$1"
  local tmp_dir
  local index
  local expected
  local actual

  tmp_dir="$(mktemp -d)"
  trap 'rm -rf "${tmp_dir}"' RETURN

  render_all "${d2_bin}" "${tmp_dir}"

  for index in "${!outputs[@]}"; do
    actual="${outputs[${index}]}"
    expected="${tmp_dir}/${actual}"

    [[ -f "${actual}" ]] || omni_die "published diagram SVG missing: ${actual}"
    if ! cmp -s "${expected}" "${actual}"; then
      omni_die "published diagram SVG is stale: ${actual}; run scripts/omni docs diagrams"
    fi
  done

  check_no_orphaned_outputs
  omni_info "Diagram SVG assets are current"
}

mode="render"
if [[ "${1:-}" =~ ^(-h|--help|help)$ ]]; then
  usage
  exit 0
fi

if [[ "${1:-}" == "--check" ]]; then
  mode="check"
  shift
fi

if [[ $# -gt 0 ]]; then
  omni_die "docs diagrams does not accept extra arguments: $*"
fi

repo_root="$(omni_repo_root)"
cd "${repo_root}"

d2_bin="$(resolve_d2)"
check_d2_version "${d2_bin}"

case "${mode}" in
  render)
    omni_info "Rendering D2 diagrams with d2 ${expected_d2_version}, layout ${d2_layout}, theme ${d2_theme}"
    render_all "${d2_bin}"
    ;;
  check)
    omni_info "Checking D2 diagram SVG assets"
    check_all "${d2_bin}"
    ;;
  *)
    omni_die "unknown docs diagrams mode: ${mode}"
    ;;
esac
