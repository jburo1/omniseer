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
  scripts/omni run autonomy --target <class> [options] [-- launch args...]
  scripts/omni run autonomy <class> [options] [-- launch args...]

Runs the real operator profile with bounded target-centering autonomy enabled.
This command is intended to be run inside the Radxa devcontainer.

Options:
  --target <class>                 Detection class to center. Required unless provided positionally.
  --run-id <id>                    Run bundle id. Defaults to autonomy_<class>_<UTC timestamp>.
  --out <path>                     Run bundle directory. Defaults to runs/<run_id>.
  --notes <text>                   Store notes in manifest.yaml.
  --system-interval-sec <seconds>  Sample system telemetry interval. Default: 1.0.
  --evidence-interval-sec <s>      Evidence frame interval. Default: 0.25.
  --no-overwrite                   Do not replace an existing output directory.
EOF
}

sanitize_run_fragment() {
  local raw="$1"
  printf '%s\n' "${raw}" \
    | tr '[:space:]' '_' \
    | sed -e 's/[^A-Za-z0-9_.-]/_/g' -e 's/^[._-]*//' -e 's/[._-]*$//'
}

target_class=""
run_id=""
run_out=""
notes=""
system_interval_sec="1.0"
evidence_interval_sec="0.25"
overwrite="true"
extra_args=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --target)
      [[ $# -ge 2 ]] || omni_die "--target requires a class name"
      target_class="$2"
      shift 2
      ;;
    --run-id)
      [[ $# -ge 2 ]] || omni_die "--run-id requires a value"
      run_id="$2"
      shift 2
      ;;
    --out)
      [[ $# -ge 2 ]] || omni_die "--out requires a path"
      run_out="$2"
      shift 2
      ;;
    --notes)
      [[ $# -ge 2 ]] || omni_die "--notes requires text"
      notes="$2"
      shift 2
      ;;
    --system-interval-sec)
      [[ $# -ge 2 ]] || omni_die "--system-interval-sec requires a numeric argument"
      system_interval_sec="$2"
      shift 2
      ;;
    --evidence-interval-sec)
      [[ $# -ge 2 ]] || omni_die "--evidence-interval-sec requires a numeric argument"
      evidence_interval_sec="$2"
      shift 2
      ;;
    --no-overwrite)
      overwrite="false"
      shift
      ;;
    --)
      shift
      extra_args=("$@")
      break
      ;;
    help|-h|--help)
      usage
      exit 0
      ;;
    --*)
      omni_die "unknown autonomy option: $1"
      ;;
    *)
      if [[ -n "${target_class}" ]]; then
        omni_die "unexpected positional argument: $1"
      fi
      target_class="$1"
      shift
      ;;
  esac
done

target_class="$(printf '%s' "${target_class}" | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//')"
[[ -n "${target_class}" ]] || omni_die "autonomy target class is required"

if [[ -z "${run_id}" ]]; then
  target_fragment="$(sanitize_run_fragment "${target_class}")"
  [[ -n "${target_fragment}" ]] || target_fragment="target"
  run_id="autonomy_${target_fragment}_$(date -u +%Y%m%dT%H%M%SZ)"
fi

if [[ -z "${run_out}" ]]; then
  run_out="runs/${run_id}"
fi

real_args=(
  "--profile" "operator"
  "--record-run" "${run_id}"
  "--record-out" "${run_out}"
  "--record-system-interval-sec" "${system_interval_sec}"
  "--record-classes" "${target_class}"
)

if [[ "${overwrite}" == "true" ]]; then
  real_args+=("--record-overwrite")
fi

if [[ -n "${notes}" ]]; then
  real_args+=("--record-notes" "${notes}")
fi

omni_info "Starting autonomy run ${run_id} targeting class '${target_class}'"
exec "${script_dir}/real.sh" "${real_args[@]}" \
  bringup \
  "classes_path:=${run_out}/classes.txt" \
  "start_autonomy:=true" \
  "autonomy_target_class:=${target_class}" \
  "autonomy_run_dir:=${run_out}" \
  "evidence_interval_sec:=${evidence_interval_sec}" \
  "${extra_args[@]}"
