#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
timeout_seconds="${OMNISEER_TOPIC_TIMEOUT_SECONDS:-15}"
gateway_host="${OMNISEER_GATEWAY_HOST:-127.0.0.1}"
gateway_port="${OMNISEER_GATEWAY_PORT:-50051}"
preview_profile="${OMNISEER_PREVIEW_PROFILE:-balanced}"
preview_host="${OMNISEER_PREVIEW_HOST:-${gateway_host}}"
preview_port="${OMNISEER_PREVIEW_PORT:-7100}"
preview_latency_ms="${OMNISEER_PREVIEW_LATENCY_MS:-125}"
gst_launch_path="${OMNISEER_GST_LAUNCH_PATH:-gst-launch-1.0}"
overlay_smoke_seconds="${OMNISEER_OVERLAY_SMOKE_SECONDS:-2}"
overlay_timeout_seconds="${OMNISEER_OVERLAY_TIMEOUT_SECONDS:-20}"
teleop_enabled=0
preview_enabled=0
reference_output=""

gateway_cli() {
  ros2 run robot_diag_control robot_gateway_cli \
    --host "${gateway_host}" --port "${gateway_port}" "$@"
}

cleanup() {
  local exit_code=$?
  set +e
  if [[ "${teleop_enabled}" == "1" ]]; then
    gateway_cli teleop off >/dev/null 2>&1
  fi
  if [[ "${preview_enabled}" == "1" ]]; then
    gateway_cli preview off >/dev/null 2>&1
  fi
  if [[ -n "${reference_output}" ]]; then
    rm -f "${reference_output}"
  fi
  exit "${exit_code}"
}
trap cleanup EXIT INT TERM

require_output() {
  local output="$1"
  local expected="$2"
  local label="$3"
  if ! grep -Fq -- "${expected}" <<<"${output}"; then
    echo "failed: ${label}; expected '${expected}'" >&2
    echo "${output}" >&2
    return 1
  fi
}

wait_for_message() {
  local topic="$1"
  shift
  if timeout "${timeout_seconds}" ros2 topic echo --once "$@" "${topic}" >/dev/null 2>&1; then
    echo "ok: received one message on ${topic}"
    return 0
  fi
  echo "no message received on ${topic} within ${timeout_seconds}s" >&2
  return 1
}

"${script_dir}/real_teleop_perception.sh"
wait_for_message "/encoder_counts"
wait_for_message "/mecanum_drive_controller/odometry"
wait_for_message "/scan" --qos-reliability best_effort

status_output="$(gateway_cli status)"
echo "${status_output}"
require_output "${status_output}" "health: state=ok ready=true" "gateway health is not ready"
require_output "${status_output}" "mobility: odom=fresh" "gateway odometry is not fresh"
require_output "${status_output}" "infer_errors=0" "vision reports inference errors"
require_output "${status_output}" "capture_fatal_errors=0" "vision reports capture errors"
if grep -Fq -- "vision: unavailable" <<<"${status_output}" || \
  grep -Eq -- '^vision:.* stale($| )' <<<"${status_output}"; then
  echo "failed: gateway vision status is unavailable or stale" >&2
  exit 1
fi

preview_output="$(gateway_cli preview on --profile "${preview_profile}")"
echo "${preview_output}"
require_output "${preview_output}" "accepted=True" "preview request was rejected"
require_output "${preview_output}" "state=running" "preview did not start"
preview_enabled=1

# The manager initially reports running after its startup window. Poll once more
# so an asynchronous camera, encoder, or SRT bind failure becomes fatal.
sleep 1
status_output="$(gateway_cli status)"
echo "${status_output}"
require_output "${status_output}" "preview: state=running" "preview worker exited after startup"
if grep -Fq -- "preview_error:" <<<"${status_output}"; then
  echo "failed: preview worker reported an error" >&2
  exit 1
fi

if [[ "${OMNISEER_SKIP_OVERLAY_SMOKE:-0}" != "1" ]]; then
  timeout "${overlay_timeout_seconds}" ros2 run robot_diag_control robot_overlay_viewer \
    --host "${gateway_host}" \
    --port "${gateway_port}" \
    --preview-host "${preview_host}" \
    --preview-port "${preview_port}" \
    --preview-latency-ms "${preview_latency_ms}" \
    --gst-launch-path "${gst_launch_path}" \
    --profile "${preview_profile}" \
    --mode fakesink \
    --duration-seconds "${overlay_smoke_seconds}" \
    --leave-preview-running
  echo "ok: overlay viewer consumed preview for ${overlay_smoke_seconds}s"
else
  echo "skipped: overlay viewer smoke disabled by OMNISEER_SKIP_OVERLAY_SMOKE=1"
fi

preview_output="$(gateway_cli preview off)"
echo "${preview_output}"
require_output "${preview_output}" "accepted=True" "preview stop was rejected"
require_output "${preview_output}" "state=disabled" "preview did not stop"
preview_enabled=0

teleop_output="$(gateway_cli teleop on)"
echo "${teleop_output}"
require_output "${teleop_output}" "accepted=True" "teleop enable was rejected"
require_output "${teleop_output}" "enabled=true" "teleop did not enable"
teleop_enabled=1

reference_output="$(mktemp /tmp/omniseer-operator-reference.XXXXXX)"
timeout "${timeout_seconds}" ros2 topic echo --once --qos-reliability best_effort \
  /mecanum_drive_controller/reference >"${reference_output}" 2>&1 &
reference_pid=$!
deadline=$((SECONDS + timeout_seconds))
teleop_output=""
teleop_command_observed=0
while kill -0 "${reference_pid}" 2>/dev/null && (( SECONDS < deadline )); do
  teleop_output="$(gateway_cli teleop stop)"
  if [[ "${teleop_command_observed}" == "0" ]]; then
    echo "${teleop_output}"
    require_output "${teleop_output}" "accepted=True" "zero teleop command was rejected"
    teleop_command_observed=1
  fi
  sleep 0.2
done
if ! wait "${reference_pid}"; then
  echo "failed: no teleop output reached /mecanum_drive_controller/reference" >&2
  cat "${reference_output}" >&2
  exit 1
fi
echo "ok: zero teleop command reached /mecanum_drive_controller/reference"

teleop_output="$(gateway_cli teleop off)"
echo "${teleop_output}"
require_output "${teleop_output}" "accepted=True" "teleop disable was rejected"
require_output "${teleop_output}" "enabled=false" "teleop did not disable"
teleop_enabled=0

echo "Operator real profile acceptance checks passed"
