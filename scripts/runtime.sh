#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source "${script_dir}/lib/log.sh"
# shellcheck disable=SC1091
source "${script_dir}/lib/common.sh"

default_image_base="ghcr.io/jburo1/omniseer-robot-runtime"
verified_tag="robot-verified"

usage() {
  cat <<'EOF'
Usage:
  scripts/omni runtime build [--image <base>] [--tag <tag>] [build args...]
  scripts/omni runtime run [--image <base>] [--tag <tag>] [container command...]
  scripts/omni runtime verify [--image <base>] [--tag <tag>] [--stage smoke|full]
  scripts/omni runtime push [--image <base>] [--tag <tag>]
  scripts/omni runtime pull [--image <base>] [--tag <tag>]

Defaults:
  --image defaults to ghcr.io/jburo1/omniseer-robot-runtime or OMNISEER_RUNTIME_IMAGE.
  build creates a runtime-<UTC>-g<shortsha> tag when --tag is omitted.
  run/verify/push use the latest local runtime build when --tag is omitted.
  pull defaults to robot-verified when --tag is omitted.
EOF
}

runtime_metadata_dir() {
  printf '%s\n' "${OMNISEER_RUNTIME_METADATA_DIR:-$(omni_repo_root)/.omniseer/runtime}"
}

runtime_image_base() {
  printf '%s\n' "${OMNISEER_RUNTIME_IMAGE:-${default_image_base}}"
}

runtime_git_sha() {
  git -C "$(omni_repo_root)" rev-parse HEAD 2>/dev/null || printf '%s\n' "unknown"
}

runtime_git_short_sha() {
  git -C "$(omni_repo_root)" rev-parse --short=12 HEAD 2>/dev/null || printf '%s\n' "unknown"
}

runtime_timestamp() {
  date -u +%Y%m%dT%H%M%SZ
}

runtime_default_tag() {
  printf 'runtime-%s-g%s\n' "$(runtime_timestamp)" "$(runtime_git_short_sha)"
}

runtime_image_ref() {
  local image_base="$1"
  local tag="$2"
  printf '%s:%s\n' "${image_base}" "${tag}"
}

runtime_safe_tag() {
  printf '%s' "$1" | tr -c '[:alnum:]_.-' '_'
}

runtime_metadata_file() {
  local kind="$1"
  local tag="$2"
  printf '%s/%s-%s.env\n' "$(runtime_metadata_dir)" "${kind}" "$(runtime_safe_tag "${tag}")"
}

runtime_latest_build_file() {
  printf '%s/latest-build.env\n' "$(runtime_metadata_dir)"
}

runtime_write_env_file() {
  local path="$1"
  shift
  mkdir -p "$(dirname "${path}")"
  : >"${path}"
  local pair key value
  for pair in "$@"; do
    key="${pair%%=*}"
    value="${pair#*=}"
    printf '%s=%q\n' "${key}" "${value}" >>"${path}"
  done
}

runtime_latest_build_tag() {
  local latest_file
  latest_file="$(runtime_latest_build_file)"
  if [[ ! -f "${latest_file}" ]]; then
    return 1
  fi
  # shellcheck disable=SC1090
  source "${latest_file}"
  [[ -n "${TAG:-}" ]] || return 1
  printf '%s\n' "${TAG}"
}

runtime_resolve_existing_tag() {
  local requested_tag="$1"
  if [[ -n "${requested_tag}" ]]; then
    printf '%s\n' "${requested_tag}"
    return 0
  fi
  runtime_latest_build_tag || omni_die "no runtime tag provided and no latest local runtime build metadata exists"
}

runtime_image_id() {
  docker image inspect --format '{{.Id}}' "$1"
}

runtime_image_digest() {
  local image_ref="$1"
  local repo_digest=""
  repo_digest="$(docker image inspect --format '{{range .RepoDigests}}{{println .}}{{end}}' "${image_ref}" 2>/dev/null | head -n 1 || true)"
  if [[ -n "${repo_digest}" ]]; then
    printf '%s\n' "${repo_digest}"
    return 0
  fi
  runtime_image_id "${image_ref}"
}

runtime_common_docker_args() {
  local image_ref="$1"
  local image_digest="$2"
  local repo_root
  repo_root="$(omni_repo_root)"
  mkdir -p "${repo_root}/runs"
  printf '%s\0' \
    "--rm" \
    "-it" \
    "--privileged" \
    "--network=host" \
    "--pid=host" \
    "--ipc=host" \
    "-v" "/dev:/dev" \
    "-v" "/run/udev:/run/udev:ro" \
    "-v" "${repo_root}/runs:/runs" \
    "-e" "OMNISEER_CONTAINER_IMAGE_REF=${image_ref}" \
    "-e" "OMNISEER_CONTAINER_IMAGE_DIGEST=${image_digest}"
}

runtime_docker_run() {
  local image_ref="$1"
  shift
  local image_digest
  image_digest="$(runtime_image_digest "${image_ref}")"
  local docker_args=()
  while IFS= read -r -d '' arg; do
    docker_args+=("${arg}")
  done < <(runtime_common_docker_args "${image_ref}" "${image_digest}")
  docker run "${docker_args[@]}" "${image_ref}" "$@"
}

runtime_parse_image_tag_args() {
  local -n _image_base_ref="$1"
  local -n _tag_ref="$2"
  shift 2
  _image_base_ref="$(runtime_image_base)"
  _tag_ref=""
  runtime_remaining_args=()
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --image)
        [[ $# -ge 2 ]] || omni_die "--image requires a value"
        _image_base_ref="$2"
        shift 2
        ;;
      --tag)
        [[ $# -ge 2 ]] || omni_die "--tag requires a value"
        _tag_ref="$2"
        shift 2
        ;;
      help|-h|--help)
        usage
        exit 0
        ;;
      --)
        shift
        runtime_remaining_args+=("$@")
        break
        ;;
      *)
        runtime_remaining_args+=("$1")
        shift
        ;;
    esac
  done
}

runtime_build() {
  local image_base tag image_ref image_id build_file latest_file timestamp git_sha
  runtime_parse_image_tag_args image_base tag "$@"
  tag="${tag:-$(runtime_default_tag)}"
  image_ref="$(runtime_image_ref "${image_base}" "${tag}")"

  "${script_dir}/build/runtime_container.sh" --image "${image_ref}" --target robot-runtime "${runtime_remaining_args[@]}"

  image_id="$(runtime_image_id "${image_ref}")"
  timestamp="$(runtime_timestamp)"
  git_sha="$(runtime_git_sha)"
  build_file="$(runtime_metadata_file build "${tag}")"
  latest_file="$(runtime_latest_build_file)"
  runtime_write_env_file \
    "${build_file}" \
    "IMAGE_BASE=${image_base}" \
    "TAG=${tag}" \
    "IMAGE_REF=${image_ref}" \
    "IMAGE_ID=${image_id}" \
    "GIT_SHA=${git_sha}" \
    "BUILT_AT=${timestamp}"
  cp "${build_file}" "${latest_file}"
  omni_info "Built runtime image ${image_ref} (${image_id})"
  omni_info "Build metadata: ${build_file}"
}

runtime_run() {
  local image_base tag image_ref
  runtime_parse_image_tag_args image_base tag "$@"
  tag="$(runtime_resolve_existing_tag "${tag}")"
  image_ref="$(runtime_image_ref "${image_base}" "${tag}")"
  runtime_docker_run "${image_ref}" "${runtime_remaining_args[@]}"
}

runtime_verify_smoke() {
  local image_base="$1"
  local tag="$2"
  local timeout_sec="${OMNISEER_RUNTIME_SAFE_SMOKE_SEC:-20}"
  local status
  set +e
  timeout "${timeout_sec}" "${BASH_SOURCE[0]}" run --image "${image_base}" --tag "${tag}" \
    run real --profile operator bringup \
    start_vision:=false \
    start_micro_ros_agent:=false \
    require_teensy:=false \
    start_lidar:=false \
    wait_for_boundary_topics:=false \
    pre_launch_cleanup:=false
  status=$?
  set -e
  if [[ "${status}" -eq 0 || "${status}" -eq 124 ]]; then
    return 0
  fi
  return "${status}"
}

runtime_verify_full() {
  local image_ref="$1"
  local run_id="$2"
  runtime_docker_run "${image_ref}" \
    run real \
    --profile operator \
    --record-run "${run_id}" \
    --record-out "/runs/${run_id}" \
    --record-experiment-config runtime-container-full \
    --record-experiment-parameter "stage=full" \
    smoke
}

runtime_write_verify_metadata() {
  local tag="$1"
  local image_base="$2"
  local image_ref="$3"
  local stage="$4"
  local run_id="$5"
  local status="$6"
  local image_id timestamp verify_file
  image_id="$(runtime_image_id "${image_ref}")"
  timestamp="$(runtime_timestamp)"
  verify_file="$(runtime_metadata_file "verify-${stage}" "${tag}")"
  runtime_write_env_file \
    "${verify_file}" \
    "IMAGE_BASE=${image_base}" \
    "TAG=${tag}" \
    "IMAGE_REF=${image_ref}" \
    "IMAGE_ID=${image_id}" \
    "STAGE=${stage}" \
    "STATUS=${status}" \
    "RUN_ID=${run_id}" \
    "RUN_DIR=$(omni_repo_root)/runs/${run_id}" \
    "VERIFIED_AT=${timestamp}"
  omni_info "Verify metadata: ${verify_file}"
}

runtime_verify() {
  local image_base tag stage image_ref run_id
  stage="smoke"
  runtime_parse_image_tag_args image_base tag "$@"
  local remaining=("${runtime_remaining_args[@]}")
  runtime_remaining_args=()
  while [[ ${#remaining[@]} -gt 0 ]]; do
    case "${remaining[0]}" in
      --stage)
        [[ ${#remaining[@]} -ge 2 ]] || omni_die "--stage requires smoke or full"
        stage="${remaining[1]}"
        remaining=("${remaining[@]:2}")
        ;;
      *)
        omni_die "unknown runtime verify argument: ${remaining[0]}"
        ;;
    esac
  done

  case "${stage}" in
    smoke|full)
      ;;
    *)
      omni_die "unsupported verify stage ${stage}; expected smoke or full"
      ;;
  esac

  tag="$(runtime_resolve_existing_tag "${tag}")"
  image_ref="$(runtime_image_ref "${image_base}" "${tag}")"
  run_id="runtime_${stage}_$(runtime_timestamp)"

  if [[ "${stage}" == "smoke" ]]; then
    runtime_verify_smoke "${image_base}" "${tag}"
  else
    runtime_verify_full "${image_ref}" "${run_id}"
  fi

  runtime_write_verify_metadata "${tag}" "${image_base}" "${image_ref}" "${stage}" "${run_id}" "passed"
}

runtime_push() {
  local image_base tag image_ref verify_file current_image_id verified_image_id promoted_ref
  runtime_parse_image_tag_args image_base tag "$@"
  [[ ${#runtime_remaining_args[@]} -eq 0 ]] || omni_die "unknown runtime push argument: ${runtime_remaining_args[0]}"
  tag="$(runtime_resolve_existing_tag "${tag}")"
  image_ref="$(runtime_image_ref "${image_base}" "${tag}")"
  verify_file="$(runtime_metadata_file verify-full "${tag}")"
  [[ -f "${verify_file}" ]] || omni_die "full verification metadata is missing for ${image_ref}; run runtime verify --stage full first"
  # shellcheck disable=SC1090
  source "${verify_file}"
  [[ "${STAGE:-}" == "full" && "${STATUS:-}" == "passed" ]] || omni_die "latest verification for ${image_ref} is not a passed full verification"
  current_image_id="$(runtime_image_id "${image_ref}")"
  verified_image_id="${IMAGE_ID:-}"
  [[ -n "${verified_image_id}" && "${current_image_id}" == "${verified_image_id}" ]] \
    || omni_die "local image ID for ${image_ref} does not match verified image ID"

  docker push "${image_ref}"
  promoted_ref="$(runtime_image_ref "${image_base}" "${verified_tag}")"
  docker tag "${image_ref}" "${promoted_ref}"
  docker push "${promoted_ref}"
  omni_info "Pushed ${image_ref} and promoted ${promoted_ref}"
}

runtime_pull() {
  local image_base tag image_ref
  runtime_parse_image_tag_args image_base tag "$@"
  [[ ${#runtime_remaining_args[@]} -eq 0 ]] || omni_die "unknown runtime pull argument: ${runtime_remaining_args[0]}"
  tag="${tag:-${verified_tag}}"
  image_ref="$(runtime_image_ref "${image_base}" "${tag}")"
  docker pull "${image_ref}"
  omni_info "Pulled ${image_ref}"
}

subcommand="${1:-help}"
if [[ $# -gt 0 ]]; then
  shift
fi

case "${subcommand}" in
  build)
    runtime_build "$@"
    ;;
  run)
    runtime_run "$@"
    ;;
  verify)
    runtime_verify "$@"
    ;;
  push)
    runtime_push "$@"
    ;;
  pull)
    runtime_pull "$@"
    ;;
  help|-h|--help)
    usage
    ;;
  *)
    omni_die "unknown runtime subcommand: ${subcommand}"
    ;;
esac
