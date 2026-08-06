#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source "${script_dir}/lib/log.sh"
# shellcheck disable=SC1091
source "${script_dir}/lib/common.sh"

default_image_base="ghcr.io/jburo1/omniseer-robot-runtime"
verified_tag="robot-verified"
runtime_docker_extra_args=()

usage() {
  cat <<'EOF'
Usage:
  scripts/omni runtime build [--image <base>] [--tag <tag>] [build args...]
  scripts/omni runtime run [--image <base>] [--tag <tag>] [container command...]
  scripts/omni runtime record [--image <base>] [--tag <tag>] [record args...] [-- launch args...]
  scripts/omni runtime stop --run-id <id> [--time <seconds>]
  scripts/omni runtime verify [--image <base>] [--tag <tag>] [--stage smoke|full]
  scripts/omni runtime push [--image <base>] [--tag <tag>] [--release-tag <tag>]
  scripts/omni runtime pull [--image <base>] [--tag <tag>]

Defaults:
  --image defaults to ghcr.io/jburo1/omniseer-robot-runtime or OMNISEER_RUNTIME_IMAGE.
  build creates a robot-candidate-<UTC>-g<shortsha> tag when --tag is omitted.
  run/record/verify/push use the latest local runtime build when --tag is omitted.
  pull defaults to robot-verified when --tag is omitted.

Record args:
  --run-id <id>                         Default: operator_<UTC>
  --notes <text>                        Store notes in manifest.yaml.
  --classes <text>                      Store configured class names in manifest.yaml.
  --system-interval-sec <seconds>       Default: 1.0
  --experiment-config <name>            Default: operator-runtime
  --experiment-parameter <key=value>    Repeatable; default: stage=manual-operator
  --record-video                         Record inference frames for RunBundle videos on the robot.
  --record-rosbag                        Record the configured ROS topic allowlist on the robot.
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
  printf 'robot-candidate-%s-g%s\n' "$(runtime_timestamp)" "$(runtime_git_short_sha)"
}

runtime_image_ref() {
  local image_base="$1"
  local tag="$2"
  printf '%s:%s\n' "${image_base}" "${tag}"
}

runtime_verified_tag_for() {
  local git_sha="$1"
  printf 'robot-verified-g%s\n' "${git_sha}"
}

runtime_safe_tag() {
  printf '%s' "$1" | tr -c '[:alnum:]_.-' '_'
}

runtime_record_container_name() {
  printf 'omniseer-runtime-record-%s\n' "$(runtime_safe_tag "$1")"
}

runtime_write_record_classes_file() {
  local classes_text="$1"
  local classes_path="$2"
  mkdir -p "$(dirname "${classes_path}")"
  printf '%s\n' "${classes_text}" | tr ',' '\n' | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//' -e '/^$/d' >"${classes_path}"
}

runtime_launch_arg_present() {
  local name="$1"
  shift
  local arg
  for arg in "$@"; do
    if [[ "${arg}" == "${name}"* ]]; then
      return 0
    fi
  done
  return 1
}

runtime_shell_join() {
  local parts=()
  local arg
  for arg in "$@"; do
    parts+=("$(printf '%q' "${arg}")")
  done
  local IFS=" "
  printf '%s\n' "${parts[*]}"
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

runtime_registry_digest() {
  local image_base="$1"
  local image_ref="$2"
  local digest=""
  if digest="$(docker buildx imagetools inspect --format '{{.Digest}}' "${image_ref}" 2>/dev/null)" \
    && [[ "${digest}" == sha256:* ]]; then
    printf '%s@%s\n' "${image_base}" "${digest}"
    return 0
  fi
  runtime_image_digest "${image_ref}"
}

runtime_require_clean_git_tree() {
  local repo_root status
  repo_root="$(omni_repo_root)"
  status="$(git -C "${repo_root}" status --porcelain 2>/dev/null)" \
    || omni_die "unable to inspect git working tree before runtime push"
  [[ -z "${status}" ]] || omni_die "git working tree must be clean before runtime push"
}

runtime_require_current_git_sha() {
  local git_sha
  git_sha="$(git -C "$(omni_repo_root)" rev-parse HEAD 2>/dev/null)" \
    || omni_die "unable to resolve current git commit before runtime push"
  [[ -n "${git_sha}" && "${git_sha}" != "unknown" ]] \
    || omni_die "current git commit is unknown; runtime push requires an exact commit"
  printf '%s\n' "${git_sha}"
}

runtime_docker_bind_repo_root() {
  local repo_root source target relative source_path
  repo_root="$(omni_repo_root)"

  if [[ -n "${OMNISEER_RUNTIME_HOST_REPO_ROOT:-}" ]]; then
    printf '%s\n' "${OMNISEER_RUNTIME_HOST_REPO_ROOT}"
    return 0
  fi

  if omni_command_available findmnt; then
    target="$(findmnt -T "${repo_root}" -o TARGET --noheadings --raw 2>/dev/null | head -n 1 || true)"
    source="$(findmnt -T "${repo_root}" -o SOURCE --noheadings --raw 2>/dev/null | head -n 1 || true)"
    if [[ -n "${target}" && -n "${source}" && "${repo_root}" == "${target}"* && "${source}" == *"["*"]" ]]; then
      source_path="${source#*[}"
      source_path="${source_path%]}"
      relative="${repo_root#"${target}"}"
      relative="${relative#/}"
      if [[ -n "${relative}" ]]; then
        printf '%s/%s\n' "${source_path}" "${relative}"
      else
        printf '%s\n' "${source_path}"
      fi
      return 0
    fi
  fi

  printf '%s\n' "${repo_root}"
}

runtime_runs_bind_root() {
  if [[ -n "${OMNISEER_RUNTIME_RUNS_HOST_ROOT:-}" ]]; then
    printf '%s\n' "${OMNISEER_RUNTIME_RUNS_HOST_ROOT}"
    return 0
  fi
  printf '%s/runs\n' "$(runtime_docker_bind_repo_root)"
}

runtime_common_docker_args() {
  local image_ref="$1"
  local image_digest="$2"
  local repo_root runs_bind_root
  repo_root="$(omni_repo_root)"
  runs_bind_root="$(runtime_runs_bind_root)"
  mkdir -p "${repo_root}/runs"
  printf '%s\0' \
    "--rm" \
    "--sig-proxy=true" \
    "--privileged" \
    "--network=host" \
    "--pid=host" \
    "--ipc=host" \
    "-v" "/dev:/dev" \
    "-v" "/run/udev:/run/udev:ro" \
    "-v" "${runs_bind_root}:/runs" \
    "-e" "OMNISEER_CONTAINER_IMAGE_REF=${image_ref}" \
    "-e" "OMNISEER_CONTAINER_IMAGE_DIGEST=${image_digest}"
}

runtime_docker_stream_args() {
  local tty_mode="${OMNISEER_RUNTIME_DOCKER_TTY:-auto}"
  case "${tty_mode}" in
    auto)
      if [[ -t 0 && -t 1 ]]; then
        printf '%s\0' "-it"
      fi
      ;;
    always)
      printf '%s\0' "-it"
      ;;
    never)
      ;;
    *)
      omni_die "unsupported OMNISEER_RUNTIME_DOCKER_TTY=${tty_mode}; expected auto, always, or never"
      ;;
  esac
}

runtime_docker_run() {
  local image_ref="$1"
  shift
  local image_digest
  image_digest="$(runtime_image_digest "${image_ref}")"
  local docker_args=()
  while IFS= read -r -d '' arg; do
    docker_args+=("${arg}")
  done < <(runtime_docker_stream_args)
  while IFS= read -r -d '' arg; do
    docker_args+=("${arg}")
  done < <(runtime_common_docker_args "${image_ref}" "${image_digest}")
  docker_args+=("${runtime_docker_extra_args[@]}")
  if [[ $# -gt 0 ]]; then
    docker_args+=("-e" "OMNISEER_RUNTIME_CONTAINER_COMMAND=$(runtime_shell_join "$@")")
  fi
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

runtime_record() {
  local image_base tag image_ref run_id system_interval_sec experiment_config status record_video record_rosbag
  local repo_root runs_bind_root host_run_dir
  local experiment_parameters=()
  local launch_args=()
  local notes=""
  local classes=""
  run_id="operator_$(runtime_timestamp)"
  system_interval_sec="1.0"
  experiment_config="operator-runtime"
  record_video="false"
  record_rosbag="false"
  experiment_parameters+=("stage=manual-operator")

  runtime_parse_image_tag_args image_base tag "$@"
  local remaining=("${runtime_remaining_args[@]}")
  runtime_remaining_args=()
  while [[ ${#remaining[@]} -gt 0 ]]; do
    case "${remaining[0]}" in
      --run-id|--record-run)
        [[ ${#remaining[@]} -ge 2 ]] || omni_die "${remaining[0]} requires a value"
        run_id="${remaining[1]}"
        remaining=("${remaining[@]:2}")
        ;;
      --system-interval-sec|--record-system-interval-sec)
        [[ ${#remaining[@]} -ge 2 ]] || omni_die "${remaining[0]} requires a value"
        system_interval_sec="${remaining[1]}"
        remaining=("${remaining[@]:2}")
        ;;
      --notes|--record-notes)
        [[ ${#remaining[@]} -ge 2 ]] || omni_die "${remaining[0]} requires a value"
        notes="${remaining[1]}"
        remaining=("${remaining[@]:2}")
        ;;
      --classes|--record-classes)
        [[ ${#remaining[@]} -ge 2 ]] || omni_die "${remaining[0]} requires a value"
        classes="${remaining[1]}"
        remaining=("${remaining[@]:2}")
        ;;
      --experiment-config|--record-experiment-config)
        [[ ${#remaining[@]} -ge 2 ]] || omni_die "${remaining[0]} requires a value"
        experiment_config="${remaining[1]}"
        remaining=("${remaining[@]:2}")
        ;;
      --experiment-parameter|--record-experiment-parameter)
        [[ ${#remaining[@]} -ge 2 ]] || omni_die "${remaining[0]} requires a value"
        experiment_parameters+=("${remaining[1]}")
        remaining=("${remaining[@]:2}")
        ;;
      --record-video)
        record_video="true"
        remaining=("${remaining[@]:1}")
        ;;
      --record-rosbag)
        record_rosbag="true"
        remaining=("${remaining[@]:1}")
        ;;
      --)
        launch_args=("${remaining[@]:1}")
        remaining=()
        ;;
      help|-h|--help)
        usage
        exit 0
        ;;
      --*)
        omni_die "unknown runtime record argument: ${remaining[0]}"
        ;;
      *)
        launch_args=("${remaining[@]}")
        remaining=()
        ;;
    esac
  done

  tag="$(runtime_resolve_existing_tag "${tag}")"
  image_ref="$(runtime_image_ref "${image_base}" "${tag}")"
  repo_root="$(omni_repo_root)"
  runs_bind_root="$(runtime_runs_bind_root)"
  host_run_dir="${runs_bind_root}/${run_id}"

  local command=(
    run real
    --profile operator
    --record-run "${run_id}"
    --record-out "/runs/${run_id}"
    --record-overwrite
    --record-system-interval-sec "${system_interval_sec}"
    --record-experiment-config "${experiment_config}"
  )
  if [[ -n "${notes}" ]]; then
    command+=(--record-notes "${notes}")
  fi
  if [[ -n "${classes}" ]]; then
    command+=(--record-classes "${classes}")
  fi
  if [[ "${record_video}" == "true" ]]; then
    command+=(--record-video)
  fi
  if [[ "${record_rosbag}" == "true" ]]; then
    command+=(--record-rosbag)
  fi
  local parameter
  for parameter in "${experiment_parameters[@]}"; do
    command+=(--record-experiment-parameter "${parameter}")
  done

  if [[ -n "${classes}" ]]; then
    runtime_write_record_classes_file "${classes}" "${host_run_dir}/classes.txt"
    if ! runtime_launch_arg_present "classes_path:=" "${launch_args[@]}"; then
      launch_args=("classes_path:=/runs/${run_id}/classes.txt" "${launch_args[@]}")
    fi
  fi

  command+=(bringup)
  command+=("${launch_args[@]}")

  omni_info "Recording runtime operator run ${run_id}"
  omni_info "Run bundle path: ${host_run_dir}"
  if [[ "${runs_bind_root}" != "${repo_root}/runs" ]]; then
    omni_info "Docker host run bind: ${runs_bind_root}/${run_id}"
  fi
  runtime_docker_extra_args=(
    "--name" "$(runtime_record_container_name "${run_id}")"
    "--label" "org.omniseer.kind=runtime-record"
    "--label" "org.omniseer.run_id=${run_id}"
  )
  set +e
  runtime_docker_run "${image_ref}" "${command[@]}"
  status=$?
  runtime_docker_extra_args=()
  set -e
  omni_info "Run bundle path: ${host_run_dir}"
  return "${status}"
}

runtime_stop() {
  local run_id="" timeout_sec="20"
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --run-id|--record-run)
        [[ $# -ge 2 ]] || omni_die "$1 requires a value"
        run_id="$2"
        shift 2
        ;;
      --time|--timeout)
        [[ $# -ge 2 ]] || omni_die "$1 requires a value"
        timeout_sec="$2"
        shift 2
        ;;
      help|-h|--help)
        usage
        exit 0
        ;;
      *)
        omni_die "unknown runtime stop argument: $1"
        ;;
    esac
  done
  [[ -n "${run_id}" ]] || omni_die "runtime stop requires --run-id"

  local container_name
  container_name="$(runtime_record_container_name "${run_id}")"
  if ! docker container inspect "${container_name}" >/dev/null 2>&1; then
    omni_warn "runtime record container is not present: ${container_name}"
    return 0
  fi

  docker stop --time "${timeout_sec}" "${container_name}" >/dev/null
  omni_info "Stopped runtime record container ${container_name}"
}

runtime_verify_smoke() {
  local image_base="$1"
  local tag="$2"
  local timeout_sec="${OMNISEER_RUNTIME_SAFE_SMOKE_SEC:-20}"
  local status
  set +e
  OMNISEER_RUNTIME_DOCKER_TTY=never timeout "${timeout_sec}" "${BASH_SOURCE[0]}" run --image "${image_base}" --tag "${tag}" \
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
  OMNISEER_RUNTIME_DOCKER_TTY=never runtime_docker_run "${image_ref}" \
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
  local git_sha image_id timestamp verify_file
  git_sha="$(runtime_git_sha)"
  image_id="$(runtime_image_id "${image_ref}")"
  timestamp="$(runtime_timestamp)"
  verify_file="$(runtime_metadata_file "verify-${stage}" "${tag}")"
  runtime_write_env_file \
    "${verify_file}" \
    "IMAGE_BASE=${image_base}" \
    "TAG=${tag}" \
    "IMAGE_REF=${image_ref}" \
    "IMAGE_ID=${image_id}" \
    "GIT_SHA=${git_sha}" \
    "STAGE=${stage}" \
    "STATUS=${status}" \
    "RUN_ID=${run_id}" \
    "RUN_DIR=$(omni_repo_root)/runs/${run_id}" \
    "VERIFIED_AT=${timestamp}"
  omni_info "Verify metadata: ${verify_file}"
}

runtime_write_release_metadata() {
  local source_tag="$1"
  local image_base="$2"
  local image_ref="$3"
  local image_id="$4"
  local git_sha="$5"
  local registry_digest="$6"
  local verify_file="$7"
  shift 7
  local release_file timestamp pushed_tags
  timestamp="$(runtime_timestamp)"
  pushed_tags="$(runtime_shell_join "$@")"
  release_file="$(runtime_metadata_file release "${source_tag}")"
  runtime_write_env_file \
    "${release_file}" \
    "IMAGE_BASE=${image_base}" \
    "SOURCE_TAG=${source_tag}" \
    "SOURCE_IMAGE_REF=${image_ref}" \
    "IMAGE_ID=${image_id}" \
    "GIT_SHA=${git_sha}" \
    "REGISTRY_DIGEST=${registry_digest}" \
    "PUSHED_TAGS=${pushed_tags}" \
    "VERIFY_FILE=${verify_file}" \
    "VERIFY_STAGE=full" \
    "PUSHED_AT=${timestamp}"
  omni_info "Release metadata: ${release_file}"
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
  local image_base tag image_ref verify_file current_image_id verified_image_id verified_git_sha current_git_sha
  local immutable_verified_ref moving_verified_ref registry_digest release_metadata_tag
  local immutable_verified_tag release_tag="" release_ref=""
  local pushed_tags=()
  runtime_parse_image_tag_args image_base tag "$@"
  local remaining=("${runtime_remaining_args[@]}")
  runtime_remaining_args=()
  while [[ ${#remaining[@]} -gt 0 ]]; do
    case "${remaining[0]}" in
      --release-tag)
        [[ ${#remaining[@]} -ge 2 ]] || omni_die "--release-tag requires a value"
        release_tag="${remaining[1]}"
        remaining=("${remaining[@]:2}")
        ;;
      *)
        omni_die "unknown runtime push argument: ${remaining[0]}"
        ;;
    esac
  done

  runtime_require_clean_git_tree
  current_git_sha="$(runtime_require_current_git_sha)"
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
  verified_git_sha="${GIT_SHA:-}"
  [[ -n "${verified_git_sha}" && "${verified_git_sha}" == "${current_git_sha}" ]] \
    || omni_die "verification commit does not match current git commit"

  immutable_verified_tag="$(runtime_verified_tag_for "${current_git_sha}")"
  immutable_verified_ref="$(runtime_image_ref "${image_base}" "${immutable_verified_tag}")"
  moving_verified_ref="$(runtime_image_ref "${image_base}" "${verified_tag}")"
  docker tag "${image_ref}" "${immutable_verified_ref}"
  docker push "${immutable_verified_ref}"
  pushed_tags+=("${immutable_verified_tag}")
  if [[ -n "${release_tag}" ]]; then
    release_ref="$(runtime_image_ref "${image_base}" "${release_tag}")"
    docker tag "${image_ref}" "${release_ref}"
    docker push "${release_ref}"
    pushed_tags+=("${release_tag}")
  fi
  docker tag "${image_ref}" "${moving_verified_ref}"
  docker push "${moving_verified_ref}"
  pushed_tags+=("${verified_tag}")

  registry_digest="$(runtime_registry_digest "${image_base}" "${immutable_verified_ref}")"
  release_metadata_tag="${immutable_verified_tag}"
  runtime_write_release_metadata \
    "${release_metadata_tag}" \
    "${image_base}" \
    "${image_ref}" \
    "${current_image_id}" \
    "${current_git_sha}" \
    "${registry_digest}" \
    "${verify_file}" \
    "${pushed_tags[@]}"

  omni_info "Promoted immutable verified image ${immutable_verified_ref}"
  if [[ -n "${release_ref}" ]]; then
    omni_info "Promoted release image ${release_ref}"
  fi
  omni_info "Promoted moving verified image ${moving_verified_ref}"
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
  record)
    runtime_record "$@"
    ;;
  stop)
    runtime_stop "$@"
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
