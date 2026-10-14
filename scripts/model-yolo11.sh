#!/usr/bin/env bash
# Sourced by model.sh; reuse its Docker host bind and repository path helpers.
model_yolo11_usage() {
  cat <<'HELP'
Usage:
  scripts/omni model yolo11 assets
  scripts/omni model yolo11 image [--image <name>] [docker build args...]
  scripts/omni model yolo11 export --weights <checkpoint.pt> [--output <model.onnx>] [--image <name>]
  scripts/omni model yolo11 compile --onnx <model.onnx> --precision fp|int8 [--output <model.rknn>] [--calibration-manifest <file>] [--image <name>]

Default image: omniseer/model-builder:yolo11-rknn-toolkit2-2.3.0
Outputs: artifacts/models/yolo11/<input-stem>.onnx or <input-stem>_<precision>.rknn
INT8 requires a dedicated calibration manifest with excluded validation/test images.
HELP
}

model_yolo11_main() {
  local command="${1:-help}" image="${OMNISEER_YOLO11_BUILDER_IMAGE:-omniseer/model-builder:yolo11-rknn-toolkit2-2.3.0}"
  local weights="" onnx="" output="" precision="" calibration="" path image_id
  local -a args=()
  [[ $# -eq 0 ]] || shift
  case "${command}" in
    help|-h|--help) model_yolo11_usage; return ;;
    assets)
      [[ $# -eq 0 ]] || omni_die "yolo11 assets does not accept options"
      python3 "${script_dir}/../tools/model/yolo11.py" assets --directory "$(omni_repo_root)/models/source/yolo11"
      return ;;
    image)
      case "${1:-}" in
        --image) [[ $# -ge 2 ]] || omni_die "--image requires a value"; image="$2"; shift 2 ;;
        --image=*) image="${1#--image=}"; shift ;;
        help|-h|--help) model_yolo11_usage; return ;;
      esac
      model_require_docker
      docker build --platform linux/amd64 --file "$(omni_repo_root)/docker/model-builder/yolo11/Dockerfile" \
        --tag "${image}" "$@" "$(omni_repo_root)/docker/model-builder/yolo11"
      return ;;
    export|compile) ;;
    *) omni_die "unknown yolo11 subcommand: ${command}" ;;
  esac
  while [[ $# -gt 0 ]]; do
    case "$1" in
      help|-h|--help) model_yolo11_usage; return ;;
      --weights|--onnx|--output|--precision|--calibration-manifest|--image)
        [[ $# -ge 2 && -n "$2" ]] || omni_die "$1 requires a value"
        case "$1" in
          --weights) weights="$2" ;; --onnx) onnx="$2" ;; --output) output="$2" ;;
          --precision) precision="$2" ;; --calibration-manifest) calibration="$2" ;; --image) image="$2" ;;
        esac
        shift 2 ;;
      *) omni_die "unknown yolo11 ${command} option: $1" ;;
    esac
  done
  if [[ "${command}" == export ]]; then
    [[ -n "${weights}" && -z "${onnx}${precision}${calibration}" ]] || omni_die "yolo11 export requires --weights only (plus --output/--image)"
    weights="$(model_existing_path "${weights}")"
    model_require_in_repo "${weights}"
    [[ "${weights}" == *.pt && -s "${weights}" ]] || omni_die "expected non-empty .pt checkpoint"
    output="${output:-$(omni_repo_root)/artifacts/models/yolo11/$(basename "${weights}" .pt).onnx}"
    args=(export --weights "$(model_container_path "${weights}")")
  else
    [[ -n "${onnx}" && -z "${weights}" ]] || omni_die "yolo11 compile requires --onnx"
    [[ "${precision}" == fp || "${precision}" == int8 ]] || omni_die "--precision must be fp or int8"
    onnx="$(model_existing_path "${onnx}")"
    model_require_in_repo "${onnx}"
    [[ "${onnx}" == *.onnx && -s "${onnx}" ]] || omni_die "expected non-empty .onnx model"
    output="${output:-$(omni_repo_root)/artifacts/models/yolo11/$(basename "${onnx}" .onnx)_${precision}.rknn}"
    args=(compile --onnx "$(model_container_path "${onnx}")" --precision "${precision}")
    if [[ "${precision}" == int8 ]]; then
      [[ -n "${calibration}" ]] || omni_die "INT8 requires --calibration-manifest (dedicated calibration split)"
      calibration="$(model_existing_path "${calibration}")"
      model_require_in_repo "${calibration}"
      args+=(--calibration-manifest "$(model_container_path "${calibration}")")
    else
      [[ -z "${calibration}" ]] || omni_die "FP does not use calibration"
    fi
  fi
  [[ ! -L "${output}" ]] || omni_die "refusing to overwrite existing artifact symlink: ${output}"
  output="$(model_output_path "${output}")"
  case "${output}" in
    "$(omni_repo_root)/artifacts/models/yolo11/"*) ;;
    *) omni_die "YOLO11 outputs must be under artifacts/models/yolo11/" ;;
  esac
  if [[ "${command}" == export ]]; then
    [[ "${output}" == *.onnx ]] || omni_die "export output must end in .onnx"
  else
    [[ "${output}" == *.rknn ]] || omni_die "compile output must end in .rknn"
  fi
  for path in "${output}" "${output}.json" "${output}.lock"; do
    [[ ! -e "${path}" && ! -L "${path}" ]] || omni_die "refusing to overwrite existing artifact: ${path}"
  done
  model_require_docker
  image_id="$(docker image inspect --format '{{.Id}}' "${image}")"
  model_docker_run "${image_id}" env "OMNISEER_YOLO11_IMAGE_ID=${image_id}" \
    python /workspace/tools/model/yolo11.py "${args[@]}" --output "$(model_container_path "${output}")"
}
