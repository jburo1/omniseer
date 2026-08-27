#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source "${script_dir}/lib/log.sh"
# shellcheck disable=SC1091
source "${script_dir}/lib/common.sh"

readonly model_builder_default_image="omniseer/model-builder:yolo-world-v2s-rknn-toolkit2-2.1.0"
readonly model_default_variant="v2s"
readonly model_yolo_world_revision="4340b03f4f59f46279a6581bbb818e0f77765d4d"
readonly model_clip_revision="3d74acf9a28c67741b2f4f2ea7635f0aaf6f0268"
readonly model_rockchip_model_zoo_revision="bad6c7334531becaf90a561988519b7bec34d0ab"
readonly model_yolo_world_assets_url="https://huggingface.co/wondervictor/YOLO-World/resolve/${model_yolo_world_revision}"
readonly model_clip_assets_url="https://huggingface.co/openai/clip-vit-base-patch32/resolve/${model_clip_revision}"
readonly model_rockchip_assets_url="https://raw.githubusercontent.com/airockchip/rknn_model_zoo/${model_rockchip_model_zoo_revision}/examples/yolo_world/model"

# Keep this deliberately small: all supported variants use the same exporter,
# validator, RKNN settings, calibration data, and target. Only these values
# differ between the Rockchip YOLO-World v2 checkpoints.
model_select_variant() {
  local variant="${1:-${model_default_variant}}"
  case "${variant}" in
    v2s)
      model_variant="v2s"
      model_checkpoint_name="yolo_world_v2_s_obj365v1_goldg_pretrain-55b943ea.pth"
      model_config_path="configs/pretrain/yolo_world_v2_s_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_lvis_minival.py"
      model_artifact_stem="yolo_world_v2_s"
      ;;
    v2m)
      model_variant="v2m"
      model_checkpoint_name="yolo_world_v2_m_obj365v1_goldg_pretrain-c6237d5b.pth"
      model_config_path="configs/pretrain/yolo_world_v2_m_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_lvis_minival.py"
      model_artifact_stem="yolo_world_v2_m"
      ;;
    *) omni_die "unsupported YOLO-World model variant: ${variant}; expected v2s or v2m" ;;
  esac
}

model_usage() {
  cat <<'EOF'
Usage:
  scripts/omni model assets
  scripts/omni model image [--image <name>] [docker build args...]
  scripts/omni model export [--variant v2s|v2m] --weights <checkpoint.pth> [--output <model.onnx>] [--clip-model <dir>] [--image <name>]
  scripts/omni model compile [--variant v2s|v2m] --onnx <model.onnx> --precision fp|int8 [--output <model.rknn>] [--calibration-dir <dir>] [--image <name>]
  scripts/omni model build [--variant v2s|v2m] --weights <checkpoint.pth> --precision fp|int8 [--onnx-output <model.onnx>] [--output <model.rknn>] [--clip-model <dir>] [--calibration-dir <dir>] [--image <name>]

Defaults:
  image             omniseer/model-builder:yolo-world-v2s-rknn-toolkit2-2.1.0
  variant           v2s (default); supported: v2s, v2m
  clip model        models/source/clip-vit-base-patch32
  ONNX output       artifacts/models/yolo_world_v2_<s|m>.onnx
  FP RKNN output    artifacts/models/yolo_world_v2_<s|m>_fp.rknn
  INT8 RKNN output  artifacts/models/yolo_world_v2_<s|m>_i8.rknn
  calibration dir   models/source/yolo_world/calibration
EOF
}

model_image_name() {
  printf '%s\n' "${OMNISEER_MODEL_BUILDER_IMAGE:-${model_builder_default_image}}"
}

model_require_docker() {
  omni_require_command docker
}

model_existing_path() {
  local path="$1"
  [[ -e "${path}" ]] || omni_die "path does not exist: ${path}"
  realpath -e "${path}"
}

model_output_path() {
  local path="$1"
  realpath -m "${path}"
}

model_require_in_repo() {
  local path="$1"
  local repo_root
  repo_root="$(omni_repo_root)"
  case "${path}" in
    "${repo_root}"|"${repo_root}"/*) ;;
    *) omni_die "model inputs and outputs must be under ${repo_root}: ${path}" ;;
  esac
}

# Docker invoked from the devcontainer is the host Docker client. Resolve the
# host-side repository path from its bind mount instead of attempting Docker-in-
# Docker. OMNISEER_MODEL_HOST_REPO_ROOT is the explicit fallback for unusual
# mounts.
model_docker_host_repo_root() {
  local repo_root target source source_path relative
  repo_root="$(omni_repo_root)"

  if [[ -n "${OMNISEER_MODEL_HOST_REPO_ROOT:-}" ]]; then
    printf '%s\n' "${OMNISEER_MODEL_HOST_REPO_ROOT}"
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

model_container_path() {
  local local_path="$1"
  local repo_root relative
  repo_root="$(omni_repo_root)"
  relative="${local_path#"${repo_root}"}"
  printf '/workspace%s\n' "${relative}"
}

model_docker_run() {
  local image="$1"
  shift
  local host_repo
  host_repo="$(model_docker_host_repo_root)"
  docker run --rm --user "$(id -u):$(id -g)" \
    --mount "type=bind,src=${host_repo},dst=/workspace" \
    --workdir /workspace \
    "${image}" "$@"
}

model_require_checkpoint() {
  local weights="$1"
  [[ -f "${weights}" && -s "${weights}" ]] || omni_die "YOLO-World ${model_variant} checkpoint is missing or empty: ${weights}"
  [[ "$(basename "${weights}")" == "${model_checkpoint_name}" ]] \
    || omni_die "unsupported checkpoint; expected ${model_checkpoint_name}, got $(basename "${weights}")"
}

model_require_clip_model() {
  local clip_model="$1"
  [[ -d "${clip_model}" ]] || omni_die "local CLIP model directory is missing: ${clip_model}"
  [[ -s "${clip_model}/config.json" && -s "${clip_model}/pytorch_model.bin" ]] \
    || omni_die "local CLIP model must contain config.json and pytorch_model.bin: ${clip_model}"
}

model_require_calibration_embedding() {
  local embedding="$1"
  [[ -s "${embedding}" ]] \
    || omni_die "Rockchip INT8 text embedding is missing: ${embedding}"

  if ! python3 - "${embedding}" <<'PY'
import ast
import struct
import sys

path = sys.argv[1]
with open(path, "rb") as asset:
    if asset.read(6) != b"\x93NUMPY":
        raise ValueError("missing NumPy magic")
    version = tuple(asset.read(2))
    if version == (1, 0):
        header_length = struct.unpack("<H", asset.read(2))[0]
    elif version in {(2, 0), (3, 0)}:
        header_length = struct.unpack("<I", asset.read(4))[0]
    else:
        raise ValueError(f"unsupported NumPy version {version}")
    header = ast.literal_eval(asset.read(header_length).decode("latin1").strip())
    data_offset = asset.tell()

if header.get("shape") != (1, 80, 512) or header.get("descr") != "<f4":
    raise ValueError("expected a float32 array with shape (1, 80, 512)")
with open(path, "rb") as asset:
    asset.seek(0, 2)
    if asset.tell() != data_offset + 1 * 80 * 512 * 4:
        raise ValueError("unexpected array data size")
PY
  then
    omni_die "Rockchip INT8 text embedding must be a valid NumPy file with shape [1,80,512]: ${embedding}"
  fi
}

model_require_calibration() {
  local calibration_dir="$1"
  [[ -s "${calibration_dir}/dataset.txt" ]] \
    || omni_die "INT8 calibration dataset.txt is missing or empty: ${calibration_dir}/dataset.txt"
  cmp -s "${calibration_dir}/dataset.txt" <(printf 'bus.jpg coco_text_outp.npy\n') \
    || omni_die "INT8 calibration dataset.txt must contain exactly: bus.jpg coco_text_outp.npy"
  [[ -s "${calibration_dir}/bus.jpg" ]] \
    || omni_die "Rockchip INT8 calibration image is missing: ${calibration_dir}/bus.jpg"
  model_require_calibration_embedding "${calibration_dir}/coco_text_outp.npy"
}

model_download_missing() {
  local url="$1" destination="$2" part
  [[ -s "${destination}" ]] && return 0
  [[ ! -d "${destination}" ]] || omni_die "asset path is a directory: ${destination}"
  part="${destination}.part"
  [[ ! -e "${part}" ]] || omni_die "incomplete asset download already exists: ${part}"
  mkdir -p "$(dirname "${destination}")"
  if ! curl -fL --output "${part}" "${url}"; then
    rm -f "${part}"
    omni_die "failed to download model asset: ${url}"
  fi
  [[ -s "${part}" ]] || {
    rm -f "${part}"
    omni_die "downloaded model asset is empty: ${url}"
  }
  mv "${part}" "${destination}"
}

model_write_missing_calibration_dataset() {
  local dataset="$1" part
  [[ -s "${dataset}" ]] && return 0
  [[ ! -d "${dataset}" ]] || omni_die "asset path is a directory: ${dataset}"
  part="${dataset}.part"
  [[ ! -e "${part}" ]] || omni_die "incomplete asset download already exists: ${part}"
  mkdir -p "$(dirname "${dataset}")"
  printf 'bus.jpg coco_text_outp.npy\n' >"${part}"
  mv "${part}" "${dataset}"
}

model_require_assets() {
  local source_root clip_model calibration_dir checkpoint variant
  source_root="$(omni_repo_root)/models/source"
  clip_model="${source_root}/clip-vit-base-patch32"
  calibration_dir="${source_root}/yolo_world/calibration"

  for variant in v2s v2m; do
    model_select_variant "${variant}"
    checkpoint="${source_root}/yolo_world/${model_variant}/${model_checkpoint_name}"
    model_require_checkpoint "${checkpoint}"
  done
  for checkpoint in config.json pytorch_model.bin tokenizer_config.json tokenizer.json merges.txt vocab.json special_tokens_map.json preprocessor_config.json; do
    [[ -s "${clip_model}/${checkpoint}" ]] \
      || omni_die "local CLIP model asset is missing or empty: ${clip_model}/${checkpoint}"
  done
  model_require_calibration "${calibration_dir}"
}

model_assets() {
  local source_root clip_model calibration_dir checkpoint variant asset
  case "${1:-}" in
    "") ;;
    help|-h|--help) model_usage; return 0 ;;
    *) omni_die "model assets does not accept options" ;;
  esac
  omni_require_command curl
  omni_require_command python3

  source_root="$(omni_repo_root)/models/source"
  clip_model="${source_root}/clip-vit-base-patch32"
  calibration_dir="${source_root}/yolo_world/calibration"
  for variant in v2s v2m; do
    model_select_variant "${variant}"
    checkpoint="${source_root}/yolo_world/${model_variant}/${model_checkpoint_name}"
    model_download_missing "${model_yolo_world_assets_url}/${model_checkpoint_name}" "${checkpoint}"
  done
  for asset in config.json pytorch_model.bin tokenizer_config.json tokenizer.json merges.txt vocab.json special_tokens_map.json preprocessor_config.json; do
    model_download_missing "${model_clip_assets_url}/${asset}" "${clip_model}/${asset}"
  done
  model_write_missing_calibration_dataset "${calibration_dir}/dataset.txt"
  model_download_missing "${model_rockchip_assets_url}/bus.jpg" "${calibration_dir}/bus.jpg"
  model_download_missing "${model_rockchip_assets_url}/coco_text_outp.npy" "${calibration_dir}/coco_text_outp.npy"
  model_require_assets

  printf 'Model assets ready:\n'
  printf '  %s\n' \
    "${source_root}/yolo_world/v2s/yolo_world_v2_s_obj365v1_goldg_pretrain-55b943ea.pth" \
    "${source_root}/yolo_world/v2m/yolo_world_v2_m_obj365v1_goldg_pretrain-c6237d5b.pth" \
    "${clip_model}" \
    "${calibration_dir}"
}

model_image() {
  local image
  image="$(model_image_name)"
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --image)
        [[ $# -ge 2 ]] || omni_die "--image requires a value"
        image="$2"
        shift 2
        ;;
      --image=*) image="${1#--image=}"; shift ;;
      -h|--help|help) model_usage; return 0 ;;
      *) break ;;
    esac
  done
  model_require_docker
  exec docker build --file "$(omni_repo_root)/docker/model-builder/Dockerfile" --tag "${image}" "$@" "$(omni_repo_root)/docker/model-builder"
}

model_export() {
  local image output clip_model variant
  image="$(model_image_name)"
  variant="${model_default_variant}"
  local weights=""
  clip_model="$(omni_repo_root)/models/source/clip-vit-base-patch32"
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --variant) [[ $# -ge 2 ]] || omni_die "--variant requires v2s or v2m"; variant="$2"; shift 2 ;;
      --variant=*) variant="${1#--variant=}"; shift ;;
      --weights) [[ $# -ge 2 ]] || omni_die "--weights requires a path"; weights="$2"; shift 2 ;;
      --weights=*) weights="${1#--weights=}"; shift ;;
      --output) [[ $# -ge 2 ]] || omni_die "--output requires a path"; output="$2"; shift 2 ;;
      --output=*) output="${1#--output=}"; shift ;;
      --clip-model) [[ $# -ge 2 ]] || omni_die "--clip-model requires a path"; clip_model="$2"; shift 2 ;;
      --clip-model=*) clip_model="${1#--clip-model=}"; shift ;;
      --image) [[ $# -ge 2 ]] || omni_die "--image requires a value"; image="$2"; shift 2 ;;
      --image=*) image="${1#--image=}"; shift ;;
      -h|--help|help) model_usage; return 0 ;;
      *) omni_die "unknown model export option: $1" ;;
    esac
  done
  model_select_variant "${variant}"
  output="${output:-$(omni_repo_root)/artifacts/models/${model_artifact_stem}.onnx}"
  [[ -n "${weights}" ]] || omni_die "model export requires --weights"
  weights="$(model_existing_path "${weights}")"
  clip_model="$(model_existing_path "${clip_model}")"
  output="$(model_output_path "${output}")"
  model_require_in_repo "${weights}"
  model_require_in_repo "${clip_model}"
  model_require_in_repo "${output}"
  model_require_checkpoint "${weights}"
  model_require_clip_model "${clip_model}"
  [[ ! -e "${output}" ]] || omni_die "refusing to replace existing ONNX artifact: ${output}"
  model_require_docker
  mkdir -p "$(dirname "${output}")"

  local weights_in_container output_in_container clip_in_container texts_in_container config_in_container
  weights_in_container="$(model_container_path "${weights}")"
  output_in_container="$(model_container_path "${output}")"
  clip_in_container="$(model_container_path "${clip_model}")"
  texts_in_container="$(model_container_path "$(omni_repo_root)/tools/model/yolo_world_v2_s_coco_texts.json")"
  config_in_container="/opt/yolo-world/${model_config_path}"
  # shellcheck disable=SC2016 # ${1..5} expand in the container's bash, not here.
  model_docker_run "${image}" /bin/bash -euc '
    source_config="${5}"
    config=/tmp/omniseer-yolo-world-export.py
    sed -e "s|../../third_party/mmyolo|/opt/yolo-world/third_party/mmyolo|" \
        -e "s|openai/clip-vit-base-patch32|${4}|" "${source_config}" >"${config}"
    # init_detector only needs this dataset for palette metadata. Avoid making
    # the host provide the pretraining config LVIS evaluation annotations.
    printf "\\ntest_dataloader = dict(dataset=dict(_delete_=True, type=\\\"mmdet.CocoDataset\\\", ann_file=\\\"\\\", data_root=\\\"\\\", data_prefix=dict(img=\\\"\\\"), pipeline=[]))\\n" >>"${config}"
    cd /opt/yolo-world
    PYTHONPATH=./ HF_HOME=/tmp/huggingface MPLCONFIGDIR=/tmp/matplotlib \
      HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 python deploy/export_onnx.py \
      "${config}" "${1}" \
      --custom-text "${2}" --opset 12 --model-only --work-dir "$(dirname "${3}")" --device cpu
    mv "$(dirname "${3}")/$(basename "${1%.pth}").onnx" "${3}"
  ' omniseer-model-export "${weights_in_container}" "${texts_in_container}" "${output_in_container}" "${clip_in_container}" "${config_in_container}"
  model_docker_run "${image}" python /workspace/tools/model/validate_yolo_world_onnx.py --set-output-shapes "${output_in_container}"
  [[ -s "${output}" ]] || omni_die "ONNX export did not produce a non-empty artifact: ${output}"
  printf 'ONNX artifact: %s\n' "${output}"
}

model_compile() {
  local image calibration_dir variant
  image="$(model_image_name)"
  variant="${model_default_variant}"
  local onnx=""
  local precision=""
  local output=""
  calibration_dir="$(omni_repo_root)/models/source/yolo_world/calibration"
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --variant) [[ $# -ge 2 ]] || omni_die "--variant requires v2s or v2m"; variant="$2"; shift 2 ;;
      --variant=*) variant="${1#--variant=}"; shift ;;
      --onnx) [[ $# -ge 2 ]] || omni_die "--onnx requires a path"; onnx="$2"; shift 2 ;;
      --onnx=*) onnx="${1#--onnx=}"; shift ;;
      --precision) [[ $# -ge 2 ]] || omni_die "--precision requires fp or int8"; precision="$2"; shift 2 ;;
      --precision=*) precision="${1#--precision=}"; shift ;;
      --output) [[ $# -ge 2 ]] || omni_die "--output requires a path"; output="$2"; shift 2 ;;
      --output=*) output="${1#--output=}"; shift ;;
      --calibration-dir) [[ $# -ge 2 ]] || omni_die "--calibration-dir requires a path"; calibration_dir="$2"; shift 2 ;;
      --calibration-dir=*) calibration_dir="${1#--calibration-dir=}"; shift ;;
      --image) [[ $# -ge 2 ]] || omni_die "--image requires a value"; image="$2"; shift 2 ;;
      --image=*) image="${1#--image=}"; shift ;;
      -h|--help|help) model_usage; return 0 ;;
      *) omni_die "unknown model compile option: $1" ;;
    esac
  done
  model_select_variant "${variant}"
  [[ -n "${onnx}" ]] || omni_die "model compile requires --onnx"
  case "${precision}" in fp|int8) ;; *) omni_die "--precision must be fp or int8" ;; esac
  onnx="$(model_existing_path "${onnx}")"
  model_require_in_repo "${onnx}"
  if [[ -z "${output}" ]]; then
    if [[ "${precision}" == int8 ]]; then output="$(omni_repo_root)/artifacts/models/${model_artifact_stem}_i8.rknn"; else output="$(omni_repo_root)/artifacts/models/${model_artifact_stem}_fp.rknn"; fi
  fi
  output="$(model_output_path "${output}")"
  model_require_in_repo "${output}"
  [[ ! -e "${output}" ]] || omni_die "refusing to replace existing RKNN artifact: ${output}"
  if [[ "${precision}" == int8 ]]; then
    calibration_dir="$(model_existing_path "${calibration_dir}")"
    model_require_in_repo "${calibration_dir}"
    model_require_calibration "${calibration_dir}"
  fi
  model_require_docker
  mkdir -p "$(dirname "${output}")"
  local onnx_in_container output_in_container args=(python /workspace/tools/model/compile_yolo_world_rknn.py)
  onnx_in_container="$(model_container_path "${onnx}")"
  output_in_container="$(model_container_path "${output}")"
  args+=("${onnx_in_container}" "${output_in_container}" --precision)
  [[ "${precision}" == int8 ]] && args+=(i8 --dataset "$(model_container_path "${calibration_dir}")/dataset.txt") || args+=(fp)
  model_docker_run "${image}" python /workspace/tools/model/validate_yolo_world_onnx.py "${onnx_in_container}"
  model_docker_run "${image}" "${args[@]}"
  [[ -s "${output}" ]] || omni_die "RKNN compilation did not produce a non-empty artifact: ${output}"
  printf 'RKNN artifact: %s\n' "${output}"
}

model_build() {
  local weights="" precision="" image onnx_output="" output="" clip_model="" calibration_dir="" variant
  image="$(model_image_name)"
  variant="${model_default_variant}"
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --variant) [[ $# -ge 2 ]] || omni_die "--variant requires v2s or v2m"; variant="$2"; shift 2 ;;
      --variant=*) variant="${1#--variant=}"; shift ;;
      --weights) [[ $# -ge 2 ]] || omni_die "--weights requires a path"; weights="$2"; shift 2 ;;
      --weights=*) weights="${1#--weights=}"; shift ;;
      --precision) [[ $# -ge 2 ]] || omni_die "--precision requires fp or int8"; precision="$2"; shift 2 ;;
      --precision=*) precision="${1#--precision=}"; shift ;;
      --onnx-output) [[ $# -ge 2 ]] || omni_die "--onnx-output requires a path"; onnx_output="$2"; shift 2 ;;
      --onnx-output=*) onnx_output="${1#--onnx-output=}"; shift ;;
      --output) [[ $# -ge 2 ]] || omni_die "--output requires a path"; output="$2"; shift 2 ;;
      --output=*) output="${1#--output=}"; shift ;;
      --clip-model) [[ $# -ge 2 ]] || omni_die "--clip-model requires a path"; clip_model="$2"; shift 2 ;;
      --clip-model=*) clip_model="${1#--clip-model=}"; shift ;;
      --calibration-dir) [[ $# -ge 2 ]] || omni_die "--calibration-dir requires a path"; calibration_dir="$2"; shift 2 ;;
      --calibration-dir=*) calibration_dir="${1#--calibration-dir=}"; shift ;;
      --image) [[ $# -ge 2 ]] || omni_die "--image requires a value"; image="$2"; shift 2 ;;
      --image=*) image="${1#--image=}"; shift ;;
      -h|--help|help) model_usage; return 0 ;;
      *) omni_die "unknown model build option: $1" ;;
    esac
  done
  model_select_variant "${variant}"
  [[ -n "${weights}" && -n "${precision}" ]] || omni_die "model build requires --weights and --precision"
  local export_args=(--variant "${model_variant}" --weights "${weights}" --image "${image}")
  [[ -n "${onnx_output}" ]] && export_args+=(--output "${onnx_output}")
  [[ -n "${clip_model}" ]] && export_args+=(--clip-model "${clip_model}")
  model_export "${export_args[@]}"
  local compiled_onnx="${onnx_output:-$(omni_repo_root)/artifacts/models/${model_artifact_stem}.onnx}"
  local compile_args=(--variant "${model_variant}" --onnx "${compiled_onnx}" --precision "${precision}" --image "${image}")
  [[ -n "${output}" ]] && compile_args+=(--output "${output}")
  [[ -n "${calibration_dir}" ]] && compile_args+=(--calibration-dir "${calibration_dir}")
  model_compile "${compile_args[@]}"
}

model_main() {
  local subcommand="${1:-help}"
  if [[ $# -gt 0 ]]; then shift; fi
  case "${subcommand}" in
    assets) model_assets "$@" ;;
    image) model_image "$@" ;;
    export) model_export "$@" ;;
    compile) model_compile "$@" ;;
    build) model_build "$@" ;;
    help|-h|--help) model_usage ;;
    *) omni_die "unknown model subcommand: ${subcommand}" ;;
  esac
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  model_main "$@"
fi
