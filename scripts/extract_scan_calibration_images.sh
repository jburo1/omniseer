#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
source_video="${repo_root}/runs/scan_calibration/video/source.ts"
output_dir="${repo_root}/runs/scan_calibration/calibration images"
frame_count=100

die() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

command -v ffmpeg >/dev/null || die "ffmpeg is required"
command -v ffprobe >/dev/null || die "ffprobe is required"
[[ -f "${source_video}" ]] || die "source video is missing: ${source_video}"

duration="$(ffprobe -v error -show_entries format=duration -of default=noprint_wrappers=1:nokey=1 "${source_video}")"
[[ "${duration}" =~ ^[0-9]+([.][0-9]+)?$ ]] || die "could not determine source video duration: ${source_video}"

mkdir -p "$(dirname "${output_dir}")"
temp_dir="$(mktemp -d "${output_dir}.tmp.XXXXXX")"
cleanup() {
  rm -rf "${temp_dir}"
}
trap cleanup EXIT

for ((index = 0; index < frame_count; ++index)); do
  timestamp="$(LC_ALL=C awk -v duration="${duration}" -v frame_index="${index}" -v count="${frame_count}" \
    'BEGIN { printf "%.9f", duration * (frame_index + 0.5) / count }')"
  frame_path="${temp_dir}/frame_$(printf '%03d' "${index}").jpg"

  ffmpeg -hide_banner -loglevel error -nostdin -threads 1 -i "${source_video}" \
    -ss "${timestamp}" -map 0:v:0 -frames:v 1 -q:v 2 -bitexact "${frame_path}"
done

mapfile -t images < <(find "${temp_dir}" -maxdepth 1 -type f -name 'frame_*.jpg' -printf '%f\n' | sort)
[[ "${#images[@]}" -eq "${frame_count}" ]] || die "extraction produced ${#images[@]} images; expected ${frame_count}"

for ((index = 0; index < frame_count; ++index)); do
  frame_name="frame_$(printf '%03d' "${index}").jpg"
  [[ "${images[index]}" == "${frame_name}" ]] || die "expected ${frame_name}, found ${images[index]:-nothing}"
  ffprobe -v error -select_streams v:0 -show_entries stream=width,height \
    -of csv=p=0 "${temp_dir}/${frame_name}" >/dev/null \
    || die "unreadable extracted image: ${frame_name}"
done

rm -rf "${output_dir}"
mv "${temp_dir}" "${output_dir}"
trap - EXIT

printf 'Created %d JPEGs in %s\n' "${frame_count}" "${output_dir}"
