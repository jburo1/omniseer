#!/usr/bin/env bash
set -euo pipefail

timeout_seconds="${OMNISEER_TOPIC_TIMEOUT_SECONDS:-15}"
require_detections="${OMNISEER_REQUIRE_DETECTIONS:-0}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not found in PATH" >&2
  exit 2
fi

wait_for_topic_type() {
  local topic="$1"
  local expected_type="$2"
  local deadline=$((SECONDS + timeout_seconds))
  local observed_type=""

  while (( SECONDS < deadline )); do
    observed_type="$(ros2 topic type "${topic}" 2>/dev/null || true)"
    if [[ "${observed_type}" == "${expected_type}" ]]; then
      echo "ok: ${topic} type is ${expected_type}"
      return 0
    fi
    sleep 1
  done

  echo "missing or wrong type: ${topic} expected ${expected_type} observed ${observed_type:-<none>}" >&2
  return 1
}

wait_for_message() {
  local topic="$1"
  local deadline=$((SECONDS + timeout_seconds))

  while (( SECONDS < deadline )); do
    if ros2 topic echo --once "${topic}" >/dev/null 2>&1; then
      echo "ok: received one message on ${topic}"
      return 0
    fi
    sleep 1
  done

  echo "no message received on ${topic} within ${timeout_seconds}s" >&2
  return 1
}

wait_for_topic_type "/mecanum_drive_controller/reference" "geometry_msgs/msg/TwistStamped"
wait_for_topic_type "/yolo/detections" "yolo_msgs/msg/DetectionArray"
wait_for_topic_type "/vision/perf" "omniseer_msgs/msg/VisionPerfSummary"

wait_for_message "/vision/perf"

if wait_for_message "/yolo/detections"; then
  :
elif [[ "${require_detections}" == "1" ]]; then
  exit 1
else
  echo "warning: no /yolo/detections message observed; rerun with a visible target or set OMNISEER_REQUIRE_DETECTIONS=1 to make this fatal" >&2
fi
