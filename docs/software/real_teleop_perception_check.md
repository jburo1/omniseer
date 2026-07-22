# Real Teleop + Native Perception Check

_Status: planned; target-hardware verification record pending_

This checklist verifies the minimum integrated real-robot slice kept as the
`legacy-teleop` profile:

- keyboard teleop reaches `/mecanum_drive_controller/reference`
- the native vision bridge publishes `/yolo/detections`
- the native vision bridge publishes `/vision/perf`
- teleop and perception run at the same time

Do not count the legacy teleop path as target-verified until one target-hardware
run has been recorded in the verification record at the end of this document.

## Preconditions

- ROS 2 workspace built and sourced.
- Real robot connected with MCU and LiDAR available as expected by
  `real.launch.py`.
- Camera available at the configured `camera.device`.
- RKNN, RGA, and V4L2 runtime dependencies installed on the target robot.

## Example Native Vision Asset Overrides

The committed `vision_bridge.real.yaml` intentionally leaves required native
vision asset paths empty. The bridge will fail fast until valid paths are
supplied.

Example using repo-shipped test assets:

```bash
REPO_ROOT=/home/radxa/apps/omniseer

DETECTOR_MODEL_PATH="${REPO_ROOT}/vision/testdata/rknn_runner/yolo_world_v2s_i8.rknn"
CLIP_MODEL_PATH="${REPO_ROOT}/vision/testdata/text_embeddings/clip_text_fp16.rknn"
CLIP_VOCAB_PATH="${REPO_ROOT}/vision/testdata/text_embeddings/clip_vocab.bpe"
CLASSES_PATH="${REPO_ROOT}/vision/testdata/text_embeddings/classes_person_bus.txt"
```

If the target robot uses different assets or a different checkout path, replace
those values with the real local paths before launch.

## Bringup Command

Launch the minimum real stack with navigation, SLAM, RF2O, and gateway disabled:

```bash
ros2 launch bringup real.launch.py \
  start_nav:=false \
  start_slam:=false \
  start_rf2o:=false \
  start_gateway:=false \
  start_vision:=true \
  wait_for_boundary_topics:=false \
  detector_model_path:="${DETECTOR_MODEL_PATH}" \
  clip_model_path:="${CLIP_MODEL_PATH}" \
  clip_vocab_path:="${CLIP_VOCAB_PATH}" \
  classes_path:="${CLASSES_PATH}"
```

Expected launch behavior:

- `vision_bridge` starts or fails immediately with a clear parameter/runtime
  error.
- `twist_mux` remains available even with nav disabled.
- No navigation stack processes are required for this check.

## Single-Entry Helper

For the supported repo-local entrypoint, use:

```bash
scripts/omni run real --profile legacy-teleop
```

Other useful modes:

- `scripts/omni run real --profile legacy-teleop smoke`
  - start the legacy teleop bringup, run the passive verifier, then stop
- `scripts/omni run real --profile legacy-teleop bringup`
  - run only the legacy teleop bringup
- `scripts/omni run teleop`
  - run only the stamped keyboard teleop publisher
- `scripts/omni check real-perception`
  - run only the passive verifier against an existing ROS graph

The legacy `scripts/phase2_real.sh` helper still delegates to the same
`legacy-teleop` implementation for compatibility.

## Teleop Command

In another terminal:

```bash
scripts/omni run teleop
```

This script publishes `geometry_msgs/msg/TwistStamped` to
`/cmd_vel_keyboard`.

## Verification Commands

Run these while the real stack and teleop are active:

```bash
ros2 topic list

ros2 topic type /cmd_vel_keyboard
ros2 topic type /mecanum_drive_controller/reference
ros2 topic echo --once /mecanum_drive_controller/reference

ros2 topic type /yolo/detections
ros2 topic echo --once /yolo/detections
ros2 topic hz /yolo/detections

ros2 topic type /vision/perf
ros2 topic echo --once /vision/perf
ros2 topic hz /vision/perf
```

Expected observations:

- `/cmd_vel_keyboard` reports `geometry_msgs/msg/TwistStamped`
- `/mecanum_drive_controller/reference` reports
  `geometry_msgs/msg/TwistStamped`
- `/mecanum_drive_controller/reference` receives teleop commands while keys are
  pressed
- `/vision/perf` publishes repeatedly
- `/yolo/detections` publishes while the camera sees configured classes

## Optional Passive Smoke Check

This helper does not publish motion commands:

```bash
scripts/omni check real-perception
```

To require at least one detection message as well:

```bash
OMNISEER_REQUIRE_DETECTIONS=1 scripts/omni check real-perception
```

## Troubleshooting

| Symptom | Likely cause | Check |
| --- | --- | --- |
| `vision_bridge` exits immediately with `camera.device` errors | Wrong V4L2 device path or camera unavailable | Check `/dev/video*`, confirm `camera.device`, test device permissions |
| `vision_bridge` exits with RKNN or RGA loader/runtime errors | Missing target-hardware acceleration libraries | Verify RKNN/RGA runtime install on the robot |
| `vision_bridge` exits with `models.* must not be empty` or `classes.path must not be empty` | Required asset overrides missing | Re-run launch with concrete asset paths |
| `/vision/perf` publishes but `/yolo/detections` stays empty | Scene does not contain configured classes, model/classes mismatch, or postprocess threshold too strict | Verify class list matches expected scene objects; temporarily lower score threshold if needed |
| `/cmd_vel_keyboard` type is `geometry_msgs/msg/Twist` | Teleop script not using stamped mode | Re-run `scripts/omni run teleop` and inspect the teleop wrapper contents |
| `/mecanum_drive_controller/reference` type does not match teleop input | Twist mux/controller path mismatch | Check `twist_mux.yaml`, confirm teleop topic type, inspect launch remap to `/mecanum_drive_controller/reference` |
| `vision_bridge` logs a fatal runtime stop after startup | Capture, preprocess, or inference pipeline failure | Read the `vision_bridge` error line for `producer` or `consumer` failure stage and errno |

## Verification Record

- Date:
- Operator:
- Robot / SBC:
- Launch command used:
- Asset paths used:
- `/cmd_vel_keyboard` type:
- `/mecanum_drive_controller/reference` type:
- `/vision/perf` observed:
- `/yolo/detections` observed:
- Notes:
