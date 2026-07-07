# Phase 0.5 Real Test Runbook

This runbook captures the recommended sequence for testing the Phase 0.5 real
teleop + native perception path on the target robot.

## Goal

Verify that all of the following work at the same time on real hardware:

- keyboard teleop publishes stamped commands
- `twist_mux` forwards to `/mecanum_drive_controller/reference`
- the microcontroller is connected through micro-ROS
- the native vision bridge publishes `/vision/perf`
- the native vision bridge publishes `/yolo/detections`

## Environment Setup

Enter the Docker or devcontainer environment you normally use for ROS on the
robot SBC, then source ROS and the built workspace.

Typical sequence:

```bash
source /opt/ros/kilted/setup.bash
source /opt/venv/bin/activate
source /ros_ws/install/setup.bash
```

If your built workspace is in a different path, source the correct
`install/setup.bash` for that environment.

## Required Device Checks

Before launching anything, verify that the serial and camera devices are visible
inside the container or dev environment:

```bash
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
ls -l /dev/video*
v4l2-ctl --list-devices
```

Repo defaults:

- micro-ROS serial device: `/dev/ttyACM0`
- camera device: `/dev/video11`

If your hardware enumerates differently, override those launch arguments.

## MCU Bringup Expectations

The real bringup path already includes the real IO layer. Running
`real.launch.py` starts:

- `micro_ros_agent`
- LiDAR driver
- encoder-to-odometry adapter
- optional native vision bridge

You do not need to start `micro_ros_agent` separately unless you are debugging
the serial link.

The firmware in this repo is expected to:

- subscribe to `/mecanum_drive_controller/reference`
- publish `/encoder_counts`
- publish `/imu`
- publish `/range`
- publish `/battery`

These expectations come from `firmware/include/micro_ros_config.hpp`.

## MCU Correctness Checks

Check these four things before blaming launch or teleop.

### 1. Correct Serial Device

Identify the actual board path:

```bash
ls -l /dev/serial/by-id
ls /dev/ttyACM*
```

If the MCU is not at `/dev/ttyACM0`, override it when launching:

```bash
micro_ros_serial_device:=/dev/ttyACM1
```

### 2. Correct ROS Domain

The firmware is compiled with ROS domain ID `42`.

Check your current shell:

```bash
echo "${ROS_DOMAIN_ID:-unset}"
```

If needed:

```bash
export ROS_DOMAIN_ID=42
```

### 3. Correct Firmware Topic Contract

The MCU must still use the topic contract compiled into the firmware:

- command input: `/mecanum_drive_controller/reference`
- IMU: `/imu`
- range: `/range`
- battery: `/battery`
- encoders: `/encoder_counts`

If the flashed MCU firmware is older or built from a different branch, this
test may fail even if the ROS side is correct.

### 4. MCU Topics Actually Appear

After launch, confirm the real IO topics exist:

```bash
ros2 topic list | egrep '^/encoder_counts$|^/imu$|^/range$|^/battery$|^/mecanum_drive_controller/odometry$'
```

If these do not appear, the serial agent-to-MCU link is not working.

## Vision Asset Overrides

The committed real vision config intentionally leaves model and class-list paths
empty. For repeatable runs, you can place the required paths in a reusable YAML
file and point the launch at it.

A repo-local example is available at:

```bash
ros_ws/src/bringup/config/vision_bridge.real.paths.yaml
```

That file already contains the repo-shipped test asset paths for the real vision
stack. If your target robot uses different paths, edit that file in place.

You can also use shell variables if you prefer a one-off launch:

```bash
REPO_ROOT=/home/radxa/apps/omniseer

DETECTOR_MODEL_PATH="${REPO_ROOT}/vision/testdata/rknn_runner/yolo_world_v2s_i8.rknn"
CLIP_MODEL_PATH="${REPO_ROOT}/vision/testdata/text_embeddings/clip_text_fp16.rknn"
CLIP_VOCAB_PATH="${REPO_ROOT}/vision/testdata/text_embeddings/clip_vocab.bpe"
CLASSES_PATH="${REPO_ROOT}/vision/testdata/text_embeddings/classes_person_bus.txt"
```

If the target robot uses different paths, replace them with the real local
values.

## Real Bringup Command

Launch the minimum real stack with navigation, SLAM, RF2O, and gateway
disabled:

```bash
ros2 launch bringup real.launch.py \
  start_nav:=false \
  start_slam:=false \
  start_rf2o:=false \
  start_gateway:=false \
  start_vision:=true \
  wait_for_boundary_topics:=false \
  vision_params_file:=vision_bridge.real.paths.yaml \
  detector_model_path:=__from_config__ \
  clip_model_path:=__from_config__ \
  clip_vocab_path:=__from_config__ \
  classes_path:=__from_config__
```

If you want to override the camera or serial device explicitly, append them:

```bash
ros2 launch bringup real.launch.py \
  start_nav:=false \
  start_slam:=false \
  start_rf2o:=false \
  start_gateway:=false \
  start_vision:=true \
  wait_for_boundary_topics:=false \
  vision_params_file:=vision_bridge.real.paths.yaml \
  micro_ros_serial_device:=/dev/ttyACM0 \
  camera_device:=/dev/video11 \
  detector_model_path:=__from_config__ \
  clip_model_path:=__from_config__ \
  clip_vocab_path:=__from_config__ \
  classes_path:=__from_config__
```

If needed, also override hardware paths:

```bash
ros2 launch bringup real.launch.py \
  start_nav:=false \
  start_slam:=false \
  start_rf2o:=false \
  start_gateway:=false \
  start_vision:=true \
  wait_for_boundary_topics:=false \
  micro_ros_serial_device:=/dev/ttyACM0 \
  camera_device:=/dev/video11 \
  detector_model_path:="${DETECTOR_MODEL_PATH}" \
  clip_model_path:="${CLIP_MODEL_PATH}" \
  clip_vocab_path:="${CLIP_VOCAB_PATH}" \
  classes_path:="${CLASSES_PATH}"
```

Expected launch behavior:

- `vision_bridge` starts or fails immediately with a clear parameter or runtime
  error
- `twist_mux` remains available with navigation disabled
- no nav stack is required for this test

## Teleop

Open another shell in the same environment and source the same setup.

Then run:

```bash
bash ros_ws/src/scripts/teleop.sh
```

This script now publishes `geometry_msgs/msg/TwistStamped` on
`/cmd_vel_keyboard`.

## Verify the Teleop and MCU Path

Run these commands while bringup and teleop are active:

```bash
ros2 topic type /cmd_vel_keyboard
ros2 topic type /mecanum_drive_controller/reference
ros2 topic echo --once /mecanum_drive_controller/reference
ros2 topic echo --once /encoder_counts
ros2 topic echo --once /imu
ros2 topic echo --once /range
```

Expected:

- `/cmd_vel_keyboard` is `geometry_msgs/msg/TwistStamped`
- `/mecanum_drive_controller/reference` is `geometry_msgs/msg/TwistStamped`
- `/mecanum_drive_controller/reference` receives messages while keys are pressed
- MCU sensor topics publish at least one message

## Verify the Vision Path

Run:

```bash
ros2 topic type /vision/perf
ros2 topic echo --once /vision/perf
ros2 topic type /yolo/detections
ros2 topic echo --once /yolo/detections
```

You can also inspect publish rates:

```bash
ros2 topic hz /vision/perf
ros2 topic hz /yolo/detections
```

Expected:

- `/vision/perf` publishes repeatedly
- `/yolo/detections` publishes when the camera sees configured classes

## Passive Smoke Check

This helper does not publish motion commands:

```bash
scripts/check_real_teleop_perception.sh
```

To make missing detections fatal:

```bash
OMNISEER_REQUIRE_DETECTIONS=1 scripts/check_real_teleop_perception.sh
```

## If the MCU Path Is Broken

If no MCU topics appear:

- check the actual `/dev/ttyACM*` device
- check `ROS_DOMAIN_ID`
- confirm the correct firmware is flashed
- confirm the container has access to the serial device

If `/encoder_counts` appears but teleop does not move the robot:

- inspect `/mecanum_drive_controller/reference`
- confirm teleop is publishing stamped messages
- confirm the firmware still subscribes to `/mecanum_drive_controller/reference`

If you want to debug the agent separately, stop the launch and run:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6
```

Replace `/dev/ttyACM0` if your board is on a different device.

## Notes to Record After a Real Run

- Date
- Robot or SBC name
- Serial device used
- Camera device used
- Asset paths used
- Whether `/encoder_counts`, `/imu`, `/range`, `/vision/perf`, and
  `/yolo/detections` were observed
- Whether teleop reached `/mecanum_drive_controller/reference`
- Any fatal errors or unexpected disconnects
