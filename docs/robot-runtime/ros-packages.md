# ROS Packages and Sim/Real Boundary

## Purpose

This page is the current reference for the ROS boundary between Gazebo
simulation and the physical robot. The boundary is the set of normalized ROS
contracts consumed by localization, mapping, navigation, autonomy, and
operator tooling. Provider-specific transport, sensor, and compute details
remain below it.

Simulation validates launch composition and the ROS contracts it exercises. It
does not validate physical sensor behavior, firmware timing, micro-ROS
transport, camera acceleration, calibration, or robot motion.

## Current Boundary Contract

`twist_mux` publishes the stamped command contract used by both providers. The
simulated `mecanum_drive_controller` and the Teensy firmware both consume
`/mecanum_drive_controller/reference`; the controller-shaped topic name is
therefore part of the current public boundary.

The common graph consumes wheel odometry, IMU, and scan data through the same
topic names. `robot_io_adapters` derives real wheel odometry from the raw
encoder-count topic so that both paths expose
`/mecanum_drive_controller/odometry`.

| Topic or service | Type | Simulation producer | Real-hardware producer | Current consumers / status |
| --- | --- | --- | --- | --- |
| `/mecanum_drive_controller/reference` | `geometry_msgs/msg/TwistStamped` | `twist_mux` feeds the Gazebo `mecanum_drive_controller` | `twist_mux` feeds the Teensy through the micro-ROS agent | Command input for both motion providers. Firmware retains motor control, kinematics, and timeout handling. |
| `/mecanum_drive_controller/odometry` | `nav_msgs/msg/Odometry` | Gazebo `mecanum_drive_controller` | `encoder_counts_to_odometry` derives it from Teensy `/encoder_counts` | EKF. Frame IDs are configured as `odom` and `base_link`. |
| `/imu` | `sensor_msgs/msg/Imu` | Gazebo sensor through `ros_gz_bridge` | Teensy micro-ROS publisher | EKF. |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo LiDAR through `ros_gz_bridge` | `rplidar_ros` | RF2O, SLAM, and Nav2. |
| `/range` | `sensor_msgs/msg/Range` | `scan_to_range` converts the Gazebo `/sonar` `LaserScan` | Teensy HC-SR04 publisher | Optional target-centering autonomy input. The simulated conversion is a contract adapter, not a physical-sonar equivalence claim. |
| `/yolo/detections` | `yolo_msgs/msg/DetectionArray` | Optional `yolo_bringup` YOLO-World provider using `/front_camera/image` | `omniseer_vision_bridge` native runtime | Optional gateway, target-centering autonomy, and run recording. |
| `/vision/perf` | `omniseer_msgs/msg/VisionPerfSummary` | No producer | `omniseer_vision_bridge` native runtime | Optional gateway and run recording; absent in normal simulation. |
| `/battery` | `sensor_msgs/msg/BatteryState` | No producer | Teensy micro-ROS publisher | Optional gateway and run recording; absent in simulation. |
| `/vision/capture_frame` | `omniseer_msgs/srv/CaptureFrame` | No provider | `omniseer_vision_bridge` | Optional target-centering evidence capture. This is a real-vision capability, not a sim/real-parity contract. |

`/encoder_counts` (`omniseer_msgs/msg/WheelEncoderCounts`) is intentionally
below the common boundary. It is a real-only firmware output consumed by the
odometry adapter, not a topic required by simulation consumers. The message
contains a timestamp and front-left, front-right, rear-left, and rear-right
counts.

Raw camera input is also provider-local: Gazebo publishes
`/front_camera/image` through `ros_gz_image`, while the native vision runtime
captures from its configured V4L2 device. The shared perception output is
`/yolo/detections`.

## Simulation Producers

`sim_io.launch.py` owns Gazebo, the Gazebo bridges, robot spawning, the
Gazebo-backed controller path, and the scan-to-range adapter. The Gazebo model
in `omniseer.urdf.xacro` defines the camera, sonar, LiDAR, IMU,
`gz_ros2_control`, and the odometry publisher. `bridge_config.yaml` bridges
`/clock`, `/imu`, `/scan`, `/sonar`, and `/front_camera/camera_info`; the
separate image bridge exposes `/front_camera/image`.

The optional simulation detection provider is launched by
`perception.launch.py` when `start_yolo:=true`. It uses the simulation image
topic and publishes under the `yolo` namespace, including
`/yolo/detections`.

## Real-Hardware Producers

`real_io.launch.py` starts the real I/O side:

- a Teensy device preflight followed by `micro_ros_agent` when enabled;
- the RPLidar driver publishing `/scan` when enabled; and
- `encoder_counts_to_odometry`, configured by `encoder_odometry.yaml`.

The firmware publishes `/encoder_counts`, `/imu`, `/range`, and `/battery`,
and subscribes to `/mecanum_drive_controller/reference` as a
`TwistStamped` message. The firmware continues to own command handling,
mecanum kinematics, motor output, encoder acquisition, IMU acquisition, and
sonar acquisition.

`real.launch.py` separately includes `real_vision.launch.py` by default.
That launch starts `omniseer_vision_bridge`, which publishes
`/yolo/detections` and `/vision/perf` and serves `/vision/capture_frame`.
It is a real-only dependency on the configured camera, native vision runtime,
and its model assets.

## Shared Launch Composition

The launch composition in the repository is:

```text
sim.launch.py   = pre-launch cleanup + sim_io.launch.py  + common.launch.py + optional RViz
real.launch.py  = pre-launch cleanup + real_io.launch.py + real_vision.launch.py
                  + common.launch.py + optional autonomy, recording, and gateway
```

`common.launch.py` starts the layers shared by the two top-level paths:

- `description.launch.py` and `robot_state_publisher`;
- EKF using either `ekf_fusion.yaml` or `ekf_fusion_real.yaml`;
- `perception.launch.py`, which hosts SLAM, RF2O, and the optional YOLO
  provider;
- `twist_mux`; and
- Nav2 after `/odometry/filtered` and `/map` are available.

The gateway is optional in the shared launch. In the real top-level launch,
optional target-centering autonomy and RunBundle recording are added outside
the common layer.

`real.launch.py` defaults to waiting for first messages on `/imu`,
`/encoder_counts`, `/scan`, and `/mecanum_drive_controller/odometry` before it
starts the common graph. It starts a baseline `twist_mux` while that wait runs,
so teleoperation remains available. The check is a launch precondition, not
hardware acceptance evidence.

The currently shared description launch uses
`omniseer_description/urdf/xacro/omniseer.urdf.xacro`. That xacro still embeds
Gazebo sensors, Gazebo friction, and `gz_ros2_control`; it is shared as a
launch file, not yet a simulation-free robot-description artifact.

## Adapter Responsibilities

`robot_io_adapters` contains compute-bearing conversions only:

- `encoder_counts_to_odometry` integrates the real
  `WheelEncoderCounts` stream with configured mecanum geometry and publishes
  `nav_msgs/msg/Odometry`. It uses explicit wheel ordering, timestamps, and
  `odom`/`base_link` frame parameters.
- `scan_to_range` converts a `sensor_msgs/msg/LaserScan` source to
  `sensor_msgs/msg/Range`. Simulation uses it to expose `/range` from the
  Gazebo sonar scan.

The package does not provide name-only relays. Simulation and firmware publish
the aligned command, IMU, and range names directly.

## Remaining Parity Gaps

- The shared xacro contains Gazebo-only sensors, plugins, and control blocks.
- The optional simulation YOLO provider remains in `perception.launch.py`,
  which is included by `common.launch.py`, rather than in `sim_io.launch.py`.
- Simulation has no `/vision/perf`, `/battery`, or `/vision/capture_frame`
  provider. Those are optional real-side capabilities.
- The simulation EKF configuration fuses wheel odometry, RF2O odometry, and
  IMU; the real default configuration fuses wheel odometry and IMU only.
  Real RF2O may still be launched, but it is not an input to the real-default
  EKF configuration.
- The smoke test checks topic existence and type only. It does not assert the
  command topic, `/range`, perception topics, frame IDs, timestamps, rates,
  stale-data behavior, control timeouts, or simulated/real sensor fidelity.
- There is no mocked real-I/O launch smoke path. The real launch requires its
  configured Teensy and LiDAR inputs when their enabled defaults are used.

## Verification Boundary

The ROS CI job builds and tests the portable ROS package set and runs the
headless Gazebo smoke test. The smoke test launches `bringup sim.launch.py`
with navigation, SLAM, RF2O, RViz, and gateway disabled, then verifies that
the running launch reports these topic types:

| Topic | Required type in smoke test |
| --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` |
| `/imu` | `sensor_msgs/msg/Imu` |
| `/scan` | `sensor_msgs/msg/LaserScan` |
| `/mecanum_drive_controller/odometry` | `nav_msgs/msg/Odometry` |

This CI coverage verifies the tested simulation launch and four interfaces. It
does not verify the physical robot. Hardware acceptance requires the target
SBC, Teensy, LiDAR, camera, micro-ROS transport, native vision dependencies,
and an observed run with the applicable sensors and safety controls.

## Primary Implementation Files

- `ros_ws/src/bringup/launch/common.launch.py`
- `ros_ws/src/bringup/launch/description.launch.py`
- `ros_ws/src/bringup/launch/sim.launch.py`
- `ros_ws/src/bringup/launch/sim_io.launch.py`
- `ros_ws/src/bringup/launch/real.launch.py`
- `ros_ws/src/bringup/launch/real_io.launch.py`
- `ros_ws/src/bringup/launch/real_vision.launch.py`
- `ros_ws/src/bringup/launch/perception.launch.py`
- `ros_ws/src/bringup/config/bridge_config.yaml`
- `ros_ws/src/bringup/config/encoder_odometry.yaml`
- `ros_ws/src/bringup/config/ekf_fusion.yaml`
- `ros_ws/src/bringup/config/ekf_fusion_real.yaml`
- `ros_ws/src/bringup/config/twist_mux.yaml`
- `ros_ws/src/robot_io_adapters/src/encoder_counts_to_odometry_node.cpp`
- `ros_ws/src/robot_io_adapters/src/scan_to_range_node.cpp`
- `ros_ws/src/omniseer_description/urdf/xacro/omniseer.urdf.xacro`
- `firmware/include/micro_ros_config.hpp`
- `firmware/src/micro_ros_node.cpp`
- `ros_ws/src/bringup/test/test_sim_launch_smoke.py`
- `.github/workflows/ci.yml`
