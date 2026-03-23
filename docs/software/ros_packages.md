# ROS Packages and Sim/Real Boundary

_Status: draft v0_

_Last updated: 2026-03-22_

## Purpose

Define a concrete package and launch structure so that:

- simulation and real hardware share the same ROS graph above a thin boundary
- the boundary is explicit, testable, and small
- simulation becomes a strong indicator of software architecture and contract correctness
- hardware-specific code stays below the boundary

This page is intentionally practical. It is the working plan for the next
bringup refactor, not just a general architecture description.

## Core Decision

The recommended near-term boundary is:

- `common` owns the mission ROS graph that consumes normalized robot IO
- `sim` owns Gazebo, sim-only plugins, bridges, and sim-only compute adapters
- `real` owns the MCU bridge, sensor drivers, SBC camera runtime, and real-only compute adapters

The boundary should be a canonical ROS contract, not Gazebo internals and not
`ros2_control` internals.

For this refactor, prefer source-side standardization over pure rename relays:

- if a mismatch is only topic naming, fix the producer or subscriber
- if a mismatch is only stamped-vs-unstamped command shape, update the real
  consumer to match the sim/common contract
- only keep adapter nodes when they derive or transform data semantically, such
  as encoder counts to odometry

Why this is the right first move:

- it matches the current firmware shape
- it minimizes churn while still making sim and real converge
- it lets us keep the current MCU motion-control path while still normalizing
  the ROS graph above it
- it preserves a later path to a true `ros2_control` hardware interface if that
  later pays for itself

## Current State

Today the boundary is not aligned.

### Simulation Side

Simulation currently provides:

- controller path via `mecanum_drive_controller`
- odometry via `/mecanum_drive_controller/odometry`
- IMU via `/imu`
- LiDAR via `/scan`
- sonar as `LaserScan` on `/sonar`, then converted to `/range`
- camera image via `/front_camera/image`
- simulated YOLO provider inside `perception.launch.py`

Relevant files:

- `ros_ws/src/bringup/launch/orchestrate_sim.launch.py`
- `ros_ws/src/bringup/launch/controllers.launch.py`
- `ros_ws/src/bringup/config/bridge_config.yaml`
- `ros_ws/src/bringup/config/ekf_fusion.yaml`
- `ros_ws/src/robot_io_adapters/src/scan_to_range.cpp`
- `ros_ws/src/omniseer_description/urdf/xacro/omniseer.urdf.xacro`

### Real Hardware Side

The MCU currently provides:

- input command on `/mecanum_drive_controller/reference` as `geometry_msgs/msg/TwistStamped`
- encoder counts on `/encoder_counts`
- IMU on `/imu`
- sonar range on `/range`
- battery on `/battery`

The SBC currently provides:

- real camera inference runtime publishing `/yolo/detections`
- vision perf on `/vision/perf`
- operator gateway consuming `/vision/perf` and filtered odometry

Relevant files:

- `firmware/include/micro_ros_config.hpp`
- `firmware/src/micro_ros_node.cpp`
- `firmware/src/omniseer_tasks.cpp`
- `firmware/include/omniseer_config.hpp`
- `ros_ws/src/omniseer_vision_bridge/src/vision_bridge_node.cpp`
- `ros_ws/src/robot_diag_control_cpp/src/robot_diag_control_cpp_node.cpp`

### Main Mismatches

The key remaining mismatches are:

- wheel odom derivation mismatch:
  - sim publishes `/mecanum_drive_controller/odometry` directly
  - real still derives that contract from raw `/encoder_counts`
- sonar implementation mismatch:
  - sim produces `/sonar` as `LaserScan` and adapts it to `/range`
  - real publishes `/range` directly
- description mismatch:
  - the shared xacro currently embeds Gazebo-only sensors and Gazebo-only
    control plugins
- launch-topology mismatch:
  - sim has a staged orchestration path
  - real bringup currently covers only the phase-1 MCU/encoder-odometry slice

The earlier topic-only naming mismatches and the firmware/sim kinematics drift
have been addressed in the phase-1 slice:

- the MCU now uses the sim/common command, IMU, and range topic names directly
- the firmware kinematics constants now match the current sim/controller values

Wheel odometry still remains a compute boundary, because the MCU publishes raw
encoder counts rather than odometry.

## Recommended Boundary Contract

The boundary should be a small set of canonical topics.

These are the topics that `common` is allowed to know about. Everything below
them belongs to `sim` or `real`.

Near-term, we intentionally keep the existing sim-facing names for command and
wheel odometry. That leaks some controller naming upward, but it avoids a layer
of no-op relays and gets us to sim/real parity faster.

| Topic | Type | Canonical frame / semantics | Producer / standardization path | Consumers above boundary |
| --- | --- | --- | --- | --- |
| `/mecanum_drive_controller/reference` | `geometry_msgs/msg/TwistStamped` | base command in base frame | `twist_mux` in common, real MCU updated to subscribe directly | sim controller, real MCU |
| `/mecanum_drive_controller/odometry` | `nav_msgs/msg/Odometry` | wheel/base odom, `odom -> base_link`, no global correction | sim controller direct or real encoder odom adapter | EKF, diagnostics |
| `/imu` | `sensor_msgs/msg/Imu` | IMU at `imu_link` | Gazebo bridge or MCU renamed to publish directly | EKF |
| `/scan` | `sensor_msgs/msg/LaserScan` | planar LiDAR at `lidar_frame` | Gazebo bridge or real LiDAR driver | RF2O, SLAM, costmaps |
| `/range` | `sensor_msgs/msg/Range` | forward range at `sonar_link` | sim sonar adapter or MCU renamed to publish directly | costmaps |
| `/yolo/detections` | `yolo_msgs/msg/DetectionArray` | source-space detections | sim YOLO provider or real vision bridge | shared policy/consumers |
| `/vision/perf` | `omniseer_msgs/msg/VisionPerfSummary` | normalized vision health/perf | real vision bridge, optional sim stub | gateway/diagnostics |
| `/battery` | `sensor_msgs/msg/BatteryState` | robot battery state | MCU direct | optional UI/diagnostics |

### Raw Image Contract

The raw camera image does **not** need to be part of the common mission graph
to get the sim-to-real benefit we want today.

For current navigation and contract work, the more important shared boundary is:

- command
- wheel odometry
- IMU
- scan
- range
- detections
- perf/diagnostics

Raw camera image can remain provider-local for now:

- sim YOLO consumes `/front_camera/image` below the boundary
- real vision runtime consumes `/dev/video12` below the boundary

Both then converge on `/yolo/detections`.

That is the right boundary for the current navigation-policy and graph-contract
goal.

## Why Not Use `ros2_control` as the Boundary Yet

That remains a valid later direction, but it is not the smallest effective move
right now.

### Later Option

Later, the real robot could expose a true `ros2_control` hardware interface so
both sim and real use:

- `controller_manager`
- `joint_state_broadcaster`
- `mecanum_drive_controller`

with only the hardware plugin changing.

### Why Not First

The current firmware already owns:

- command timeout
- mecanum kinematics
- wheel command writeout
- encoder readout

Moving that boundary immediately would require:

- a real hardware interface implementation
- transport design between SBC and MCU
- lifecycle/error handling around that transport
- a migration of control semantics that is larger than the current need

That can pay off later. It is not the smallest next step.

## Target Package and Launch Layout

### `omniseer_description`

Split the description into pure robot model vs sim-only attachments.

Recommended files:

- `ros_ws/src/omniseer_description/urdf/xacro/omniseer.base.urdf.xacro`
- `ros_ws/src/omniseer_description/urdf/xacro/omniseer.sim_sensors.xacro`
- `ros_ws/src/omniseer_description/urdf/xacro/omniseer.sim_control.xacro`
- `ros_ws/src/omniseer_description/urdf/xacro/omniseer.sim.urdf.xacro`
- `ros_ws/src/omniseer_description/urdf/xacro/omniseer.real.urdf.xacro`

Responsibilities:

- `omniseer.base.urdf.xacro`
  - links, joints, frames, inertias, geometry
  - no Gazebo plugins
  - no Gazebo sensors
  - no Gazebo `ros2_control`
- `omniseer.sim_sensors.xacro`
  - Gazebo camera, lidar, sonar, IMU blocks
- `omniseer.sim_control.xacro`
  - Gazebo `ros2_control` block and Gazebo wheel/friction tuning
- `omniseer.sim.urdf.xacro`
  - includes base + sim sensor/control overlays
- `omniseer.real.urdf.xacro`
  - includes base only for now

This lets `robot_state_publisher` be shared without dragging Gazebo-only
behavior into the real path.

### `bringup`

Recommended launch layout:

- `ros_ws/src/bringup/launch/description.launch.py`
- `ros_ws/src/bringup/launch/common.launch.py`
- `ros_ws/src/bringup/launch/sim_io.launch.py`
- `ros_ws/src/bringup/launch/real_io.launch.py`
- `ros_ws/src/bringup/launch/sim.launch.py`
- `ros_ws/src/bringup/launch/real.launch.py`

Status:

- `description.launch.py`, `common.launch.py`, `sim_io.launch.py`, and
  `real_io.launch.py` are now the shared baseline launch layers
- `sim.launch.py` and `real.launch.py` both include the shared common layer
- the gateway is hosted in `common.launch.py` behind a flag instead of a
  separate mandatory top-level launch
- RF2O, SLAM, and Nav2 are currently being hosted from `common.launch.py`
  ahead of final `/scan` parity validation

Recommended responsibilities:

### `description.launch.py`

Shared robot description launch.

Inputs:

- xacro path
- `use_sim_time`

Starts:

- `robot_state_publisher`

### `common.launch.py`

Shared mission/runtime graph above the boundary.

Starts:

- `description.launch.py`
- EKF
- RF2O
- SLAM
- Nav2
- `twist_mux`
- optional gateway / diag control

Must **not** start:

- Gazebo
- Gazebo bridges
- `mecanum_drive_controller`
- micro-ROS agent
- real LiDAR driver
- SBC-specific camera provider
- sim-only YOLO provider

### `sim_io.launch.py`

Simulation-only producers and compute adapters below the boundary.

Starts:

- Gazebo
- robot spawn into Gazebo
- Gazebo ROS bridges
- Gazebo-side control path
- sim-only compute adapters such as `scan_to_range`

### `real_io.launch.py`

Real-hardware-only producers and compute adapters below the boundary.

Starts:

- `micro_ros_agent`
- real LiDAR driver publishing `/scan`
- MCU publishers/subscribers using sim-aligned topic names
- real encoder-to-odom adapter

### `sim.launch.py`

Top-level sim bringup:

- start `sim_io.launch.py`
- start `common.launch.py`
- optionally start RViz

### `real.launch.py`

Top-level real bringup:

- start `real_io.launch.py`
- wait for `/imu`, `/scan`, and `/mecanum_drive_controller/odometry`
- start `common.launch.py`

## New Adapter Package

Create a focused package only for boundary transformations that do real work.

Recommended package:

- `ros_ws/src/robot_io_adapters`

Use `ament_cmake` and keep the adapter core in testable C++ code. This is part
of the real hardware boundary and should feel native to the rest of robot-core.

Pure rename or pass-through relay nodes should not live here. If a node only
changes names or copies fields without changing semantics, fix the source or
consumer instead.

Recommended nodes:

### `encoder_counts_to_odometry`

Purpose:

- subscribe to `encoder_counts`
- compute wheel/base odometry using the same mecanum geometry used elsewhere
- publish `/mecanum_drive_controller/odometry`

Why:

- this is the most important missing real adapter
- it makes real hardware publish the same odometry contract that sim already
  provides

This node should be deterministic and boring:

- no hidden filtering beyond what is necessary for finite differencing
- explicit wheel order mapping
- explicit timestamp handling
- explicit frame ids

### Optional `vision_perf_stub.py`

Purpose:

- publish a minimal `/vision/perf` heartbeat in sim when no native perf source exists

Why:

- lets gateway/diagnostic flows behave more similarly in sim and real

This is optional for the first slice.

## Concrete Refactor of Existing Files

The following changes should happen early because they define the boundary.

### `nav.launch.py`

Current issue:

- `twist_mux` output is remapped directly to
  `/mecanum_drive_controller/reference`

Target:

- keep `twist_mux` output on `/mecanum_drive_controller/reference`
- update the MCU subscriber to match that contract directly

Reason:

- avoids a no-op command shim
- preserves the current working sim/common command contract

### `ekf_fusion.yaml`

Current issue:

- `odom0` uses `/mecanum_drive_controller/odometry`
- `imu0` uses `/imu`

Target:

- keep `odom0: /mecanum_drive_controller/odometry`
- keep `imu0: /imu`
- make real hardware conform to those names

Reason:

- avoids no-op relays for odom naming and IMU naming
- keeps the current sim/common EKF contract intact

For the real-only baseline, use a dedicated `ekf_fusion_real.yaml` that drops
the simulated RF2O input and fuses only wheel odometry plus IMU.

### `firmware/include/micro_ros_config.hpp` and `firmware/src/micro_ros_node.cpp`

Phase-1 implementation:

- MCU topic constants use `/mecanum_drive_controller/reference`, `/imu`, and `/range`
- MCU command subscriber expects `geometry_msgs/msg/TwistStamped`

Behavior preserved:

- keep the downstream motion-controller semantics unchanged after the ROS
  message is unpacked

Why this shape is still correct:

- removes no-op rename/shim nodes from the real path
- makes the real ROS surface match the current sim/common contract directly
- keeps the firmware control loop architecture intact while normalizing the ROS
  boundary

### `perception.launch.py`

Current issue:

- it mixes shared perception consumers with sim-only detection provider
- it also starts sim sonar conversion

Target:

- keep only shared consumers above the boundary:
  - RF2O
  - SLAM
  - any detection consumers that consume `/yolo/detections`
- move sim-only YOLO provider to `sim_io.launch.py`
- move sim-only sonar conversion below the boundary

### `controllers.launch.py`

Current issue:

- controller manager remains coupled to the shared graph shape

Target:

- treat it as sim-only below the boundary for now
- only `sim_io.launch.py` should include it

### `robot_io_adapters/scan_to_range.cpp`

Current issue:

- it is a sim-only adapter but is launched from shared runtime paths

Target:

- keep the node, but launch it only from `sim_io.launch.py`

## Current Common Graph After Refactor

The target common graph is:

```text
             /mecanum_drive_controller/reference
                            |
                            v
                       [ common graph ]
          +----------------+----------------+----------------+
          |                |                |                |
       localization      mapping          nav            diagnostics
          |                |                |                |
          +----------------+----------------+----------------+
                            |
             consumes canonical robot-IO boundary only
                            |
 /mecanum_drive_controller/odometry  /imu  /scan  /range  /yolo/detections
```

Below that line:

- sim provides those topics through Gazebo + bridges + compute adapters
- real provides those topics through MCU/driver/runtime + direct naming
  standardization plus compute adapters

## Recommended Phase Order

### Phase 0: Lock Contracts and Constants

Do first:

- agree on canonical topic names
- agree on message types
- agree on frame ids
- unify mecanum geometry constants between firmware and ROS

Suggested output:

- one shared constants source or one generated config source

This phase is required before any odometry adapter work.

### Phase 1: Normalize the Command and Odometry Boundary

Implement:

- MCU subscription rename and message update to
  `/mecanum_drive_controller/reference`
- `encoder_counts_to_odometry`

Then update:

- MCU micro-ROS topic definitions
- real bringup to launch the encoder odom adapter

This is the highest-value slice because it makes localization and navigation
consume the same semantic inputs in sim and real.

### Phase 2: Normalize IMU and Range

Implement:

- MCU IMU topic rename to `/imu`
- MCU sonar topic rename to `/range`

Then update:

- firmware topic/config constants
- real bringup docs and contract checks

This removes the last obvious sensor-topic mismatch in the navigation path.

### Phase 3: Split the Description and Launch Topology

Implement:

- base/sim/real xacro split
- `description.launch.py`
- `common.launch.py`
- `sim_io.launch.py`
- `real_io.launch.py`
- real top-level launch

This is the point where the codebase layout starts reflecting the architecture
we actually want.

### Phase 4: Move Detection Providers Below the Boundary

Implement:

- move sim YOLO provider into `sim_io.launch.py`
- keep real vision bridge in `real_io.launch.py`
- treat `/yolo/detections` as the shared detection contract

Optional:

- add sim `/vision/perf` stub for diagnostic parity

### Phase 5: Verification Infrastructure

Add targeted verification for both sim and real:

- contract test: required boundary topics exist
- contract test: expected message types
- contract test: expected `frame_id`s
- contract test: expected rates are within bounds
- launch smoke: `sim.launch.py`
- launch smoke: `real.launch.py` with drivers mocked or selectively disabled
- rosbag replay support for the canonical boundary topics

This phase is what turns the architecture into evidence rather than intent.

## Verification Standard for the Boundary

Simulation becomes a strong indicator only if the contract is verified.

Minimum checks per boundary topic:

- topic exists
- type matches
- frame id matches
- timestamp moves forward correctly
- publish rate is within expected bounds
- stale detection is visible and explicit

For command and odometry specifically:

- command timeout behavior must be explicit
- sign conventions must be explicit
- wheel ordering must be explicit
- geometry constants must be shared

## What This Refactor Will and Will Not Prove

After this refactor, simulation will be a strong indicator for:

- launch composition
- topic and frame contracts
- TF topology
- navigation wiring
- SLAM and localization graph behavior
- policy consumers of `/yolo/detections`
- gateway status wiring and optional operator flow wiring

It will still not fully prove:

- SBC compute headroom
- RKNN/RGA/V4L2 behavior
- Wi-Fi link behavior
- hardware timing jitter
- calibration correctness
- micro-ROS transport reliability

That is expected. The goal is not to make sim prove everything. The goal is to
make sim prove the software architecture and contracts above a thin,
well-defined hardware boundary.

## Recommended Immediate Next Slice

The smallest high-value next slice is:

1. unify mecanum constants across firmware and ROS
2. update the MCU command subscriber to use
   `/mecanum_drive_controller/reference`
3. add `robot_io_adapters/encoder_counts_to_odometry`
4. update the MCU IMU topic to `/imu`
5. update the MCU sonar topic to `/range`
6. keep EKF on `/mecanum_drive_controller/odometry` and `/imu`
7. move `controllers.launch.py` and `scan_to_range` below the sim boundary

That slice gives the biggest sim-to-real payoff without first rewriting the
firmware control architecture.
