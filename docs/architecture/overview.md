# System Architecture

This page is the top-level technical map of Omniseer. It describes the implemented
robot runtime, operator tooling, diagnostic gateway, firmware boundary, and local
experiment-review path.

## System Loop

<object data="../assets/diagrams/explorer/system-explorer.svg" type="image/svg+xml" aria-label="Omniseer system explorer" width="100%"></object>

Omniseer connects onboard perception, bounded robot behavior, and reviewable
experiment evidence:

```text
[ Camera ] -> [ V4L2 / RGA / RKNN Vision Runtime ]
                                |
                 +--------------+--------------+
                 |                             |
          /yolo/detections              /vision/perf
                 |                             |
                 v                             v
 [ Target Acquisition + Framing ]      [ RunBundle Recorder ]
                 |                             |
                 v                             v
          /cmd_vel_autonomy          [ Laptop Inspection ]
```

The robot performs inference locally. ROS 2 carries normalized detections and
performance summaries. The target-centering controller performs a bounded in-place
visual scan for a configured target class, acquires a stable detection, centers the
target horizontally with yaw, and uses small bounded forward or reverse motion to
bring the target's bounding-box area into the configured framing range. Forward
motion is blocked when proximity range crosses the configured safety threshold.
This is bounded visual target acquisition and framing, not navigation-based object
search, global exploration, mapping, or semantic planning.
The experiment workflow records robot-created RunBundles, retrieves them onto the
laptop, annotates selected evidence, and generates static HTML reports.

Navigation, SLAM, simulation, firmware, and operator connectivity support data
collection, hardware operation, and reproducible validation.

## Runtime Boundaries

### Robot SBC

The ROCK 5B+ hosts the mission-critical runtime:

- V4L2 camera capture from the Rockchip ISP
- RGA preprocessing into fixed model input buffers
- RKNN YOLO-World text encoding and detector inference
- bounded post-processing and typed detection publication
- ROS 2 bringup, normalized robot IO, and local diagnostics
- optional gateway and preview subprocesses

Optional diagnostics must not become dependencies of the vision or control path.
The robot-side compute budget prioritizes mission-critical perception, bounded
robot behavior, command arbitration, and robot IO. Preview, dashboards, plotting,
hosted review, and high-cost analysis are optional diagnostic work and belong off
the robot where practical.

### Firmware and Robot IO

The Teensy firmware owns low-level motor and sensor integration through micro-ROS.
Real and simulated producers converge on the normalized boundary topics documented in
[ROS Packages and Sim/Real Boundary](../robot-runtime/ros-packages.md).

### Operator Laptop

The laptop supports:

- gRPC status and preview control
- SRT preview receive and decode
- CLI, monitor shell, and initial Tk monitoring workflows
- RunBundle retrieval, inspection, evidence annotation, and static HTML reports
- RViz, telemetry analysis, and other development tools

The laptop keeps dashboard, plotting, and evidence inspection work off the robot.
Operator connectivity can be disconnected or disabled without changing the local
mission-critical perception and robot behavior contracts.

## Implemented Data Paths

### Perception

```text
/dev/video12 NV12
       |
       v
 V4L2 capture -> RGA letterbox -> latest-wins DMA buffer pool
                                      |
                                      v
                        RKNN inference -> YOLO-World postprocess
                                      |
                         +------------+------------+
                         |                         |
                 /yolo/detections             /vision/perf
```

The native runtime loads its class list during startup, prepares CLIP text embeddings,
and then runs producer and consumer threads. The Python `yolo_ros` integration
includes a `SetClasses` service; the native RKNN bridge uses its configured startup
class list.

### Operator Diagnostics

```text
ROS status -> C++ gateway -> gRPC -> laptop tools
                   |
                   +-> managed GStreamer worker -> MPEG-TS/SRT preview
```

The gateway aggregates vision and odometry health, implements the locked unary gRPC
API, samples platform status for operator diagnostics, and manages preview as an
optional child process. The implemented preview path uses software x264 over
MPEG-TS/SRT.

### Simulation and Hardware

Simulation and real bringup share a common graph above explicit command, odometry,
IMU, LiDAR, range, detection, performance, and battery contracts. GitHub CI launches
headless Gazebo and currently verifies `/clock`, `/imu`, `/scan`, and
`/mecanum_drive_controller/odometry`. Real device behavior remains a hardware
validation responsibility.

## Capability Status

| Capability | Status | Evidence boundary |
| --- | --- | --- |
| Native producer and consumer vision pipeline | **Implemented; documented local target-hardware run evidence** | V4L2, RGA, RKNN target tests and local `runs/pipeline_001` measurements; raw bundle not included in the public repository |
| YOLO-World post-processing and text embeddings | **Implemented; documented local target-hardware run evidence** | RKNN tests, native runtime, and local `runs/pipeline_001` measurements; raw bundle not included in the public repository |
| ROS detection and performance publication | **Implemented** | `omniseer_vision_bridge` |
| Bounded visual target acquisition and framing | **Implemented** | `omniseer_autonomy`, `scripts/omni run autonomy`, controller tests, and RunBundle `autonomy.jsonl` support |
| Portable ROS, vision, firmware, simulation, docs, and portable runtime checks | **CI-verified** | GitHub Actions master-push workflows |
| gRPC gateway, platform diagnostics, and managed SRT preview | **Implemented** | C++ and Python tests plus local integration |
| Structured experiment recorder and run bundle | **Implemented; documented local target-hardware run evidence** | local `runs/pipeline_001` measurements include detections, perf, system telemetry, and native pipeline telemetry; raw bundle not included in the public repository |
| Recorded resource telemetry in experiment bundles | **Implemented; documented local target-hardware run evidence** | local `runs/pipeline_001` measurements include `system.jsonl`; gateway live status remains separate |

## Repository Ownership

- `vision/` owns the native camera-to-detection runtime and detailed telemetry.
- `ros_ws/src/omniseer_vision_bridge/` owns the native-to-ROS adapter.
- `ros_ws/src/omniseer_experiments/` owns local perception run-bundle recording.
- `ros_ws/src/bringup/` owns sim and real launch composition.
- `robot_diag_control_cpp` owns the robot-side external gateway boundary.
- `robot_diag_control` owns host-side operator tools.
- `firmware/` owns MCU behavior and micro-ROS IO.
- `docs/` owns current-state specifications and operational guidance.

## Related Documentation

- [Edge-to-Cloud Perception](../perception/edge-to-cloud.md)
- [Verification Evidence](../verification/evidence.md)
- [Vision Pipeline](../perception/vision-pipeline.md)
- [Vision Telemetry](../perception/vision-telemetry.md)
- [Robot Gateway](../gateway/robot-gateway.md)
- [Preview Streaming](../gateway/preview-streaming.md)
- [CI/CD Overview](../verification/ci-cd.md)
