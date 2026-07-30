# System Architecture

This page is the top-level technical map of Omniseer. It describes the implemented
robot runtime, operator tooling, diagnostic gateway, firmware boundary, and local
experiment-review path.

## System Loop

<object data="assets/diagrams/explorer/system-explorer.svg" type="image/svg+xml" aria-label="Omniseer system explorer" width="100%"></object>

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
     [ Target-Centering Autonomy ]      [ RunBundle Recorder ]
                 |                             |
                 v                             v
          /cmd_vel_autonomy          [ Laptop Inspection ]
```

The robot performs inference locally. ROS 2 carries normalized detections and
performance summaries. The target-centering controller consumes detections and emits
bounded velocity commands through the same arbitration path as other robot commands.
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

### Firmware and Robot IO

The Teensy firmware owns low-level motor and sensor integration through micro-ROS.
Real and simulated producers converge on the normalized boundary topics documented in
[ROS Packages and Sim/Real Boundary](software/ros_packages.md).

### Operator Laptop

The laptop supports:

- gRPC status and preview control
- SRT preview receive and decode
- CLI, monitor shell, and initial Tk monitoring workflows
- RunBundle retrieval, inspection, evidence annotation, and static HTML reports
- RViz, telemetry analysis, and other development tools

The laptop keeps dashboard, plotting, and evidence inspection work off the robot.

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
headless Gazebo and verifies five core boundary topics. Real device behavior remains a
hardware validation responsibility.

## Capability Status

| Capability | Status | Evidence boundary |
| --- | --- | --- |
| Native producer and consumer vision pipeline | **Implemented; target-hardware run evidence** | V4L2, RGA, RKNN target tests and `runs/pipeline_001` recorded native pipeline telemetry |
| YOLO-World post-processing and text embeddings | **Implemented; target-hardware run evidence** | RKNN tests, native runtime, and `runs/pipeline_001` recorded detections |
| ROS detection and performance publication | **Implemented** | `omniseer_vision_bridge` |
| Bounded target-centering autonomy | **Implemented** | `omniseer_autonomy`, `scripts/omni run autonomy`, controller tests, and RunBundle `autonomy.jsonl` support |
| Portable ROS, vision, firmware, simulation, and docs checks | **CI-verified** | GitHub Actions six-job workflow |
| gRPC gateway, platform diagnostics, and managed SRT preview | **Implemented** | C++ and Python tests plus local integration |
| Structured experiment recorder and run bundle | **Implemented; target-hardware run evidence** | `runs/pipeline_001` records detections, perf, system telemetry, and native pipeline telemetry |
| Recorded resource telemetry in experiment bundles | **Implemented; target-hardware run evidence** | `runs/pipeline_001` includes `system.jsonl`; gateway live status remains separate |

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

- [Edge-to-Cloud Perception](software/edge_to_cloud_perception.md)
- [Verification Evidence](evidence.md)
- [Vision Pipeline](software/vision_pipeline.md)
- [Vision Telemetry](software/vision_telemetry_spec.md)
- [Robot Gateway](software/robot_gateway.md)
- [Preview Streaming](software/preview_streaming.md)
- [CI/CD Overview](software/ci_cd.md)
