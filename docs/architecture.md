# System Architecture

This page is the top-level technical map of Omniseer. It distinguishes the
implemented robot and operator paths from the planned experiment and cloud-review
work.

## Active Direction

The active deliverable is an open-vocabulary edge perception and evaluation loop:

```text
                       [ Laptop / Planned Cloud Review ]
                                     ^
                                     |
                         experiment results and evidence
                                     |
 [ Camera ] -> [ Native Vision Runtime ] -> [ ROS 2 Contracts ]
                    |       |                    |
                    |       +-> /vision/perf     +-> /yolo/detections
                    |
              V4L2 -> RGA -> RKNN
```

The robot performs inference locally. ROS 2 carries normalized detections and
performance summaries. The next product slice will record those outputs into a
reproducible experiment bundle and support offboard review. Cloud synchronization
and hosted reporting are planned, provider-neutral work.

Navigation, SLAM, simulation, firmware, and operator connectivity remain valuable
platform capabilities. They support data collection and robot operation but are not
the primary portfolio deliverable. Autonomous object search and capture are deferred.

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

The laptop currently supports:

- gRPC status and preview control
- SRT preview receive and decode
- CLI, monitor shell, and initial Tk monitoring workflows
- RViz, telemetry analysis, and other development tools

The laptop is also the first review target for recorded perception experiments. It
keeps dashboard, plotting, and evidence inspection work off the robot.

### Cloud Review Layer

**Planned:** upload a provider-neutral experiment bundle and render a hosted review of
latency, detections, confidence, evidence, and failure cases. The repository does not
currently implement cloud transport, storage, or a hosted dashboard.

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
and then runs producer and consumer threads. Runtime class replacement is implemented
in the Python `yolo_ros` integration but not yet in the native RKNN bridge.

### Operator Diagnostics

```text
ROS status -> C++ gateway -> gRPC -> laptop tools
                   |
                   +-> managed GStreamer worker -> MPEG-TS/SRT preview
```

The gateway aggregates vision and odometry health, implements the locked unary gRPC
API, samples platform status for operator diagnostics, and manages preview as an
optional child process. The current preview path is a software x264 bringup path;
hardware H.265 remains planned.

### Simulation and Hardware

Simulation and real bringup share a common graph above explicit command, odometry,
IMU, LiDAR, range, detection, performance, and battery contracts. GitHub CI launches
headless Gazebo and verifies five core boundary topics. Real device behavior remains a
hardware validation responsibility.

## Capability Status

| Capability | Status | Evidence boundary |
| --- | --- | --- |
| Native producer and consumer vision pipeline | **Hardware-verified** | V4L2, RGA, RKNN target tests and harness |
| YOLO-World post-processing and text embeddings | **Hardware-verified** | RKNN tests and integrated native runtime |
| ROS detection and performance publication | **Implemented** | `omniseer_vision_bridge` |
| Portable ROS, vision, firmware, simulation, and docs checks | **CI-verified** | GitHub Actions six-job workflow |
| gRPC gateway, platform diagnostics, and managed SRT preview | **Implemented** | C++ and Python tests plus local integration |
| Native runtime class updates | **Planned** | Python integration exists; native bridge support does not |
| Structured experiment recorder and run bundle | **Planned** | No integrated recorder exists |
| Recorded resource telemetry in experiment bundles | **Planned** | Gateway live status reports compute/network/power; recorder does not persist it yet |
| Cloud synchronization and hosted dashboard | **Planned** | No provider or transport selected |
| Autonomous semantic search and capture | **Deferred** | Outside the active deliverable |

## Repository Ownership

- `vision/` owns the native camera-to-detection runtime and detailed telemetry.
- `ros_ws/src/omniseer_vision_bridge/` owns the native-to-ROS adapter.
- `ros_ws/src/bringup/` owns sim and real launch composition.
- `robot_diag_control_cpp` owns the robot-side external gateway boundary.
- `robot_diag_control` owns host-side operator tools.
- `firmware/` owns MCU behavior and micro-ROS IO.
- `docs/` owns current-state specifications and operational guidance.

## Related Documentation

- [Edge-to-Cloud Perception](software/edge_to_cloud_perception.md)
- [Vision Pipeline](software/vision_pipeline.md)
- [Vision Telemetry](software/vision_telemetry_spec.md)
- [Robot Gateway](software/robot_gateway.md)
- [Preview Streaming](software/preview_streaming.md)
- [CI/CD Overview](software/ci_cd.md)
