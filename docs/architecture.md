# System Architecture

This page is the top-level technical map of the repository. It describes the
major runtime boundaries, what currently exists, and which subsystem pages hold
the deeper design notes.

Omniseer currently spans:

- an SBC-hosted ROS 2 robot runtime
- a hardware-accelerated vision stack on Rockchip hardware
- robot description, bringup, and analysis packages
- firmware for lower-level hardware integration
- simulation assets and launch support
- an emerging operator connectivity stack for remote monitoring and control

The repository is still in an active architecture-building phase. Some
subsystems, especially the native vision runtime, are already implemented in
detail. Others, such as the robot gateway and preview export path, are now at
the spec and first-integration stage.

## Major Runtime Boundaries

### Robot SBC

The robot SBC is the primary runtime host.

Current and planned responsibilities:

- ROS 2 graph for perception, navigation, control, and local diagnostics
- hardware-accelerated vision pipeline using V4L2, RGA, and RKNN
- mission-critical consumers of detections and other sensor state
- local bringup and launch orchestration
- optional operator-facing preview export path

The current preferred deployment model is one `robot-core` container on the SBC
with mission-critical ROS nodes always on. Optional operator diagnostics, such
as preview streaming, should be isolated enough that failure does not interfere
with the mission path.

### Operator Laptop

The operator laptop is the preferred location for higher-cost human-facing work.

Planned responsibilities:

- native operator application
- RViz2
- host-side overlay and analysis tools
- bagging, replay, plotting, and inspection workflows
- gRPC client for robot control/state access
- SRT video receive and decode

This keeps the robot focused on low-latency onboard work while moving
diagnostic-heavy workloads offboard.

### Optional Cloud / Remote Bridge

The current repo does not implement a cloud bridge yet, but the architecture is
being shaped so that one could be added later without exposing the internal ROS
graph directly.

Potential later roles:

- fleet telemetry
- offsite health/status access
- web-facing dashboards
- remote ops support through a narrower external API

## Current High-Level Shape

```text
                        [ Operator Laptop ]
                               |
                 +-------------+-------------+
                 |                           |
              gRPC state/control         SRT preview
                 |                           |
                 +-------------+-------------+
                               |
                        [ Robot Gateway ]
                               |
          +--------------------+--------------------+
          |                    |                    |
       diagnostics          teleop              mission
          |                    |                    |
          +--------------------+--------------------+
                               |
                         [ ROS 2 graph ]
                  perception / nav / control / logging
                               |
                     +---------+---------+
                     |                   |
                  vision runtime      firmware / IO
```

This is the target direction. The current repo is partway there:

- the native vision runtime exists
- the ROS bridge for detections exists
- the operator gateway does not yet exist as a first-class component
- the preview export path is specified but not yet integrated

## Major Software Subsystems

### Vision Runtime

The most detailed and mature subsystem in the repo today.

Responsibilities:

- capture from the ISP via V4L2
- preprocess using RGA
- run model inference via RKNN
- produce canonical detections and telemetry

Primary docs:

- [Vision pipeline](software/vision_pipeline.md)
- [Producer pipeline](software/producer_pipeline.md)
- [Consumer pipeline](software/consumer_pipeline.md)
- [Vision telemetry spec](software/vision_telemetry_spec.md)

### ROS Runtime

The ROS layer ties together bringup, robot description, analysis helpers, and
the vision bridge into a graph that can run on the SBC.

Responsibilities:

- package and launch composition
- topic/service boundaries
- bridge between the native vision runtime and ROS-native consumers
- robot description and visualization configuration

Primary docs today:

- [ROS packages](software/ros_packages.md)
- bringup and launch files in `ros_ws/src/bringup`

### Operator Connectivity Stack

This is the next major subsystem being designed.

Responsibilities:

- gRPC control/state API for the operator app
- on-demand preview management
- SRT preview export
- status aggregation for monitoring
- later teleop and operator workflow coordination

Primary docs:

- [Remote monitoring architecture](software/remote_monitoring_architecture.md)
- [Robot gateway](software/robot_gateway.md)
- [Gateway API](software/gateway_api.md)
- [Preview streaming](software/preview_streaming.md)

### Firmware and Hardware Integration

The repository also includes firmware and hardware artifacts which sit below the
SBC runtime boundary.

Responsibilities:

- direct hardware control
- low-level IO
- board/actuator/sensor interfacing
- electrical and mechanical integration

Primary docs today:

- [Circuit](hardware/circuit.md)
- [Wiring guide](hardware/wiring.md)

## Repository Layout

Important top-level directories:

- `vision/`: native vision runtime and tests
- `ros_ws/src/`: ROS 2 packages
- `firmware/`: embedded firmware project
- `sim/`: simulation assets and launch support
- `docs/`: long-form project documentation
- `notes/`: scratch notes and local operational references

## What Exists vs What Is Planned

### Implemented or substantially implemented

- native producer/consumer vision pipeline
- detections bridge into ROS 2
- build and CI basics
- robot description and core bringup scaffolding

### In active specification / integration

- robot gateway for operator-facing control/state
- preview streaming path from `rkisp_mainpath`
- host-side monitoring application shape
- separation between mission-critical and operator-facing runtime concerns

### Explicitly later

- cloud bridge
- browser-first remote ops stack
- hardened fleet/deployment automation

## Documentation Strategy

This repo is gradually moving toward a consistent documentation style:

- one umbrella architecture page per major subsystem
- narrower spec pages for hot paths, contracts, or lifecycle-critical behavior
- implementation notes that reflect current state, not just end-state intent

The vision docs are the current best example of this style, and newer subsystem
docs should trend in that direction as the project is refined.
