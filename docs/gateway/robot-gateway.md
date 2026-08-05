# Robot Gateway

_Status: implemented v1 diagnostics, preview-control, and bounded-teleop slice_

This document describes the operator-facing gateway between the robot's internal
ROS 2 graph and the external operator laptop application. The first C++ gateway,
typed gRPC API, preview lifecycle manager, bounded teleop path, and Python operator
tools are implemented.

## Purpose

Provide one explicit integration boundary for operator-facing features:

- gRPC control and state access
- preview session management
- bounded teleop control

The gateway keeps the internal ROS 2 graph private and prevents the laptop app
from becoming a generic DDS client that is tightly coupled to internal topics
and services.

## Current State

What exists today:

- ROS 2 graph on the SBC
- native vision runtime
- `omniseer_vision_bridge` publishing detections into ROS 2
- RViz and debug tooling on the ROS side
- a `robot_diag_control_cpp` package with:
  - a shared generated C++ gRPC/protobuf library from the locked `.proto`
  - an in-memory gateway state store
  - a synchronous unary gRPC service/server layer
  - a ROS-backed node that aggregates `/vision/perf`, filtered odometry, and serves the locked API
  - a gateway-owned preview subprocess manager backing `SetPreviewMode`
  - a bounded teleop manager backing `SetTeleopEnabled` and `SendTeleopCommand`
  - live platform diagnostics for compute, Wi-Fi, LiPo, and onboard battery state
  - a built-in GStreamer preview worker path using `/dev/video11 -> x264 -> MPEG-TS -> SRT`
  - packaged Python client tools for gateway status/control and host-side preview consumption
  - a packaged Python monitor shell that integrates status polling and preview launch
  - a first packaged Tk monitor GUI for desktop status, preview, and teleop bring-up
  - local verification against those packaged Python tools

Current API boundary:

- stream endpoint metadata in the API
- preview transport uses the implemented software x264/MPEG-TS/SRT path
- experiment recording is owned by launch profiles and `scripts/omni runs`

Operating model:

- keep `robot_diag_control_cpp` inside `robot-core`
- use the gateway for optional status, preview control, and bounded operator teleop

## Implementation Shape

The current C++ implementation stays deliberately simple:

- one process
- one synchronous unary gRPC server
- one shared in-memory state store
- one ROS subscription path feeding that store
- one expected operator client

Completed slices:

1. add a standalone C++ gRPC service/server layer with tests
2. wire that layer into the existing ROS-backed node
3. replace the original preview toggle with a gateway-owned subprocess lifecycle
4. wire the first real preview export command into that lifecycle
5. add bounded teleop enable/disable and command forwarding through the gateway
6. surface live platform diagnostics in system status and overlay snapshots

This keeps the control/status boundary small while avoiding premature async
gRPC complexity.

## Major Design Considerations

- Narrow external contract: expose stable RPCs, not internal ROS topic names.

- Mission isolation: preview, UI, and diagnostics must not interfere with
  navigation/control runtime. Gateway failure, preview failure, or laptop
  disconnect must not terminate mission-critical perception, command
  arbitration, or robot behavior.

- Process-level containment: preview transport should run as an on-demand child
  process or equivalent supervised worker.

- Typed API: use gRPC for request/response, state queries, and event streaming.

- ROS stays internal: the gateway translates between external API calls and
  internal ROS topics/services/actions.

- Ownership: experiment recording and RunBundle review stay outside the gRPC
  gateway API.

## High-Level Shape

```text
                    [ Operator Laptop App ]
                               |
                    gRPC control / state API
                               |
                        [ Robot Gateway ]
                  +------------+-------------+
                  |            |             |
               preview      diagnostics    teleop
               manager       adapter       adapter
                  |            |             |
                  +------------+-------------+
                               |
                         internal ROS 2 API
                               |
                         [ ROS 2 graph ]
```

The implemented process model is:

- one process inside `robot-core`
- one gRPC server
- one ROS adapter layer
- one preview manager
- one bounded teleop manager

## Responsibilities

### External API boundary

Expose a versioned operator-facing API for:

- robot status
- preview enable/disable
- preview session status
- bounded teleop session control

### State aggregation

Collect and normalize data from the internal ROS graph:

- vision health/perf
- preview status
- teleop status
- platform diagnostics
- robot mode or mission state
- fault summaries

### Preview lifecycle control

Own the state machine for preview export:

- disabled
- starting
- running
- stopping
- faulted

This includes spawning and supervising the preview subprocess.

### Internal ROS adaptation

The gateway should speak ROS internally, not expose ROS externally.

Likely responsibilities:

- subscribe to selected internal topics
- call selected services/actions
- translate ROS status into gateway status
- publish internal commands requested by the operator app

## API Boundary

- browser-native delivery
- remote internet-facing access
- generic ROS graph proxying
- replacing local mission-critical ROS consumers

## Internal Modules

### gRPC server

Accepts operator requests and exposes state.

Likely responsibilities:

- request validation
- versioning
- streaming updates to the host app
- authn/authz hooks later if needed

### ROS adapter

Internal-only bridge between gateway logic and ROS resources.

Likely responsibilities:

- subscribe to `/vision/perf` and selected status topics
- call internal services
- isolate ROS-specific message names from the external API

### Preview manager

Owns the on-demand preview worker lifecycle.

Likely responsibilities:

- resolve preview profile to command/config
- launch subprocess
- monitor health
- return SRT endpoint metadata

### Teleop adapter

Likely responsibilities:

- explicit teleop command validation
- explicit enable/disable gating
- command stop on disable and gateway shutdown
- command arbitration

## Lifecycle

### Boot

At boot:

- `robot-core` starts
- gateway process starts with preview disabled
- no preview worker is running
- mission-critical ROS nodes operate without the gateway being on the hot path

### Preview enable flow

1. Operator app sends `SetPreviewMode`.
2. Gateway validates the request.
3. Gateway resolves a bounded preview profile.
4. Gateway spawns the preview worker.
5. Gateway reports preview state; the current client uses configured endpoint data.

### Preview disable flow

1. Operator app requests preview off.
2. Gateway terminates preview worker cleanly.
3. Gateway reports preview disabled.

## Failure Policy

- Gateway failure must not terminate mission-critical robot behavior.
- Preview worker failure must not terminate the gateway.
- External client disconnect must not affect the ROS runtime.
- Repeated preview failures should be visible through counters and state.

The gateway prefers fail-open behavior for the mission path and fail-closed behavior
for optional diagnostics.

## Observability

The gateway status surface includes:

- preview restart/fault counters
- current preview state
- selected transport/profile information

The gateway keeps status aggregation bounded and cache-backed.

## Rollout Status

**Implemented:**

- locked unary gRPC service and generated C++/Python code
- cache-backed system status from ROS vision, odometry, platform, preview, and teleop inputs
- bounded preview profiles and subprocess lifecycle management
- bounded teleop enable/disable plus bounded velocity commands
- CLI, monitor shell, Tk monitor, SRT preview helper, and overlay viewer

## Related Docs

- [Gateway API](gateway-api.md)
- [Preview streaming](preview-streaming.md)
