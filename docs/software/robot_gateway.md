# Robot Gateway

_Status: implemented v1 diagnostics and preview-control slice_

This document describes the operator-facing gateway between the robot's internal
ROS 2 graph and the external operator laptop application. The first C++ gateway,
typed gRPC API, preview lifecycle manager, and Python operator tools are implemented.
Teleop, experiment export control, and cloud integration remain planned.

## Purpose

Provide one explicit integration boundary for operator-facing features:

- gRPC control and state access
- preview session management
- later teleop control
- later recording/export triggers
- later cloud-bridge integration

The gateway keeps the internal ROS 2 graph private and prevents the laptop app
from becoming a generic DDS client that is tightly coupled to internal topics
and services.

## Current State

What exists today:

- ROS 2 graph on the SBC
- native vision runtime
- `omniseer_vision_bridge` publishing detections into ROS 2
- RViz and debug tooling on the ROS side
- a remote monitoring architecture spec
- a `robot_diag_control_cpp` package with:
  - a shared generated C++ gRPC/protobuf library from the locked `.proto`
  - an in-memory gateway state store
  - a synchronous unary gRPC service/server layer
  - a ROS-backed node that aggregates `/vision/perf`, filtered odometry, and serves the locked API
  - a gateway-owned preview subprocess manager backing `SetPreviewMode`
  - a built-in GStreamer preview worker path using `/dev/video11 -> x264 -> MPEG-TS -> SRT`
  - packaged Python client tools for gateway status/control and host-side preview consumption
  - a packaged Python monitor shell that integrates status polling and preview launch
  - a first packaged Tk monitor GUI for desktop status/control bring-up
  - local verification against those packaged Python tools

What does **not** exist yet:

- stream endpoint metadata in the API
- the intended low-overhead hardware H.265 preview path
- an embedded preview panel inside the host-side GUI
- experiment recording or cloud synchronization RPCs

Near-term direction:

- keep `robot_diag_control_cpp` inside `robot-core`
- use the gateway for optional status and preview control
- add experiment export control only after the local recorder contract is stable

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
3. replace the original stubbed preview toggle with a gateway-owned subprocess lifecycle
4. wire the first real preview export command into that lifecycle

This keeps the control/status boundary small while avoiding premature async
gRPC complexity.

The next portfolio-facing slice is experiment observability and review integration,
not more gateway transport abstraction.

## Major Design Considerations

- Narrow external contract: expose stable RPCs, not internal ROS topic names.

- Mission isolation: preview, UI, and diagnostics must not interfere with
  navigation/control runtime.

- Process-level containment: preview transport should run as an on-demand child
  process or equivalent supervised worker.

- Typed API: use gRPC for request/response, state queries, and event streaming.

- ROS stays internal: the gateway translates between external API calls and
  internal ROS topics/services/actions.

- Growth path: the same gateway boundary should later support teleop, logging
  triggers, and possibly a cloud bridge without needing to expose the full ROS
  graph.

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

The first implementation can be simpler than this diagram:

- one process inside `robot-core`
- one gRPC server
- one ROS adapter layer
- one preview manager

Teleop can remain out of scope for the first slices.

## Responsibilities

### External API boundary

Expose a versioned operator-facing API for:

- robot status
- preview enable/disable
- preview session status
- later recording/export control
- later teleop session control

### State aggregation

Collect and normalize data from the internal ROS graph:

- vision health/perf
- preview status
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

## Non-Goals (v1)

- full teleop implementation
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

Later addition.

Likely responsibilities:

- explicit teleop command validation
- heartbeat/deadman enforcement
- rate limiting
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

The first implementation should prefer fail-open behavior for the mission path
and fail-closed behavior for optional diagnostics.

## Observability

The gateway should eventually emit:

- request counters
- preview session counters
- preview restart/fault counters
- current preview state
- current connected client count
- selected transport/profile information

This does not need a full metrics system on day one, but the component should
be structured so that basic telemetry is easy to add.

## Rollout Status

**Implemented:**

- locked unary gRPC service and generated C++/Python code
- cache-backed system status from ROS vision and odometry inputs
- bounded preview profiles and subprocess lifecycle management
- CLI, monitor shell, Tk monitor, and SRT preview helper

**Planned:**

- experiment recording/export control after the recorder contract is proven
- stream endpoint metadata and an embedded preview panel
- hardware H.265 encode
- any cloud-facing gateway behavior

## Related Docs

- [Remote monitoring architecture](remote_monitoring_architecture.md)
- [Gateway API](gateway_api.md)
- [Preview streaming](preview_streaming.md)
