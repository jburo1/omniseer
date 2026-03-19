# Robot Gateway

This document describes the planned operator-facing gateway that will sit
between the robot's internal ROS 2 graph and the external operator laptop
application.

The gateway does **not** exist yet as a standalone implemented subsystem.
However, the architecture direction is now clear enough to document the shape
and build toward it deliberately.

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
- a draft remote monitoring architecture spec

What does **not** exist yet:

- a gRPC server on the robot
- an explicit external operator API
- a managed preview subprocess owned by a gateway component
- a host-side operator app bound to that API

Near-term direction:

- start with `robot-diag-control` inside `robot-core`
- let that component evolve into the first version of the robot gateway
- keep the first scope narrow: diagnostics and preview control

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

## Proposed High-Level Shape

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

## Suggested Internal Modules

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
5. Gateway reports preview state and stream endpoint.

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

## First Slice Candidates

1. Define the gRPC service and protobuf contract.
2. Implement a minimal gateway process with:
   - `GetSystemStatus`
   - `SetPreviewMode`
   - in-memory preview state only
3. Add subprocess management for the preview worker.
4. Add ROS adapter wiring once the API surface is stable enough.

## Related Docs

- [Remote monitoring architecture](remote_monitoring_architecture.md)
- [Gateway API](gateway_api.md)
- [Preview streaming](preview_streaming.md)
