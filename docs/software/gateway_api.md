# Gateway API

This document describes the intended external API contract between the operator
laptop application and the robot gateway.

The selected direction for the first implementation is:

- `gRPC` for control and state
- `SRT` for preview video transport

This page focuses on the control/state plane only.

## Purpose

Define a typed, explicit, versionable API that:

- exposes operator-relevant robot state
- controls optional diagnostics such as preview streaming
- avoids exposing the internal ROS graph directly
- can evolve without forcing the UI to know ROS topic or service names

## Current State

What exists today:

- ROS 2 internal topics and services
- a monitoring architecture draft
- no protobuf definitions yet
- no gRPC server yet

So this page is still a design/spec document, not an implementation reference.

## Major Design Considerations

- Explicit API, not generic passthrough: the API should model robot operations,
  not mirror DDS details.

- Stable semantics: prefer bounded enums and named profiles over arbitrary
  strings and free-form encoder options.

- Versionability: protobuf/

gRPC should allow additive evolution without
  reworking the operator app.

- Separation of concerns: control/state rides over gRPC; preview video rides
  over SRT and is only referenced by endpoint metadata in the API.

- Operator safety: teleop-related operations later need stricter semantics than
  diagnostics operations.

## Initial Scope (v1)

The first API surface should stay narrow.

Recommended initial RPCs:

- `GetSystemStatus`
- `GetPreviewStatus`
- `SetPreviewMode`
- `SubscribeEvents`

Optional early additions if they prove useful:

- `Ping`
- `GetBuildInfo`

Teleop and cloud-facing operations should wait until the diagnostics/control
plane is stable.

## High-Level API Shape

```text
Operator App  -- gRPC -->  Robot Gateway  -- ROS adapter --> internal ROS graph
                                     |
                                     +--> preview manager / subprocess
```

## Suggested v1 Service Sketch

This is not final protobuf syntax, but it captures the intended shape.

```proto
service RobotGateway {
  rpc GetSystemStatus(GetSystemStatusRequest) returns (SystemStatus);
  rpc GetPreviewStatus(GetPreviewStatusRequest) returns (PreviewStatus);
  rpc SetPreviewMode(SetPreviewModeRequest) returns (SetPreviewModeResponse);
  rpc SubscribeEvents(SubscribeEventsRequest) returns (stream GatewayEvent);
}
```

## Suggested Core Messages

### `SystemStatus`

Intended contents:

- robot mode or high-level runtime state
- health summary
- current preview state
- selected software/build identifiers
- key diagnostics summaries

### `PreviewStatus`

Intended contents:

- current preview state
- active preview profile
- transport kind
- stream URI or endpoint metadata
- width / height / fps
- bitrate target or resolved bitrate
- last error text if faulted
- restart count

### `SetPreviewModeRequest`

Suggested fields:

- `bool enabled`
- `PreviewProfile profile`

### `SetPreviewModeResponse`

Suggested fields:

- `bool accepted`
- `PreviewState state`
- `string message`
- `string stream_uri`

### `GatewayEvent`

Intended for push-style updates without polling.

Candidate categories:

- preview state change
- fault raised/cleared
- vision health update
- build/runtime info update

## Suggested Enums

The first version should use explicit enums rather than free-form strings where
possible.

Candidate enums:

- `PreviewState`
  - `PREVIEW_STATE_UNSPECIFIED`
  - `PREVIEW_DISABLED`
  - `PREVIEW_STARTING`
  - `PREVIEW_RUNNING`
  - `PREVIEW_STOPPING`
  - `PREVIEW_FAULTED`

- `PreviewProfile`
  - `PREVIEW_PROFILE_UNSPECIFIED`
  - `PREVIEW_PROFILE_LOW_BW`
  - `PREVIEW_PROFILE_BALANCED`
  - `PREVIEW_PROFILE_HIGH_QUALITY`

- `TransportKind`
  - `TRANSPORT_KIND_UNSPECIFIED`
  - `TRANSPORT_KIND_SRT`

## Mapping to Internal Systems

The API should stay independent of internal ROS resource names, but the current
implementation direction is expected to map to internal resources roughly like
this:

- `GetSystemStatus`
  - aggregates internal health and diagnostics topics

- `GetPreviewStatus`
  - reads gateway-owned preview state

- `SetPreviewMode`
  - drives the preview worker lifecycle

- `SubscribeEvents`
  - streams gateway-owned normalized state changes

The gateway may use ROS topics and services internally, but that should not
leak into the protobuf contract.

## Error Handling

Initial rules:

- use gRPC status codes for transport-level/request-level failures
- use structured response fields for accepted-but-faulted runtime state
- keep operator-visible messages concise and action-oriented

Examples:

- invalid preview profile -> request rejected
- preview worker failed to start -> request accepted, state returns `FAULTED`
- stream already running with same profile -> idempotent success

## Versioning Strategy

The API should be easy to extend over time.

Recommended rules:

- add fields, do not repurpose fields
- prefer enums over overloaded strings
- keep v1 small enough that migration churn stays low
- avoid leaking implementation-specific command lines or ROS resource names

## Non-Goals (v1)

- generic ROS graph browsing
- topic tunneling
- raw video transport in gRPC
- full teleop command set
- browser-specific signaling flows

## First Slice Candidates

1. Create the protobuf package and generate code for the chosen languages.
2. Implement a stub gRPC server with:
   - `GetSystemStatus`
   - `GetPreviewStatus`
   - `SetPreviewMode`
3. Back `SetPreviewMode` with an in-memory state machine before wiring the real
   preview subprocess.
4. Add streaming events after the basic request/response path is stable.

## Related Docs

- [Robot gateway](robot_gateway.md)
- [Preview streaming](preview_streaming.md)
- [Remote monitoring architecture](remote_monitoring_architecture.md)
