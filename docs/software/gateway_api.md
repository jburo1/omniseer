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
- a minimal protobuf definition in
  `ros_ws/src/robot_diag_control/robot_diag_control/api/robot_gateway.proto`
- Python client tools for exercising the gateway contract
- a C++ unary gRPC service/server layer in `robot_diag_control_cpp`
- a ROS-backed C++ node that serves the locked API and aggregates `/vision/perf`
- a parameter-driven preview subprocess manager that backs `SetPreviewMode`
- a built-in robot-side GStreamer preview command for `x264 -> MPEG-TS -> SRT`
- a minimal host-side preview helper that consumes that SRT stream via
  `gst-launch-1.0`
- a minimal host-side monitor shell that polls status and launches the preview
  helper
- a first Tk-based monitor GUI that refreshes status, toggles preview, and
  launches the preview helper as a separate process

This page now serves two roles:

- describe the intended long-term direction
- document the currently locked minimal v1 control/status contract

## Major Design Considerations

- Explicit API, not generic passthrough: the API should model robot operations,
  not mirror DDS details.

- Stable semantics: prefer bounded enums and named profiles over arbitrary
  strings and free-form encoder options.

- Versionability: protobuf/gRPC should allow additive evolution without
  reworking the operator app.

- Separation of concerns: control/state rides over gRPC; preview video rides
  over SRT and should not force extra transport detail into the first API slice.

- Operator safety: teleop-related operations later need stricter semantics than
  diagnostics operations.

## Initial Scope (v1)

The first API surface should stay narrow.

Recommended initial RPCs:

- `GetSystemStatus`
- `SetPreviewMode`

Optional early additions if they prove useful:

- `SubscribeEvents`
- `Ping`

Teleop and cloud-facing operations should wait until the diagnostics/control
plane is stable.

## High-Level API Shape

```text
Operator App  -- gRPC -->  Robot Gateway  -- ROS adapter --> internal ROS graph
                                     |
                                     +--> preview manager / subprocess
```

## Locked v1 Service Shape

The current `.proto` is now the contract we should carry into the C++ rewrite.
Keep it small and evolve additively.

```proto
service RobotGateway {
  rpc GetSystemStatus(GetSystemStatusRequest) returns (SystemStatus);
  rpc SetPreviewMode(SetPreviewModeRequest) returns (SetPreviewModeResponse);
}
```

## Current C++ Server Shape

The current implementation is intentionally narrow:

- one synchronous unary gRPC server in C++
- one in-memory `GatewayStateStore`
- one Python CLI for status/control
- one Python preview helper for host-side stream consumption
- one Python monitor shell for an integrated bring-up workflow
- one Python/Tk monitor GUI for the first desktop app slice
- one expected operator client at a time

The server intentionally avoids extra asynchronous machinery:

- no completion queues
- no streaming RPCs
- no separate gateway subprocess
- no ROS round-trip per request

Instead, status should be cache-backed:

- internal ROS subscriptions update the in-memory store
- `GetSystemStatus` returns the latest cached snapshot
- freshness is represented by the `VisionStatus.stale` flag

This is now implemented and locally verified against the packaged Python tools.

## Current Rollout Status

Completed:

- standalone protobuf-to-store service adapter
- standalone synchronous gRPC server wrapper
- ROS-backed C++ node wiring
- local end-to-end smoke path with the packaged Python CLI
- local host-side preview consumption with the packaged Python preview helper
- local shell-driven status/watch/preview flow with the packaged monitor shell
- local headless smoke of the packaged Tk GUI against the live C++ gateway

Still deferred:

- stream endpoint metadata in the API
- a hardware H.265 preview path
- broader status sources beyond the current vision/perf aggregation
- any streaming or multi-client behavior
- an embedded video panel inside the GUI

## Suggested Core Messages

### `SystemStatus`

Current contents:

- gateway name/version
- current robot health summary
- current preview state
- current normalized vision status summary

Deferred until later if needed:

- robot mode or mission state
- broader health summary
- build/runtime details beyond gateway version

### `PreviewStatus`

Current contents:

- current preview state
- active preview profile
- last error text if the gateway has one to report

Deferred until later if needed:

- transport kind
- stream URI or endpoint metadata
- restart count
- width / height / fps
- bitrate target or resolved bitrate

### `SetPreviewModeRequest`

Current fields:

- `bool enabled`
- `PreviewProfile profile`

### `SetPreviewModeResponse`

Current fields:

- `bool accepted`
- `string message`
- `PreviewStatus preview`

## Suggested Enums

The first version should use explicit enums rather than free-form strings where
possible.

Candidate enums:

- `PreviewState`
  - `PREVIEW_STATE_UNSPECIFIED`
  - `PREVIEW_DISABLED`
  - `PREVIEW_RUNNING`

- `PreviewProfile`
  - `PREVIEW_PROFILE_UNSPECIFIED`
  - `PREVIEW_PROFILE_LOW_BW`
  - `PREVIEW_PROFILE_BALANCED`
  - `PREVIEW_PROFILE_HIGH_QUALITY`

## Mapping to Internal Systems

The API should stay independent of internal ROS resource names, but the current
implementation direction is expected to map to internal resources roughly like
this:

- `GetSystemStatus`
  - aggregates internal health and diagnostics topics

- `SetPreviewMode`
  - drives a gateway-owned preview child process lifecycle

The gateway may use ROS topics and services internally, but that should not
leak into the protobuf contract.

## Error Handling

Initial rules:

- use gRPC status codes for transport-level/request-level failures
- use structured response fields for accepted-but-faulted runtime state
- keep operator-visible messages concise and action-oriented

Examples:

- invalid preview profile -> request rejected
- preview worker failed to start -> request accepted, response includes an operator-facing message
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
2. Implement a standalone C++ gRPC service/server layer with:
   - `GetSystemStatus`
   - `SetPreviewMode`
3. Wire that server into the ROS-backed C++ node.
4. Back `SetPreviewMode` with a real preview subprocess command.
5. Add streaming events after the basic request/response path is stable.

Items 1-4 are now complete for the current bring-up path, and the repo now has
a minimal host-side preview helper that consumes the stream using a configured
SRT endpoint. The gateway still does not publish endpoint metadata through the
protobuf API.

## Related Docs

- [Robot gateway](robot_gateway.md)
- [Preview streaming](preview_streaming.md)
- [Remote monitoring architecture](remote_monitoring_architecture.md)
