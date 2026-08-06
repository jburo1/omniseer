# Gateway API

_Status: locked unary v1 API implemented_

This page is the current control/status API reference for communication between
the operator laptop tools and the robot gateway. Preview video uses SRT and is
documented separately in [Preview streaming](preview-streaming.md).

## Current implementation

The implemented API uses:

- gRPC for control and state
- protocol buffers for the external contract
- SRT for preview video transport outside the gRPC service

The locked proto lives at
`ros_ws/src/robot_diag_control/robot_diag_control/api/robot_gateway.proto`.
Generated Python code is packaged with `robot_diag_control`, and generated C++
code is linked into `robot_diag_control_cpp`.

The C++ server implementation contains:

- one synchronous unary gRPC server
- one `RobotGatewayService`
- one cache-backed `GatewayStateStore`
- a ROS-backed node that feeds `/vision/perf`, filtered odometry, detections,
  and battery state into the store
- a platform sampler for compute, Wi-Fi, LiPo, and onboard battery diagnostics
- a parameter-driven preview subprocess manager behind `SetPreviewMode`
- a bounded teleop manager behind `SetTeleopEnabled` and `SendTeleopCommand`

The Python host tools currently exercise the same contract through:

- a CLI client
- a preview helper that consumes SRT with `gst-launch-1.0`
- an overlay viewer that combines SRT video with gateway overlay snapshots
- a monitor shell
- a Tk monitor GUI

## Current contract

The service contract is `omniseer.gateway.v1.RobotGateway`:

```proto
service RobotGateway {
  rpc GetSystemStatus(GetSystemStatusRequest) returns (SystemStatus);
  rpc SetPreviewMode(SetPreviewModeRequest) returns (SetPreviewModeResponse);
  rpc SetTeleopEnabled(SetTeleopEnabledRequest) returns (SetTeleopEnabledResponse);
  rpc SendTeleopCommand(SendTeleopCommandRequest) returns (SendTeleopCommandResponse);
  rpc GetOverlaySnapshot(GetOverlaySnapshotRequest) returns (OverlaySnapshot);
}
```

Versioning rules:

- evolve additively
- do not repurpose field numbers
- keep ROS resource names out of the protobuf contract
- use bounded enums for operator-facing modes and states

`GetSystemStatus` returns the latest cached `SystemStatus` snapshot. The
gateway does not perform a ROS round trip per status request.

`SystemStatus` contains:

- gateway name and version
- `PreviewStatus`
- `VisionStatus`
- `RobotHealth`
- `TeleopStatus`
- `PlatformStatus`

`PreviewStatus` contains:

- preview state
- preview profile
- last gateway-reported preview error

Preview states:

- `PREVIEW_STATE_UNSPECIFIED`
- `PREVIEW_DISABLED`
- `PREVIEW_RUNNING`

Preview profiles:

- `PREVIEW_PROFILE_UNSPECIFIED`
- `PREVIEW_PROFILE_LOW_BW`
- `PREVIEW_PROFILE_BALANCED`
- `PREVIEW_PROFILE_HIGH_QUALITY`

`SetPreviewMode` accepts:

- `enabled`
- `profile`

When `enabled=false`, the profile field is ignored. The response includes an
accepted flag, concise message, and post-request `PreviewStatus`.

`SetTeleopEnabled` accepts:

- `enabled`

Disabling teleop commands a stop on the internal command path. The response
includes an accepted flag, concise message, and post-request `TeleopStatus`.

`SendTeleopCommand` accepts one bounded velocity command:

- `linear_x_mps`
- `linear_y_mps`
- `angular_z_rad_s`

The gateway rejects commands when teleop is disabled or when any command value
is outside configured bounds.

`GetOverlaySnapshot` returns:

- current `SystemStatus`
- latest detection overlay snapshot from `/yolo/detections`
- source-image detection geometry for laptop-side scaling over decoded preview
  video
- gateway-local detection freshness
- a capped recent operator event list

`PlatformStatus` contains:

- `ComputeStatus`: CPU utilization, optional CPU temperature, optional thermal
  throttling, RAM usage, and optional disk usage
- `NetworkStatus`: selected Wi-Fi interface, connection state, optional RSSI,
  and optional link quality
- `PowerStatus`: LiPo battery from `/battery` and onboard battery from Linux
  `power_supply` data when present

Platform sub-statuses carry `available`, `stale`, and `age_ms` fields so the
operator can distinguish missing instrumentation from fresh healthy values.

Error handling contract:

- gRPC status codes report transport-level and request-level failures
- structured response fields report accepted runtime state
- invalid preview profiles are rejected
- starting an already-running preview profile returns idempotent success
- preview worker start failures are reported in the response message and preview
  status

The API boundary excludes:

- generic ROS graph browsing
- topic tunneling
- raw video transport over gRPC
- full robot command surfaces beyond bounded teleop
- browser-specific signaling flows
- RunBundle recording and review

## Verification

Supported local verification:

- `scripts/omni test ros`

Focused tests cover:

- protobuf-to-store service behavior
- synchronous gRPC server behavior
- gateway state snapshots and freshness flags
- preview command generation
- preview subprocess lifecycle behavior
- bounded teleop validation and stop behavior
- Python client formatting and request helpers
- preview, overlay, shell, and Tk monitor host tooling

The current host-side preview helper and monitor tooling have been smoke-tested
locally against the live C++ gateway and software x264/SRT preview worker.

## Limitations

- The API is unary only; there are no streaming RPCs.
- Multi-client ownership semantics are undefined.
- Stream endpoint metadata is not published by the protobuf API.
- The preview viewer uses configured endpoint data outside the gateway API.
- Platform diagnostics are operator diagnostics and do not currently change
  `RobotHealth.ready`.
- The API does not expose robot mode, mission state, wheel-level diagnostics,
  motor-controller diagnostics, build environment labels, or preview transport
  parameters.
- The gateway has no authn/authz layer.

## Future optimization

- Add stream endpoint metadata to `PreviewStatus` or a dedicated preview session
  message.
- Add streaming status or event RPCs.
- Add multi-client ownership rules and enforcement.
- Add robot mode and mission state once those values have stable robot-runtime
  semantics.
- Add wheel-level or motor-controller diagnostics through explicit status
  messages.
- Add preview dimensions, frame rate, bitrate, and transport metadata after
  those values are produced by the managed preview path.
- Add gateway authentication and authorization for non-local deployments.
