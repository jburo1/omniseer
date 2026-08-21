# Robot Gateway

This page is the implementation reference for the operator-facing gateway between
the robot's internal ROS 2 graph and the external operator laptop tools.

## Current implementation

The gateway is implemented by `robot_diag_control_cpp` inside the `robot-core`
runtime. It keeps the internal ROS 2 graph private and exposes a narrow external
gRPC boundary for optional operator status, preview control, overlays, and
bounded teleop.

The current process contains:

- a shared generated C++ gRPC/protobuf library from the locked
  `omniseer.gateway.v1` proto
- one synchronous unary gRPC server
- one in-memory `GatewayStateStore`
- one ROS-backed gateway node
- ROS subscriptions for `/vision/perf`, filtered odometry, detections, and
  battery state
- a platform sampler for compute, Wi-Fi, LiPo, and onboard battery diagnostics
- a gateway-owned preview subprocess manager
- a bounded teleop manager
- a built-in GStreamer preview worker command using
  `/dev/video11 -> x264 -> MPEG-TS -> SRT`

The packaged laptop-side tools are implemented in `robot_diag_control`:

- `robot_gateway_client` for status, preview control, overlay snapshots, and
  teleop RPCs
- `robot_preview_viewer` for host-side SRT consumption
- `robot_overlay_viewer` for host-side preview plus gateway overlay snapshots
- `robot_monitor_shell` for status polling and preview bring-up
- `robot_monitor_gui` for the first Tk desktop status, preview, and teleop
  surface

The implemented process model is deliberately small:

- one gateway process inside `robot-core`
- one gRPC server
- one ROS adapter layer
- one preview manager
- one bounded teleop manager
- one expected operator client at a time

## Current contract

The gateway contract is the `omniseer.gateway.v1.RobotGateway` protobuf service
defined in
`ros_ws/src/robot_diag_control/robot_diag_control/api/robot_gateway.proto`.

Implemented unary RPCs:

- `GetSystemStatus`
- `SetPreviewMode`
- `SetTeleopEnabled`
- `SendTeleopCommand`
- `GetOverlaySnapshot`

The external contract exposes robot operations and normalized status, not ROS
resource names. ROS topics, services, actions, and message types remain internal
to the gateway process.

The gateway owns these current responsibilities:

- normalize cached system status from ROS and platform inputs
- report gateway name and version
- report preview, vision, odometry, teleop, and platform status
- distinguish the last gateway teleop request from the effective post-mux drive
  command and odometry velocity
- enable and disable preview through the gateway-owned subprocess lifecycle
- reject invalid preview profiles
- enable and disable bounded teleop
- reject teleop commands while teleop is disabled or outside configured bounds
- command a stop when teleop is disabled or the teleop manager shuts down
- report latest detection overlay data and recent operator events

The gateway intentionally does not provide:

- generic ROS graph browsing
- topic tunneling
- raw video transport over gRPC
- RunBundle recording or review
- browser-native preview delivery
- remote internet-facing access
- mission-critical command arbitration

RunBundle recording and review are owned by launch profiles and
`scripts/omni runs`, outside the gRPC API.

## Verification

Supported local verification:

- `scripts/omni test ros`

Focused package tests cover the gateway state store, synchronous gRPC server,
service adapter, preview command factory, preview process manager, teleop
manager, Python gateway client formatting, preview viewer helpers, overlay
viewer helpers, monitor shell behavior, and Tk monitor smoke coverage.

Preview transport proof-of-life has been locally validated with the packaged
Python tools against the live C++ gateway and the software x264/SRT worker path.

Hardware behavior still requires target-side verification when a change depends
on ROCK 5B Plus camera devices, GStreamer plugins, Wi-Fi conditions, LiPo or
onboard battery sources, an active ROS graph, or physical robot command paths.

## Limitations

- Preview video uses the current software x264/MPEG-TS/SRT path.
- The gateway API does not publish stream endpoint metadata.
- The gRPC service is synchronous and unary.
- Streaming RPCs and multi-client semantics are not implemented.
- The Tk GUI launches preview in a separate helper process; decoded video is not
  embedded in the GUI.
- There is no gateway authn/authz layer.
- Platform diagnostics are operator diagnostics; they do not currently alter
  `RobotHealth.ready`.
- Preview and gateway failures are isolated from the mission path, but the
  gateway is not a substitute for mission-critical ROS consumers.
