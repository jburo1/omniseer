# Remote Monitoring Architecture Spec

_Status: draft v0_

_Last updated: 2026-03-19_

## 1) Purpose

Define a concrete architecture for:

- mission-critical runtime on the robot SBC
- optional operator-controlled preview streaming to a laptop
- off-robot monitoring, visualization, recording, and analysis

The key constraint is that the SBC should spend nearly all steady-state compute on robot-critical work. Preview and diagnostics should be optional, explicitly budgeted, and easy to disable.

## 2) Goals

1. Keep autonomy and perception functional even when diagnostics are disabled, disconnected, or broken.
2. Make preview streaming opt-in and remotely controllable from the laptop.
3. Keep ROS 2 internal to the robot runtime and use explicit external protocols for operator connectivity.
4. Run RViz2, overlays, plots, bagging, and higher-cost analysis on the host.
5. Preserve a path to a more isolated split deployment later without reworking the whole system.

## 3) Non-Goals (v0)

- full cloud telemetry platform
- browser-first remote operations stack
- exact pixel-perfect overlay sync in the always-on path
- replacing the mission-critical local consumers of detections

## 4) Current Hardware / Runtime Facts

Observed on the current ROCK 5B Plus target:

- `/dev/video11` is `rkisp_mainpath`
- `/dev/video12` is `rkisp_selfpath`
- both paths can stream concurrently at `1280x720 NV12 60 fps`
- neither path currently advertises `BGR24`/`RGB24`
- `selfpath` exposes YUV-family formats plus `GREY` and `RGB565`

Implications:

- onboard inference should continue using `selfpath`
- preview should preferentially use `mainpath`
- color conversion should happen in the preview stack only when preview is enabled
- raw `bgr8` should not be the default transport off the robot

Also measured locally on this board:

- current inference preprocess `NV12 1280x720 -> RGB888 640x640` costs about `0.96 ms/frame`
- an additional full-resolution `NV12 1280x720 -> BGR888 1280x720` RGA conversion costs about `0.64 ms/frame`

This means a same-frame in-pipeline preview branch is viable for targeted debug, but should not be the default always-on remote viewing path.

## 5) Top-Level Architecture

The system is split into two planes:

### 5.1 Mission-Critical Plane

Runs on the SBC and stays up regardless of diagnostics state.

Responsibilities:

- camera capture for inference
- preprocess
- inference
- detection publication
- local behavior/autonomy consumers
- local health and perf reporting

### 5.2 Diagnostic Plane

Spans robot and host, and is explicitly optional.

Responsibilities:

- preview stream enable/disable
- off-robot visualization
- recording and evaluation
- operator metrics panels
- optional host-side tracking / overlays / RViz2

## 6) Recommended Initial Deployment

### 6.1 Robot Side

Run **one container** initially:

- `robot-core`

Inside `robot-core`:

- mission-critical ROS 2 nodes
- `robot-diag-control`
- an on-demand preview streamer subprocess

Do **not** start with two robot containers by default.

Reason:

- the compute overhead of a second container is usually small
- the operational complexity is not small
- device passthrough, lifecycle, logs, startup ordering, and debugging all get harder
- the preview stack is still evolving, so start with one deployment boundary and split later only if it pays for itself

### 6.2 Host Side

Run **native applications** first, not containers.

Recommended host components:

- native operator application or helper process
- RViz2 native
- optional local host-side decode/analysis helpers

Reason:

- RViz2 and desktop video decode are simplest natively
- host GUI/container audio/video/display plumbing is usually friction-heavy
- the laptop is not compute-constrained the way the SBC is

## 7) Robot Container vs Two-Container Split

### 7.1 Initial Recommendation: One Container, Two Process Groups

Use:

- `robot-core` container
- `robot-diag-control` node inside it
- preview streamer launched on demand as a child process or supervised sibling process

Advantages:

- simplest boot and deployment story
- one ROS environment
- one filesystem namespace
- easy access to `/dev/video11` and `/dev/video12`
- no extra container orchestration needed
- lowest integration risk for early slices

Disadvantages:

- preview faults are less isolated than with a fully separate container
- logs and lifecycle management need discipline

### 7.2 Later Option: Two Robot Containers

Possible later split:

- `robot-core`
- `robot-video`

Use this only after the preview stack is stable and useful enough to justify stronger isolation.

Advantages:

- independent restart policy for the streamer
- tighter resource accounting and policy
- easier security/network hardening if preview becomes externally reachable

Disadvantages:

- more startup and lifecycle complexity
- device passthrough needs explicit management
- more networking/configuration surface
- more operational overhead for development and debugging

### 7.3 Overhead Assessment

For containers themselves:

- CPU overhead of an extra container is usually negligible
- memory overhead is mostly one more process tree plus duplicated runtime libs and buffers
- the bigger cost is operational complexity, not raw compute

For the preview stack:

- the meaningful budget item is the preview pipeline itself: capture, optional conversion, encode, socket I/O
- that cost exists whether the streamer runs as a process in `robot-core` or in a separate `robot-video` container

Conclusion:

- start as one container with a managed preview subprocess
- split later only for fault isolation or deployment hygiene, not because containers themselves are too expensive

## 8) Robot-Side Components

### 8.1 `robot-core`

Responsibilities:

- inference path on `/dev/video12`
- `/yolo/detections`
- `/vision/perf`
- local consumers of detections
- `robot-diag-control`

### 8.2 `robot-diag-control`

Lives inside `robot-core` initially.

Responsibilities:

- expose preview control over ROS 2
- own preview state machine
- start/stop preview streamer subprocess
- publish diagnostic status to the rest of the graph
- keep preview failure isolated from mission-critical nodes

Recommended outputs:

- `/diag/status`
- `/diag/preview_status`

Recommended services/actions:

- `/diag/set_preview_mode`
- `/diag/get_diag_status`

### 8.3 Preview Streamer Subprocess

Responsibilities:

- own `/dev/video11`
- capture `NV12`
- hardware-encode H.265
- serve stream to the laptop

Recommended default behavior:

- off by default
- started only on operator request
- bounded profiles only, not arbitrary encoder flags from the UI

## 9) Host-Side Components

### 9.1 Operator App / Host Helper

Native host application or helper process.

Responsibilities:

- talk to the robot over gRPC
- open/close the preview stream
- subscribe to state/events from the robot gateway
- optionally decode the stream into local ROS topics for RViz2 or host-side overlay nodes

### 9.2 UI / Monitoring App

Keep the UI separate from ROS internals where possible.

Responsibilities:

- preview toggle
- preview state display
- metrics dashboards
- event/fault display
- record/export controls

The UI can talk directly to the robot gateway over gRPC or sit on top of a thin
local helper process.

### 9.3 RViz2 / Analysis Tools

Run natively on the host.

Examples:

- RViz2
- `rqt_plot`
- `rosbag2`
- host-side `tracking_node`
- host-side `debug_node`

## 10) Control and Data Boundaries

### 10.1 ROS 2 Is Used For

Inside the robot, ROS 2 is used for:

- status
- metrics
- detections
- event/state publication

Examples:

- `/yolo/detections`
- `/vision/perf`
- `/diag/*`

### 10.2 Video Transport Is Separate

Use a dedicated video transport for preview.

Recommended first transport:

- SRT over local Wi-Fi / LAN

Later options:

- RTSP if a camera-like operator UX becomes more important later
- WebRTC only if browser/native remote ops requirements justify the added complexity

Reason:

- DDS is a poor default place for always-on compressed operator video
- dedicated video transport gives tighter control over bitrate, latency, and startup behavior

### 10.3 Preview Transport Options

The preview path should keep the robot-side data plane simple:

- `/dev/video11` mainpath captures `NV12`
- encoder consumes `NV12` directly where possible
- robot sends compressed H.265 over the network
- host decodes and performs any color conversion, scaling, or overlay work locally

This avoids paying for a robot-side `NV12 -> BGR` conversion in the default preview path.

Recommended interpretation of the main transport options:

- `RTSP`: session/control protocol commonly used to expose a camera-like stream
- `SRT`: secure reliable transport designed for lossy or variable networks
- `WebRTC`: interactive media stack with NAT traversal and adaptive control, but much higher implementation complexity

For this project:

- use SRT first for the initial operator preview path
- consider RTSP later if a camera-like session model or off-the-shelf tooling becomes more important than transport resilience
- defer WebRTC unless a browser-facing or internet-routable operator UI becomes a real requirement

### 10.4 Transport Tradeoff Matrix

#### RTSP

Strengths:

- easy mental model and broad tooling support
- simple to open from VLC, ffplay, GStreamer, and many desktop apps
- good default fit for a local operator preview stream

Weaknesses:

- RTSP itself is only the control layer; media is typically carried separately over RTP
- behavior over weak Wi-Fi depends heavily on whether media is carried over UDP or TCP

Typical modes:

- `RTSP/RTP/UDP`: lower latency, but packet loss shows up as artifacts or drops
- `RTSP interleaved over TCP`: simpler and more firewall-friendly, but retransmissions can increase jitter and stall behavior on bad Wi-Fi

#### SRT

Strengths:

- designed for unreliable links
- supports packet recovery, encryption, and configurable latency budget
- often behaves better than plain RTP/UDP on noisy Wi-Fi

Weaknesses:

- fewer "camera app" style clients than RTSP
- slightly more specialized tooling and operational knowledge
- usually trades a bit more latency for better resilience

Typical use:

- best when "keep the stream usable" matters more than shaving every millisecond

#### WebRTC

Strengths:

- very good for browser delivery and interactive remote control surfaces
- adaptive and NAT-friendly

Weaknesses:

- much more signaling and integration complexity
- overkill for the first monitoring slices

## 11) Preview Modes

Define a small set of stable profiles instead of exposing raw encoder knobs:

- `off`
- `low_bw`
- `balanced`
- `high_quality`
- `exact_sync_debug` (later, optional, not default)

Suggested semantics:

- `off`: no preview capture or encode running
- `low_bw`: 720p, low fps, low bitrate
- `balanced`: 720p, moderate fps/bitrate
- `high_quality`: higher bitrate and possibly higher resolution if budget allows
- `exact_sync_debug`: special mode that may use a same-frame preview path instead of the normal secondary ISP path

## 12) Sync Model

Two useful operating modes exist:

### 12.1 Default Diagnostic Mode

- inference on `selfpath`
- preview on `mainpath`
- detections and video are close in time but not guaranteed to be the same exact frame

Use for:

- operator monitoring
- general evaluation
- performance-safe remote diagnosis

### 12.2 Exact-Sync Debug Mode

- preview derived from the same frame path used for inference
- more expensive
- used only for targeted debugging when exact overlay alignment matters

Use for:

- validating tracking edge cases
- detailed regression capture

## 13) Boot / Lifecycle Model

### 13.1 Robot Boot

At boot:

- launch `robot-core`
- do not launch preview streamer
- `robot-diag-control` reports preview as disabled

### 13.2 Preview Enable Flow

1. Host discovers robot over ROS 2.
2. Operator requests preview.
3. Host calls `/diag/set_preview_mode`.
4. `robot-diag-control` validates request and starts preview streamer.
5. Robot publishes preview status and endpoint metadata.
6. Host opens stream.

### 13.3 Preview Disable Flow

1. Host requests preview off.
2. `robot-diag-control` stops the preview streamer.
3. Robot publishes preview disabled state.
4. Host tears down local decode/overlay path.

### 13.4 On-Demand Subprocess Model

The preview streamer should initially be a managed child process of `robot-diag-control`.

Responsibilities of `robot-diag-control`:

- maintain a single preview state machine
- translate preview profiles into a fixed command line or config blob
- start the child process
- monitor child liveness
- expose status and faults over ROS 2
- stop the child cleanly on operator request or node shutdown

Suggested state machine:

- `disabled`
- `starting`
- `running`
- `stopping`
- `faulted`

Suggested start behavior:

1. Validate no preview child is already active.
2. Resolve the requested profile into width, height, fps, bitrate, and transport.
3. Spawn the preview child process with stdout/stderr captured to logs.
4. Wait for a bounded startup window.
5. Publish `running` with the resolved stream URI if startup succeeds.
6. Publish `faulted` if the child exits early or fails health checks.

Suggested stop behavior:

1. Mark state as `stopping`.
2. Send `SIGTERM` to the child process.
3. Wait a short timeout for clean shutdown.
4. Escalate to `SIGKILL` only if needed.
5. Publish `disabled` after cleanup completes.

Failure containment requirements:

- preview child exit must not restart or block mission-critical ROS nodes
- repeated preview faults should increment a restart/error counter
- the system must tolerate the operator rapidly toggling preview on/off

### 13.5 Preview Subprocess Inputs and Outputs

Initial subprocess inputs:

- capture device path
- selected preview profile
- transport kind
- bind address / port

Initial subprocess outputs:

- process exit code
- stream URI
- negotiated resolution and fps
- encoder/transport error text
- optional periodic bitrate/frame counters

## 14) Recommended Initial Interfaces

### 14.1 ROS 2 Service: `SetPreviewMode`

Initial request fields:

- `bool enabled`
- `string profile`

Initial response fields:

- `bool accepted`
- `string state`
- `string message`
- `string stream_uri`

### 14.2 ROS 2 Topic: `PreviewStatus`

Suggested fields:

- current state
- current profile
- stream URI
- width
- height
- fps
- bitrate
- error text
- restart count

## 15) Important Decisions Answered

### Q1. Should the robot run two containers from the start?

Recommended answer:

- no

Start with one `robot-core` container and launch the preview streamer as a managed subprocess from `robot-diag-control`.

### Q2. Does splitting into two robot containers materially reduce compute load?

Recommended answer:

- not in any meaningful way

The preview pipeline cost is in capture/encode/network I/O. The container boundary itself is not the expensive part.

### Q3. Why still keep the two-container option?

Recommended answer:

- for later fault isolation and operational hygiene

Use it only if the preview stack becomes stable enough that independent restart, policy, or packaging actually matters.

### Q4. Should the host side use containers initially?

Recommended answer:

- no

Use native applications first. Keep RViz2, decode, and monitoring native on the laptop.

### Q5. How should the host app connect to the robot?

Recommended answer:

- gRPC for control/state
- SRT for preview video

### Q6. Where should tracking/overlays run?

Recommended answer:

- on the host by default

That keeps the SBC focused on mission-critical work.

### Q7. Should the robot convert preview frames to BGR before sending them?

Recommended answer:

- no, not in the default preview path

Keep the preview path in `NV12` into the encoder and send compressed H.265. Perform decode, colorspace conversion, scaling, and overlay work on the host.

### Q8. Should the host be expected to have Rockchip RGA?

Recommended answer:

- no

The host laptop will usually not have Rockchip RGA. What it usually has is sufficient CPU/GPU and often hardware video decode support. The architecture should assume only that the host can decode H.265 and render overlays locally.

### Q9. What does "on-demand subprocess" mean concretely?

Recommended answer:

- `robot-diag-control` launches and supervises the preview pipeline only while preview is enabled

This means preview has a strict lifecycle and no steady-state cost when disabled.

## 16) Recommended First Implementation Slices

1. Add `robot-diag-control` inside `robot-core` with preview state reporting and a stubbed preview state machine.
2. Add a narrow gRPC surface for preview status/control that can later grow into the robot gateway.
3. Add a managed preview streamer subprocess controlled by `robot-diag-control`, using `/dev/video11`.
4. Add host-side RViz2 / analysis integration.
5. Only after that, decide whether a separate `robot-video` container or an exact-sync debug mode is worth the added complexity.

## 17) Open Questions

1. What hardware H.265 userspace path should be installed or enabled on the SBC image.
2. Whether the first preview slice should use software `x264` as a bring-up path or wait for hardware H.265 integration.
3. Whether the first host-side overlay path should decode preview into ROS `Image` locally, or keep video display separate from ROS overlays until later.
