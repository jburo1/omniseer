# Preview Streaming

This page describes the operator preview export path from the robot SBC to the
operator laptop.

The key design goal is to keep the preview path optional and low-impact while
still being useful for diagnosis, evaluation, and visual verification of the
vision stack.

## Purpose

Provide an on-demand, remotely controlled preview stream that:

- does not burden the mission-critical inference path when disabled
- uses a separate camera path where possible
- exports video to the laptop over Wi-Fi
- keeps higher-cost decode/visualization work offboard

## Current State

What is already known from the current ROCK 5B Plus target:

- `/dev/video11` is `rkisp_mainpath`
- `/dev/video12` is `rkisp_selfpath`
- both can stream concurrently at `1280x720 NV12 60 fps`
- neither path currently advertises direct `BGR24`/`RGB24`
- `mainpath` and `selfpath` are the right split for preview vs inference

Transport/runtime checks already completed on this SBC:

- GStreamer `v4l2src` can capture from `rkisp_mainpath`
- GStreamer SRT plugins are installed
- local `mainpath -> x264 -> MPEG-TS -> SRT -> decode` loopback has been
  validated
- the current C++ gateway can now launch that `x264 -> MPEG-TS -> SRT` path as
  its built-in preview worker
- the packaged Python host tools can now request preview over gRPC and consume
  that SRT stream using `gst-launch-1.0`

Current blocker for the intended production path:

- hardware H.265 encode is not currently exposed through the installed
  userspace stack on this image
- FFmpeg exposes `hevc_v4l2m2m`, but encode failed with `Could not find a valid
  device`
- no Rockchip-specific GStreamer H.265 encoder plugin appears to be installed

Implication:

- SRT transport is feasible now
- preview export is feasible now
- low-overhead production H.265 still needs encoder integration work

## Major Design Considerations

- Keep preview off by default.

- Use `rkisp_mainpath` for preview and `rkisp_selfpath` for inference by
  default.

- Keep preview in `NV12` into the encoder when possible; do not convert to BGR
  on the robot in the default path.

- Use SRT as the first preview transport over Wi-Fi.

- Decode and render on the host laptop.

- Reserve exact-frame debug paths for later targeted debugging, not the default
  operator preview flow.

## Proposed Default Pipeline

```text
rkisp_mainpath (/dev/video11, NV12)
            |
            v
     preview worker
            |
            +--> encode (target: H.265, current fallback: x264 for bring-up)
            |
            v
        MPEG-TS / SRT
            |
            v
     operator laptop decode
            |
            +--> monitor app video panel
            +--> optional local ROS Image bridge
            +--> optional host-side overlays / RViz2
```

## Why `mainpath` for Preview

The mission-critical vision pipeline already uses a producer/consumer hot path
on the SBC and is optimized around low-latency inference. The preview path
should not add avoidable work to that path.

Using `mainpath` for preview gives:

- better isolation from inference
- no need to add a same-frame conversion branch in the default case
- a natural on/off switch for operator diagnostics

The tradeoff is that preview frames and detection frames are close in time but
not guaranteed to be the exact same frame. That is acceptable for normal
operator monitoring.

## Transport Choice: SRT

The selected transport for the first implementation is SRT.

Why:

- better fit for variable Wi-Fi than plain RTP/UDP
- built-in recovery/latency tuning
- good support in the installed GStreamer stack
- simpler than WebRTC for the current scope

This is a diagnostics-oriented link, not a browser-delivery problem.

## Preview Profiles

The preview path should expose bounded named profiles, not raw encoder knobs.

Suggested initial profiles:

- `off`
- `low_bw`
- `balanced`
- `high_quality`

Candidate meanings:

- `off`: no preview worker running
- `low_bw`: e.g. 720p, reduced fps, modest bitrate
- `balanced`: 720p, moderate fps/bitrate
- `high_quality`: higher bitrate and possibly higher resolution if budget
  allows

The exact numbers can be tuned later after integration and Wi-Fi testing.

## Sync Model

### Default mode

- inference on `selfpath`
- preview on `mainpath`
- approximate temporal alignment

This is the recommended operator mode.

### Exact-sync mode

Later optional mode:

- preview derived from the same frame path as inference
- more expensive on the SBC
- only used when exact overlay verification matters

This should not be the default always-on path.

## Worker Lifecycle

The preview worker should be managed by the robot gateway or initial
`robot-diag-control` process.

Basic lifecycle:

1. preview disabled
2. operator requests preview
3. gateway spawns preview worker
4. worker binds SRT endpoint and begins export
5. operator disables preview
6. gateway terminates preview worker

The worker should be disposable. If it dies, the mission-critical runtime keeps
running.

## Current Encoder Options

### Desired production path

- `NV12 mainpath -> hardware H.265 encode -> SRT`

This is the desired low-overhead end state.

### Current implemented bring-up path

- `NV12 mainpath -> software x264 -> SRT`

This path is now the implemented robot-side preview export path for early
integration and proof-of-life, but it is not the desired long-term robot-side
budget.

### Currently unattractive path

- `NV12 mainpath -> software x265 -> SRT`

This is too CPU-expensive for the intended mission-critical budget on the SBC.

## Host-Side Expectations

The operator laptop is expected to:

- receive SRT
- decode the stream
- render it in the operator app
- optionally feed decoded frames into local analysis/overlay pipelines

Current implemented host slice:

- `robot_preview_viewer` can enable preview over gRPC and consume the fixed or
  configured SRT endpoint
- `robot_monitor_shell` can poll gateway status, run short watch loops, and
  launch `robot_preview_viewer` from one integrated tool
- `robot_monitor_gui` can refresh status, toggle preview, and launch
  `robot_preview_viewer` from a desktop GUI
- it currently shells out to `gst-launch-1.0`, so host-side GStreamer tooling
  is required
- the GUI currently launches preview in a separate helper process; it does not
  yet embed decoded video in its own window
- these are minimal bring-up tools, not the final operator UI

The architecture should **not** assume the laptop has Rockchip RGA. It should
only assume general-purpose CPU/GPU resources and likely hardware video decode.

## Open Integration Questions

- what exact hardware H.265 userspace path should be installed on the SBC image
- what preview profile defaults should be used over the current Wi-Fi setup
- whether decoded preview should be bridged into ROS on the host in the first
  UI slices or kept as a plain video panel initially

## Related Docs

- [Remote monitoring architecture](remote_monitoring_architecture.md)
- [Robot gateway](robot_gateway.md)
- [Gateway API](gateway_api.md)
