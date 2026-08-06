# Preview Streaming

_Status: software x264/SRT bring-up path implemented_

This page is the current implementation reference for the optional operator
preview export path from the robot SBC to the operator laptop.

## Current implementation

The preview path is managed by the `robot_diag_control_cpp` gateway process. It
is off by default and runs as a gateway-owned worker process only after an
operator calls `SetPreviewMode`.

Current ROCK 5B Plus camera facts:

- `/dev/video11` is `rkisp_mainpath`
- `/dev/video12` is `rkisp_selfpath`
- both can stream concurrently at `1280x720 NV12 60 fps`
- neither path currently advertises direct `BGR24` or `RGB24`
- `mainpath` and `selfpath` provide the current preview/inference split

Implemented robot-side preview worker:

```text
rkisp_mainpath (/dev/video11, NV12)
            |
            v
     GStreamer preview worker
            |
            v
     software x264 encode
            |
            v
        MPEG-TS / SRT
```

Implemented host-side consumers:

- `robot_preview_viewer` enables preview over gRPC and consumes the configured
  SRT endpoint with `gst-launch-1.0`
- `robot_overlay_viewer` enables preview, decodes SRT into an OpenCV window,
  polls gateway overlay snapshots, and draws detections plus telemetry over the
  live video
- `robot_monitor_shell` polls gateway status, runs short watch loops, and
  launches `robot_preview_viewer`
- `robot_monitor_gui` refreshes status, toggles preview, and launches
  `robot_preview_viewer` from a desktop GUI

Transport/runtime checks completed on the target SBC:

- GStreamer `v4l2src` captures from `rkisp_mainpath`
- GStreamer SRT plugins are installed
- local `mainpath -> x264 -> MPEG-TS -> SRT -> decode` loopback has been
  validated
- the C++ gateway launches the `x264 -> MPEG-TS -> SRT` path as its built-in
  preview worker
- packaged Python host tools request preview over gRPC and consume the SRT
  stream

## Current contract

Preview control is part of the gateway gRPC API:

- `SetPreviewMode(enabled=true, profile=...)` starts the worker
- `SetPreviewMode(enabled=false)` terminates the worker
- `GetSystemStatus` reports `PreviewStatus`
- `GetOverlaySnapshot` returns source-space detection geometry for host-side
  overlay scaling

Implemented preview profiles:

- `off`: no preview worker running
- `low_bw`: 720p, 15 fps, 1000 kbit/s
- `balanced`: 720p, 30 fps, 2500 kbit/s
- `high_quality`: 720p, 60 fps, 4500 kbit/s

Preview lifecycle:

1. preview disabled
2. operator requests preview
3. gateway validates the requested profile
4. gateway resolves the profile to a bounded worker command
5. gateway spawns the worker
6. worker binds the SRT endpoint and exports video
7. operator disables preview
8. gateway terminates the worker

Operational contract:

- preview is optional and off by default
- preview startup, shutdown, and failure do not terminate mission-critical
  inference or robot behavior
- preview uses `rkisp_mainpath` by default
- inference uses `rkisp_selfpath` by default
- host tooling requires general-purpose decode/render capability, not Rockchip
  RGA
- decoded preview and detection overlays are approximately aligned, not
  exact-frame synchronized

## Verification

Supported local verification:

- `scripts/omni test ros`

Focused tests cover:

- preview command profile resolution
- preview process lifecycle behavior
- gateway service preview responses
- Python preview viewer helper behavior
- Python overlay viewer helper behavior
- monitor shell and Tk monitor smoke behavior

Target-side preview verification requires the ROCK 5B Plus camera devices,
installed GStreamer SRT plugins, the configured SRT endpoint, and representative
Wi-Fi conditions.

## Limitations

- Robot-side preview currently uses software x264.
- Hardware H.265 encode is not exposed through the installed userspace stack on
  the current image.
- FFmpeg exposes `hevc_v4l2m2m`, but encode failed with `Could not find a valid
  device` during bring-up.
- No Rockchip-specific GStreamer H.265 encoder plugin appears to be installed
  on the current image.
- The gateway API does not publish SRT endpoint metadata.
- The Tk monitor GUI launches preview in a separate helper process and does not
  embed decoded video.
- Preview frames and detection frames come from separate camera paths and are
  not exact-frame synchronized.
- Software x265 is too CPU-expensive for the intended SBC mission budget.

## Future optimization

- Install and verify a low-overhead hardware H.265 userspace encode path on the
  SBC image.
- Move from `NV12 mainpath -> software x264 -> SRT` to
  `NV12 mainpath -> hardware H.265 -> SRT`.
- Tune preview profile defaults with measured Wi-Fi and SBC load data.
- Publish stream endpoint metadata through the gateway API.
- Embed decoded preview directly in the monitor GUI.
- Add an exact-frame debug preview path for targeted overlay verification.
- Define how recorded preview evidence correlates with RunBundle evidence.
