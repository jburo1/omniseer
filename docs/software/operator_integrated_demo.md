# Operator-Integrated Real Demo

_Status: Phase 3 implementation added; target-hardware verification record pending_

This checklist verifies the Phase 3 slice between Phase 2 and the run
bundle recorder:

- real native perception publishes gateway-visible vision status
- preview can be toggled from the laptop app
- bounded teleop can be enabled and commanded from the laptop app
- teleop commands reach the existing stamped command path

The laptop app remains intentionally simple. It uses the existing Tk monitor for
status, preview controls, bounded teleop controls, and launching either the
plain preview viewer or the overlay viewer. Video still opens in an external
window rather than embedding in the Tk window.

## Preconditions

- Phase 2 has been run successfully on target hardware.
- The robot and laptop workspaces have been built with `scripts/omni build ros`.
- The laptop can reach the robot gateway gRPC port.
- The laptop has GStreamer tools and SRT/H.264 decode plugins installed for
  preview viewing.
- Native vision asset paths are available on the robot.

## Robot Command

From the repository root on the robot, launch the Phase 3 stack. The phase
profile enables the gateway and native vision, disables navigation/SLAM/RF2O,
and resolves model paths from `vision_bridge.real.paths.yaml`:

```bash
scripts/omni run real --phase 3
```

To record a local perception run bundle during the same Phase 3 bringup, enable
the experiment recorder from the same front door:

```bash
scripts/omni run real --phase 3 --record-run demo_001
```

This starts an optional `omniseer_experiments` sidecar that subscribes to
`/yolo/detections` and `/vision/perf` and writes:

```text
runs/demo_001/
  manifest.yaml
  detections.jsonl
  perf.jsonl
  summary.json
  evidence/
```

Use `--record` instead of `--record-run <run_id>` for a timestamped run id. Use
`--record-out <path>`, `--record-duration-sec <seconds>`, and
`--record-notes <text>` when the run needs explicit metadata. The recorder reads
the active vision parameter file to store detector, CLIP, vocab, class-list path,
and configured classes in `manifest.yaml`; `--record-classes <text>` remains
available as an explicit override. Bundles are stored locally on the robot first.
From the laptop, list and retrieve robot-side bundles with:

```bash
scripts/omni runs list
scripts/omni runs pull demo_001
```

The pull command imports the selected bundle into `runs/imported/<run_id>/` by
default, preserves additive files, and validates the imported copy with
`inspect_run`. The front door defaults to `radxa@192.168.1.178` and remote root
`/home/radxa/apps/omniseer/runs`; use `--host`, `--user`, `--remote-root`,
`--import-root`, `--out`, and `--overwrite` when the robot or laptop layout
differs from the defaults. Report generation and cloud synchronization remain
later slices.

Pass launch overrides after the phase, for example:

```bash
scripts/omni run real --phase 3 bringup camera_device:=/dev/video12
```

The default hardware split is inference on `rkisp_selfpath` (`/dev/video12`)
and preview on `rkisp_mainpath` (`/dev/video11`). The Teensy defaults to its
stable `/dev/serial/by-id/usb-Teensyduino_USB_Serial_16634450-if00` path; the
`/dev/omniseer_teensy` alias remains usable as an explicit override when the
repository udev rule is installed. Preview uses UDP `7100`; UDP `7001` is
occupied by the ROS 2 CLI daemon on the target and must not be reused for SRT.

Expected robot-side behavior:

- `robot_diag_control_cpp` starts and listens on gRPC port `50051`.
- `vision_bridge` publishes `/vision/perf`.
- `/yolo/detections` publishes when configured classes are visible.
- Teleop remains disabled until explicitly enabled from the laptop.

## Laptop Command

From the repository root on the laptop, launch the monitor through the same
front door. The preview host defaults to `--host` and status refreshes when the
window opens:

```bash
scripts/omni run monitor --host <robot-ip>
```

Use the GUI to:

- refresh or watch system status
- enable preview, then open the overlay viewer
- enable teleop
- send small directional commands or press `Space` for stop
- disable teleop before ending the run

For the first overlay data slice, the laptop can also confirm that detections
are visible through the gateway contract without subscribing to ROS directly:

```bash
ros2 run robot_diag_control robot_gateway_cli --host <robot-ip> overlay
```

To view the live preview with gateway detections and telemetry drawn on top,
press `Open Overlay` in the monitor or launch the OpenCV overlay viewer:

```bash
ros2 run robot_diag_control robot_overlay_viewer --host <robot-ip>
```

Use `Open Viewer` or `robot_preview_viewer` as the transport-only fallback when
debugging SRT/GStreamer separately from overlay drawing.

Keyboard bindings while the window has focus:

```text
W/S: forward/back
A/D: strafe left/right
Q/E: turn left/right
Space: stop
```

## Verification Commands

Before driving, a second robot shell can run the Phase 3 acceptance checks
against the existing graph:

```bash
OMNISEER_REQUIRE_DETECTIONS=1 scripts/omni run real --phase 3 verify
```

This requires live encoder, wheel-odometry, lidar, vision, and detection
messages; healthy gateway status; and zero vision errors. It also briefly
starts preview, runs a short headless overlay viewer smoke, stops preview, then
enables teleop, sends only a zero command, checks that it reaches the stamped
controller reference, and disables teleop. It fails if preview cannot bind its
camera or SRT port, or if the local overlay viewer cannot consume the preview
stream. Set `OMNISEER_SKIP_OVERLAY_SMOKE=1` only when running the script from a
minimal target shell without laptop overlay dependencies. For focused debugging,
inspect the individual topics on the robot:

```bash
ros2 topic echo --once /mecanum_drive_controller/reference
ros2 topic echo --once /vision/perf
ros2 topic echo --once /yolo/detections
```

Expected observations:

- laptop status shows `vision` available and fresh
- laptop overlay command reports detection freshness and source-space detections when targets are visible
- overlay viewer opens live preview, shows a telemetry HUD, and draws boxes when detections are present
- Phase 3 verification reports `ok: overlay viewer consumed preview`
- preview state changes to `running` when enabled
- plain preview viewer displays the SRT stream when used as a fallback
- teleop state changes to `enabled` after explicit enable
- `/mecanum_drive_controller/reference` receives stamped commands
- disabling teleop or closing the GUI sends a zero command

## Troubleshooting

| Symptom | Likely cause | Check |
| --- | --- | --- |
| GUI status refresh fails | gRPC host/port unreachable | Confirm robot IP, port `50051`, and firewall/network routing |
| Preview toggles but viewer is blank | SRT/GStreamer issue | Run `robot_preview_viewer --mode fakesink` and inspect plugin availability |
| Overlay smoke fails before opening video | Missing laptop OpenCV/GStreamer support | Install `python3-opencv` and GStreamer plugins, or set `OMNISEER_SKIP_OVERLAY_SMOKE=1` for robot-only checks |
| Vision unavailable in GUI | `/vision/perf` not reaching gateway | Check `ros2 topic echo --once /vision/perf` on the robot |
| Teleop command rejected as disabled | Teleop was not enabled | Press Enable before directional commands |
| Teleop command rejected as out of bounds | Linear or angular step exceeds gateway limits | Lower the GUI step value or tune gateway bounds intentionally |
| Phase 3 is rejected by the script | Checkout predates the phase profile | Update the checkout and confirm `scripts/omni env` lists `2,3` |

## Verification Record

- Date:
- Operator:
- Robot / SBC:
- Laptop:
- Robot command (`scripts/omni run real --phase 3` plus overrides):
- Recording enabled / run id:
- Laptop command (`scripts/omni run monitor` plus host):
- Preview observed:
- Vision status fresh in GUI:
- `/yolo/detections` observed:
- Teleop enabled in GUI:
- `/mecanum_drive_controller/reference` observed:
- Disable/close stop observed:
- Notes:
