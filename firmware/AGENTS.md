# AGENTS.md

Additional instructions for work under `firmware/`.

The repository-root `AGENTS.md` still applies.

## Purpose

`firmware/` owns Teensy 4.1 behavior, motor control, encoder/sensor acquisition, battery/range/IMU publication, command handling, and the micro-ROS transport boundary.

Before changing ROS-facing firmware behavior, read:

```text
docs/robot-runtime/ros-packages.md
```

## Safety-Critical Behavior

Motor commands, command freshness/timeouts, disconnect handling, watchdog behavior, kinematics, encoder mapping, and range/proximity-related behavior are safety-relevant.

Do not weaken existing stop/fail-safe behavior to make communication or testing easier.

When changing safety-relevant behavior:

- make the behavior explicit in the diff and final summary;
- preserve bounded command handling;
- add compile-time or unit coverage where practical;
- require physical validation before claiming real motor/sensor behavior is verified.

Never flash the Teensy or command physical motors without explicit user instruction.

## ROS and micro-ROS Contracts

Firmware participates in the real side of the normalized ROS boundary.

Preserve established topic types, units, frame IDs, wheel ordering, and timestamp semantics unless the task explicitly changes the contract.

When changing a firmware-published or subscribed interface, inspect and update the corresponding ROS message definitions, adapters, launch/configuration, tests, and documentation outside `firmware/` as required.

Do not claim sim/real parity merely because the topic names match; simulation does not validate physical timing or sensor behavior.

## Transport and Logging

The firmware uses the USB serial path for custom micro-ROS transport.

Do not add arbitrary `Serial` logging that can interfere with the transport after micro-ROS owns it. Use the existing debug/logging mechanisms and preserve transport lifecycle behavior.

Changes to agent reconnect, ping, executor spin, teardown/reinit, or time synchronization are lifecycle changes and should preserve safe behavior during connection loss and recovery.

## Timing and Embedded Constraints

Avoid adding blocking, unbounded, or allocation-heavy work to control, sensor, or micro-ROS loops without a clear reason.

Preserve explicit timestamp conversion and time-sync behavior when changing sensor publication.

Be careful with:

- `millis()`/`micros()` wrap-safe elapsed-time comparisons;
- motor and encoder wheel ordering;
- sensor units;
- executor/spin budgets;
- communication failures that could leave stale commands active.

## Dependencies and Generated micro-ROS Support

Use the repository-supported PlatformIO/micro-ROS setup rather than bypassing its patch/build flow.

Do not hand-edit generated dependency content when the repository script owns that transformation.

## Verification

Compile firmware through the supported front door:

```bash
scripts/omni build firmware
```

If ROS-facing contracts changed, also run:

```bash
scripts/omni test ros
```

Firmware compilation is compile-only evidence. It does not prove flashing, micro-ROS transport, sensor quality, timing, watchdog behavior, motor direction, or physical robot safety.

State explicitly what remains for board or robot validation.
