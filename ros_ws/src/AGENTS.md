# AGENTS.md

Additional instructions for work under `ros_ws/src/`.

The repository-root `AGENTS.md` still applies.

## ROS Boundary

ROS is the typed integration boundary between native perception, firmware/robot IO, simulation, autonomy, operator tooling, and experiment recording.

Before changing launch composition, topic names/types, frames, QoS, command paths, or sim/real behavior, read:

```text
docs/robot-runtime/ros-packages.md
```

Simulation validates the ROS contracts it exercises. It does not validate physical sensor behavior, micro-ROS transport, firmware timing, camera acceleration, RKNN/RGA behavior, or robot motion.

## Package Ownership

Important packages include:

- `bringup/`: sim and real launch composition and shared configuration;
- `omniseer_msgs/`: Omniseer-specific ROS interfaces;
- `omniseer_vision_bridge/`: native vision to ROS adapter;
- `omniseer_autonomy/`: bounded target acquisition, centering, and framing;
- `omniseer_experiments/`: RunBundle recording, retrieval, annotation, reports, and comparison tooling;
- `robot_io_adapters/`: normalized real/sim IO adapters;
- `robot_diag_control_cpp/`: robot-side gateway and preview process management;
- `robot_diag_control/`: laptop-side operator and diagnostic tools;
- `omniseer_description/` and `omniseer_gz_assets/`: robot description and simulation assets.

Treat `rf2o_laser_odometry/` and `yolo_ros/` as upstream-derived integration areas: avoid broad cleanup or divergence unless the task specifically requires it.

## Sim/Real Contract Rules

Preserve the normalized boundary above provider-specific implementation details.

Do not casually rename or repurpose established command, odometry, sensor, detection, performance, battery, or evidence interfaces.

When changing an interface:

- identify every producer and consumer;
- preserve units, frames, and timestamp semantics;
- update message/service definitions, package dependencies, configs, launch files, tests, and docs together;
- consider both simulated and real producers even when only one side is being edited.

Do not make a real-only capability mandatory for the shared simulation graph unless the architecture explicitly changes.

## Launch Composition

Keep shared behavior in the shared launch layer and provider-specific hardware/simulation behavior below that boundary.

Optional gateway, preview, dashboard, recording, or diagnostic components must not become prerequisites for mission-critical perception, command arbitration, robot IO, or bounded autonomy.

Changes to startup ordering, readiness waits, cleanup, shutdown, or fault propagation are behavioral changes and require focused tests.

## Autonomy and Motion

`omniseer_autonomy` implements bounded visual target acquisition and framing. Do not silently expand it into navigation-based object search, semantic exploration, or learned end-to-end control.

Changes affecting motion commands, target-loss behavior, confirmation logic, timeouts, framing thresholds, range/proximity stops, arbitration, or terminal states are safety-relevant.

For such changes:

- preserve bounded outputs and fail-safe stopping behavior;
- add or update controller/node regression tests when practical;
- call out the behavioral change explicitly;
- never run physical robot motion unless the user explicitly requests it.

## RunBundle and Operator Tooling

RunBundle raw artifacts are evidence. Reports, annotations, corrected media, and comparison outputs are derived review surfaces.

Do not modify raw bundle evidence in place merely to simplify report generation.

Preserve explicit provenance: model family/variant/precision/backend, run configuration, container/git identity, classes, and comparison metadata should remain explicit rather than inferred from filenames where the schema supports them.

Operator and diagnostic tooling should degrade gracefully. A laptop GUI, gRPC client, preview stream, or report failure must not change the robot mission path unless that dependency is explicitly part of the task.

## C++ and Python ROS Code

Match local package conventions.

Prefer:

- typed ROS interfaces over stringly-typed control;
- explicit units, frames, and timestamp sources;
- narrow callbacks and adapters;
- bounded queues and predictable lifecycle behavior;
- state-machine tests for behavior changes;
- semantic logs for startup, shutdown, faults, and externally visible transitions.

Do not perform expensive blocking work in mission-critical callbacks without a clear reason.

## Verification

For portable ROS package, interface, launch, config, operator-tool, autonomy, or RunBundle changes:

```bash
scripts/omni test ros
```

For simulation launch or shared-boundary behavior:

```bash
scripts/omni test smoke-sim
```

For changes that also modify native vision behavior, run the applicable `vision/AGENTS.md` checks.

For repository-level script/tool contracts, run the relevant root `tests/` pytest file or suite.

Do not claim real camera, LiDAR, Teensy, RKNN/RGA, micro-ROS transport, or physical motion validation from ROS portable tests or Gazebo smoke tests.
