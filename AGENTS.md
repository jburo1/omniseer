# AGENTS.md

Instructions for coding agents working in the Omniseer repository.

## Mission

Omniseer is an embodied AI system that runs perception and bounded autonomous behavior on a physical mobile robot and produces reviewable evidence from real runs.

The system loop is:

```text
sense -> decide -> act -> observe -> evaluate -> redeploy
```

Prioritize:

* correctness on real hardware;
* safe and predictable robot behavior;
* stable interfaces;
* reproducible verification;
* efficient edge execution;
* focused, reviewable changes.

Avoid speculative cleanup, unnecessary abstraction, framework churn, and broad rewrites without clear operational value.

## Default Workflow

Use this loop:

```text
inspect -> implement -> verify locally -> summarize
```

Before editing:

* read the surrounding code, tests, and nearest relevant documentation;
* identify the owning subsystem and its supported verification command;
* preserve existing architecture unless the task clearly requires structural change.

Prefer the smallest change that fully addresses the task.

## Repository Boundaries

Major ownership areas:

* `vision/` owns the native V4L2, RGA, RKNN, inference, post-processing, and detailed telemetry pipeline.
* `ros_ws/src/omniseer_vision_bridge/` owns the native-vision-to-ROS adapter.
* `ros_ws/src/omniseer_autonomy/` owns bounded target-centering behavior.
* `ros_ws/src/omniseer_experiments/` owns RunBundle recording.
* `ros_ws/src/bringup/` owns simulation and real-hardware launch composition.
* `robot_diag_control_cpp/` owns the robot-side external gateway and preview process management.
* `robot_diag_control/` owns laptop-side operator and diagnostic tooling.
* `firmware/` owns Teensy behavior, motor control, sensor integration, and micro-ROS IO.
* `docs/` owns architecture, operations, verification, and generated documentation assets.
* `scripts/omni` is the supported front door for normal repository workflows.

Mission-critical perception, autonomy, command arbitration, robot IO, and firmware behavior must not depend on optional operator dashboards, preview streaming, reports, or diagnostic tooling.

Optional systems should degrade gracefully without disrupting the robot mission path.

## Canonical Commands

Prefer `scripts/omni` over reconstructing package-local commands manually.

Useful discovery commands:

```bash
scripts/omni --help
scripts/omni env
scripts/omni doctor
```

Primary build commands:

```bash
scripts/omni build
scripts/omni build ros
scripts/omni build ros --with-vision
scripts/omni build vision
scripts/omni build firmware
scripts/omni build runtime-container
```

Primary verification commands:

```bash
scripts/omni test ros
scripts/omni test vision
scripts/omni test smoke-sim
scripts/omni docs build
scripts/omni docs diagrams --check
scripts/omni runtime verify
scripts/omni runtime verify --stage full
```

Do not substitute a broad build or test sweep when a smaller supported check directly covers the change.

## Verification Expectations

Return evidence, not confidence.

Use the smallest applicable verification set:

| Change area                                      | Minimum expected verification              |
| ------------------------------------------------ | ------------------------------------------ |
| Portable native vision code                      | `scripts/omni test vision`                 |
| ROS packages, messages, launch files, or configs | `scripts/omni test ros`                    |
| Simulation bringup or boundary topics            | `scripts/omni test smoke-sim`              |
| Documentation or D2 diagrams                     | `scripts/omni docs build`                  |
| Diagram source only                              | `scripts/omni docs diagrams --check`       |
| Firmware                                         | `scripts/omni build firmware`              |
| Runtime container packaging                      | `scripts/omni runtime verify`              |
| Candidate runtime intended for promotion         | `scripts/omni runtime verify --stage full` |

For bug fixes:

* reproduce the failure first when practical;
* add or update a regression test when practical;
* verify that the original failure no longer occurs.

When full verification requires the ROCK 5B+, RKNN/RGA SDKs, camera, LiDAR, Teensy, active ROS graph, runtime container, or physical robot, state the limitation explicitly.

Do not describe target-hardware behavior as verified when only portable, simulated, or compile-time checks were run.

## Contracts and Interfaces

Prefer explicit control and status contracts over exposing internal implementation details.

When modifying protobuf, gRPC, ROS messages, services, actions, topics, parameters, or other inter-process contracts:

* preserve existing semantics unless the task explicitly requires a change;
* evolve contracts additively where possible;
* do not casually rename, repurpose, or overload fields;
* preserve removed protobuf field numbers where applicable;
* keep definitions, generated code, package metadata, tests, and build wiring synchronized;
* document compatibility or migration implications.

Never silently change:

* control semantics;
* state-machine transitions;
* operator-visible statuses;
* API meanings;
* timing-sensitive behavior;
* startup or shutdown behavior;
* fault-handling behavior.

## Hardware and Operational Safety

Do not perform the following without explicit user instruction:

* flash firmware;
* command physical robot motion;
* bypass command arbitration or safety limits;
* delete robot or local RunBundles;
* publish or promote runtime container images;
* modify hardware addresses, device paths, credentials, or deployment targets;
* trigger destructive cleanup outside repository build artifacts.

Changes affecting motor commands, autonomy, command arbitration, watchdogs, timeouts, fault handling, or firmware must be called out explicitly.

Preserve or improve logs, telemetry, and failure reporting when changing runtime behavior.

## Performance-Critical Code

Omniseer targets constrained edge hardware and real-time-ish robot workloads.

For hot paths, avoid:

* unnecessary memory copies;
* avoidable heap allocation;
* blocking mission-critical loops;
* uncontrolled queues or memory growth;
* additional control-loop jitter;
* unnecessary startup work;
* moving critical work onto less deterministic execution paths.

When a change plausibly affects latency, throughput, frame rate, CPU, memory, startup time, or control timing:

* describe the expected impact;
* measure it when practical;
* do not use unsupported language such as “should be fine.”

## Generated and Derived Files

Do not commit arbitrary build outputs, cache directories, generated binaries, secrets, machine-specific files, or host-only paths.

Some generated documentation assets are intentionally tracked.

D2 sources live under:

```text
docs/diagrams/
```

Rendered SVG assets live under:

```text
docs/assets/diagrams/
```

When changing a tracked diagram:

```bash
scripts/omni docs diagrams
scripts/omni docs diagrams --check
```

Use:

```bash
scripts/omni docs build
```

to verify diagram freshness, MkDocs output, and embedded diagram links.

Do not edit rendered SVGs directly when a D2 source owns them.

## Dependencies

Do not add a dependency unless it clearly improves correctness, maintainability, verification, performance, or delivery speed.

When adding one:

* justify it briefly;
* prefer mature and well-maintained dependencies;
* avoid duplicating capabilities already present;
* account for availability on Ubuntu, the development container, the ROCK 5B+, and CI where relevant.

## Coding and Documentation Style

Match the local code and configuration before introducing new patterns.

Formatting and mechanical style are enforced through repository tooling, including Ruff and clang-format. Do not duplicate formatter behavior manually.

Prefer:

* explicit ownership and lifetimes;
* simple control flow;
* narrow adapters;
* bounded enums and typed messages;
* semantic comments describing invariants, units, frames, timing, ownership, and failure behavior;
* documentation that describes the current implemented system rather than speculative future architecture.

Update the nearest relevant documentation when changing:

* operational commands;
* public interfaces;
* architecture boundaries;
* deployment behavior;
* lifecycle-critical behavior;
* verification procedures.

## Working-Tree Rules

* Keep diffs focused and reviewable.
* Do not revert unrelated user changes.
* Do not modify files outside the task scope without a clear reason.
* Prefer durable repository changes over one-off shell state.
* Avoid assumptions tied to one host machine.
* Account for development through SSH, devcontainers, the robot SBC, and laptop-side tooling where relevant.
* Do not commit secrets, credentials, machine-specific absolute paths, or unreviewed generated outputs.

## CI/CD Policy

Unless explicitly requested:

* do not trigger remote CI/CD runs;
* do not poll GitHub Actions;
* do not wait for remote workflows;
* do not inspect remote CI logs;
* do not treat remote CI completion as part of the task’s done criteria.

Use targeted local checks instead.

Remote CI/CD remains intentionally controlled by the user unless the task explicitly concerns CI behavior or failures.

## Commits

When asked to commit:

* use a conventional prefix with an explicit scope and descriptive summary;
* include only intentional task-related changes;
* mention meaningful secondary changes when they help review.

Before committing and pushing to `master`, run:

```bash
python3 -m pre_commit run --all-files
```

If hooks modify files:

1. review the modifications;
2. stage them intentionally;
3. rerun pre-commit;
4. commit only after the hooks pass.

## Final Response

Final summaries must clearly distinguish:

* **Changed**
* **Locally verified**
* **Not verified**
* **Intentionally left to the user**

Include the exact commands run and report any hardware, SDK, container, ROS graph, or environment limitations.
