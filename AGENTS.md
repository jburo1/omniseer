# AGENTS.md

Instructions for coding agents working in the Omniseer repository.

## Mission

Omniseer is an edge-to-cloud embodied AI system that runs open-vocabulary perception and bounded autonomous behavior on a physical mobile robot, then produces reviewable evidence from real runs.

Prioritize real-hardware correctness, safe and predictable robot behavior, stable interfaces, reproducible evidence, efficient edge execution, and focused changes.

Avoid speculative cleanup, unnecessary abstraction, framework churn, and broad rewrites without clear operational value.

## Default Workflow

Use:

```text
inspect -> implement -> verify at the narrowest valid boundary -> summarize evidence
```

Before editing, read the surrounding implementation and tests, the nearest relevant documentation, and any applicable nested `AGENTS.md`. Identify the owning subsystem and the environment required to verify the claim.

Prefer the smallest change that fully addresses the task.

For investigations, keep observation, hypothesis, experiment, evidence, and conclusion distinct. Do not turn an unverified hypothesis into a code change merely because it sounds plausible.

## Repository Map

- `vision/`: native V4L2/RGA/RKNN perception, post-processing, replay/comparison, native telemetry. See `vision/AGENTS.md`.
- `ros_ws/src/`: ROS contracts, launch composition, autonomy, operator tooling, adapters, and RunBundle tooling. See `ros_ws/src/AGENTS.md`.
- `firmware/`: Teensy motor/sensor behavior and micro-ROS IO. See `firmware/AGENTS.md`.
- `scripts/omni`: supported front door for normal repository workflows.
- `scripts/`, `tools/model/`, `docker/model-builder/`, and `config/classes/`: host-side model/deployment workflow pieces.
- `docker/runtime/` and `scripts/runtime.sh`: robot runtime packaging, verification, and promotion.
- `tests/`: repository-level Python contract tests.
- `docs/`: current-state architecture, operations, deployment, and verification guidance.

Mission-critical perception, autonomy, command arbitration, robot IO, and firmware behavior must not depend on optional dashboards, preview streaming, reports, or diagnostic tooling.

## Read the Right Reference

Before changing a boundary, use the relevant reference:

- system architecture: `docs/architecture/overview.md`;
- ROS sim/real contracts: `docs/robot-runtime/ros-packages.md`;
- native vision: `docs/perception/vision-pipeline.md`;
- model export/calibration/RKNN compilation: `docs/perception/yolo-world-model-deployment.md`;
- quantization investigations: `docs/perception/int8-quantization-investigation.md`;
- command surface: `docs/operations/scripts-frontdoor.md`;
- runtime lifecycle: `docs/robot-runtime/robot-runtime-container.md`;
- evidence and RunBundle semantics: `docs/verification/evidence.md`.

Documentation is a reference, not a substitute for inspecting implementation and tests. If they disagree, determine the implemented contract, update stale documentation when in scope, and report the discrepancy.

## Canonical Commands and Verification

Prefer `scripts/omni` over reconstructing package-local commands manually.

```bash
scripts/omni --help
scripts/omni <command> --help
scripts/omni env
scripts/omni doctor
```

Use the smallest applicable verification:

| Area | Expected check |
| --- | --- |
| Repository Python/tooling | targeted `python3 -m pytest -q tests/<test>.py` or `python3 -m pytest -q tests` |
| ROS packages/interfaces/config | `scripts/omni test ros` |
| Simulation boundary | `scripts/omni test smoke-sim` |
| Portable native vision | `scripts/omni test vision` |
| Firmware | `scripts/omni build firmware` |
| Documentation | `scripts/omni docs build` |
| Runtime packaging | `scripts/omni runtime verify` |
| Promotion candidate | `scripts/omni runtime verify --stage full` |

Never collapse portable host checks, simulation, RKNN Toolkit host analysis, ROCK 5B+ execution, physical robot behavior, or firmware compilation into one notion of “verified.” State the actual boundary.

## Model and Runtime Lifecycle

Model work is cross-cutting. Before changing export, calibration, text embeddings, ONNX validation, RKNN compilation, or quantization analysis, read the model deployment docs and relevant `tests/test_model_toolchain.py` coverage.

Successful ONNX export, RKNN compilation, or host-simulator analysis is not sufficient evidence of deployment readiness.

Runtime promotion is a release operation. Do not bypass repository checks with ad hoc `docker tag` or `docker push` commands merely to make promotion succeed. Preserve the clean-tree, exact-commit, same-image, and full-verification guarantees implemented by the runtime workflow.

## Contracts and Interfaces

When modifying ROS/protobuf/gRPC interfaces, configs, model IO, parameters, topics, services, or other inter-process contracts:

- preserve semantics unless the task explicitly changes them;
- evolve additively where practical;
- keep definitions, generated code, package/build metadata, tests, launch wiring, and consumers synchronized;
- document compatibility or migration implications.

Never silently change control semantics, units, coordinate frames, timestamp domains, state-machine transitions, operator-visible statuses, startup/shutdown behavior, timing-sensitive behavior, or fault handling.

## Hardware and Operational Safety

Do not do the following without explicit user instruction:

- flash firmware;
- command physical robot motion or start motion-capable autonomy;
- bypass arbitration, watchdogs, or safety limits;
- delete robot or local RunBundles;
- publish or promote runtime images;
- modify credentials, deployment targets, hardware addresses, or persistent device configuration;
- perform destructive cleanup outside ordinary repository build artifacts.

Call out changes affecting motion, autonomy, arbitration, watchdogs, timeouts, proximity stops, fault handling, or firmware.

## Performance-Critical Changes

For hot paths, avoid unnecessary copies, avoidable heap allocation, blocking mission-critical loops, unbounded queues, uncontrolled memory growth, and additional control-loop jitter.

When a change plausibly affects latency, throughput, frame/inference rate, CPU/NPU use, memory, startup time, or control timing, explain the expected mechanism and measure it when practical. State the measurement boundary.

Do not claim zero-copy unless the actual ownership and transfer path demonstrates it.

## Evidence and Generated Artifacts

Treat RunBundles as experimental evidence. Preserve raw inputs; corrections, annotations, remuxes, reports, and comparisons should be clearly derived. Do not silently alter raw evidence to make downstream results look correct.

Large model, quantization, profiling, and inference outputs are temporary unless explicitly designated as durable evidence. Prefer reproducible configuration, provenance/hashes, aggregate results, reports, and minimal representative samples over bulk generated tensors or compiler intermediates.

Do not delete ambiguous experiment data, source datasets, canonical model inputs, or RunBundles without explicit instruction.

## Working Tree, Git, and CI

- Keep diffs focused and reviewable.
- Do not revert unrelated user changes or use destructive git commands to simplify a task.
- Do not modify files outside task scope without a clear reason.
- Do not commit secrets, credentials, machine-specific absolute paths, or unreviewed generated outputs.
- Do not commit or push unless asked.
- Unless explicitly requested, do not trigger, poll, rerun, or wait for remote CI/CD workflows.

When asked to commit to `master`, run:

```bash
python3 -m pre_commit run --all-files
```

Review hook modifications intentionally and rerun affected tests when needed.

## Experimental Discipline

For model, quantization, performance, or perception comparisons:

- change one meaningful variable at a time where practical;
- hold source data, class vocabulary, preprocessing, thresholds, post-processing, and runtime conditions constant unless they are the variable under test;
- record enough provenance to reproduce the compared configurations;
- compare configurations on the same workload before attributing differences to the model;
- treat unexpected results as evidence to investigate, not results to normalize away.

## Final Response

Final summaries must distinguish:

- **Changed**
- **Locally verified**
- **Not verified**
- **Intentionally left to the user**

Include exact commands run and state whether evidence came from portable checks, simulation, host model analysis, ROCK 5B+, or the physical robot.
