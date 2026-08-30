# AGENTS.md

Instructions for coding agents working in the Omniseer repository.

## Mission

Omniseer is an edge-to-cloud embodied AI system that runs open-vocabulary perception and bounded autonomous behavior on a physical mobile robot, then produces reviewable evidence from real runs.

The system loop is:

```text
sense -> infer -> decide -> act -> observe -> evaluate -> redeploy
```

Prioritize:

* correctness on real hardware;
* safe and predictable robot behavior;
* stable and explicit interfaces;
* reproducible model and runtime provenance;
* reviewable experimental evidence;
* efficient edge execution;
* focused, reviewable changes.

Avoid speculative cleanup, unnecessary abstraction, framework churn, broad rewrites, and changes that add operational complexity without clear value.

A successful change is not merely code that compiles. It should preserve or improve the path from source code or model artifact to verified behavior and evidence.

---

## Default Workflow

Use this loop:

```text
inspect -> implement -> verify at the narrowest valid boundary -> summarize evidence
```

Before editing:

* read the surrounding implementation;
* read nearby tests;
* read the nearest relevant documentation;
* identify the owning subsystem;
* identify which execution environment is required;
* identify the narrowest supported verification command;
* preserve existing architecture unless the task clearly requires structural change.

Prefer the smallest change that fully addresses the task.

Do not broaden a task into cleanup or refactoring unless the broader change is necessary for correctness.

When investigating a bug, separate:

```text
observation -> hypothesis -> experiment -> evidence -> conclusion
```

Do not encode an unverified hypothesis as a fix.

---

## Repository Boundaries

Major ownership areas:

* `vision/` owns native V4L2 capture, image buffers, RGA preprocessing, RKNN inference, post-processing, replay/comparison tooling, and detailed native telemetry.
* `ros_ws/src/omniseer_vision_bridge/` owns the native-vision-to-ROS adapter.
* `ros_ws/src/omniseer_autonomy/` owns bounded target acquisition, centering, framing, and associated state-machine behavior.
* `ros_ws/src/omniseer_experiments/` owns RunBundle recording, inspection, annotation, report generation, comparison reports, retrieval, and evidence tooling.
* `ros_ws/src/bringup/` owns simulation and real-hardware launch composition.
* `ros_ws/src/omniseer_msgs/` owns Omniseer-specific ROS interface definitions.
* `ros_ws/src/robot_io_adapters/` owns ROS-side robot IO adaptation.
* `ros_ws/src/robot_diag_control_cpp/` owns the robot-side external gateway and preview process management.
* `ros_ws/src/robot_diag_control/` owns laptop-side operator and diagnostic tooling.
* `ros_ws/src/omniseer_description/` and `ros_ws/src/omniseer_gz_assets/` own robot description and simulation assets.
* `firmware/` owns Teensy behavior, motor control, sensor integration, watchdog behavior, and micro-ROS IO.
* `scripts/` owns supported repository workflow wrappers.
* `scripts/omni` is the supported front door for normal build, test, run, model, runtime, evidence, and documentation workflows.
* `tools/model/` owns host-side model-analysis and model-preparation helpers invoked by the supported model workflow.
* `docker/model-builder/` owns the host-side YOLO-World/RKNN build environment.
* `docker/runtime/` owns robot runtime container packaging.
* `config/classes/` owns task and calibration vocabularies.
* `tests/` owns repository-level Python contract tests for scripts, model tooling, runtime packaging, and other cross-cutting behavior.
* `docs/` owns architecture, operations, perception, deployment, verification, and generated documentation assets.

Mission-critical perception, autonomy, command arbitration, robot IO, and firmware behavior must not depend on optional operator dashboards, preview streaming, reports, or diagnostic tooling.

Optional systems must degrade gracefully without disrupting the mission path.

Keep host-side model-building dependencies separate from the robot runtime image unless a task explicitly changes that architectural boundary.

---

## Execution Environments

Omniseer spans multiple execution environments. Do not treat verification in one environment as proof about another.

### Development host / devcontainer

Appropriate for:

* repository Python tests;
* portable ROS builds and tests;
* simulation;
* portable native vision tests;
* documentation;
* shell/static checks;
* runtime packaging checks that do not require target hardware.

### Host-side model builder

Appropriate for:

* YOLO-World checkpoint handling;
* CLIP text embedding generation;
* calibration dataset generation;
* ONNX export;
* RKNN compilation;
* RKNN Toolkit host-simulator accuracy analysis.

This environment does **not** prove real RK3588 execution.

### ROCK 5B+ / RK3588

Required for claims about:

* actual RKNN runtime behavior;
* RGA behavior;
* camera/device integration;
* NPU inference;
* end-to-end latency or throughput;
* memory behavior under the robot workload;
* thermal behavior;
* hardware preview encoding;
* full runtime-container verification.

### Physical robot

Required for claims about:

* real motor behavior;
* autonomy motion behavior;
* sensor integration;
* command arbitration under physical operation;
* target acquisition/framing performance in the real environment;
* safety behavior involving actual movement.

### Teensy

Required when runtime behavior depends on flashed firmware or hardware-specific Teensy IO.

Always state which boundary was actually verified.

---

## Canonical Commands

Prefer `scripts/omni` over reconstructing package-local commands manually.

Discovery:

```bash
scripts/omni --help
scripts/omni <command> --help
scripts/omni env
scripts/omni doctor
```

Build:

```bash
scripts/omni build
scripts/omni build strict
scripts/omni build ros
scripts/omni build ros --with-vision
scripts/omni build vision
scripts/omni build firmware
scripts/omni build runtime-container
```

Portable verification:

```bash
scripts/omni test ros
scripts/omni test vision
scripts/omni test smoke-sim
python3 -m pytest -q tests
scripts/omni docs build
scripts/omni docs diagrams --check
```

Model workflow:

```bash
scripts/omni model assets
scripts/omni model image
scripts/omni model calibration
scripts/omni model analyze --variant <v2s|v2m|v2l>
scripts/omni model export --variant <v2s|v2m|v2l> --weights <checkpoint>
scripts/omni model compile --variant <v2s|v2m|v2l> --onnx <model.onnx> --precision <fp|int8>
scripts/omni model build --variant <v2s|v2m|v2l> --weights <checkpoint> --precision <fp|int8>
```

Runtime lifecycle:

```bash
scripts/omni runtime build
scripts/omni runtime verify
scripts/omni runtime verify --stage full
scripts/omni runtime record
scripts/omni runtime push
scripts/omni runtime pull
```

RunBundle workflow:

```bash
scripts/omni runs inspect <run_dir>
scripts/omni runs annotate <run_dir>
scripts/omni runs report <run_dir>
scripts/omni runs video <run_dir>
scripts/omni runs compare <run_dir>
scripts/omni runs comparison-report ...
```

Do not substitute a broad build or test sweep when a smaller supported check directly covers the change.

---

## Verification Expectations

Return evidence, not confidence.

Use the smallest applicable verification set:

| Change area                                                             | Minimum expected verification                                                                        |
| ----------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| Repository-level Python/tooling contract                                | targeted `python3 -m pytest -q tests/<relevant_test>.py`                                             |
| Cross-cutting scripts/tooling behavior                                  | `python3 -m pytest -q tests`                                                                         |
| Model wrapper, calibration, export/compile wiring, model-builder config | `python3 -m pytest -q tests/test_model_toolchain.py`                                                 |
| Runtime lifecycle or container workflow                                 | `python3 -m pytest -q tests/test_runtime_container_workflow.py` plus applicable runtime verification |
| Portable native vision code                                             | `scripts/omni test vision`                                                                           |
| ROS packages, messages, launch files, or configs                        | `scripts/omni test ros`                                                                              |
| Simulation bringup or boundary topics                                   | `scripts/omni test smoke-sim`                                                                        |
| Documentation                                                           | `scripts/omni docs build`                                                                            |
| D2 source only                                                          | `scripts/omni docs diagrams --check`                                                                 |
| Firmware source                                                         | `scripts/omni build firmware`                                                                        |
| Runtime container packaging                                             | `scripts/omni runtime verify`                                                                        |
| Candidate runtime intended for promotion                                | `scripts/omni runtime verify --stage full`                                                           |
| Shell script behavior                                                   | applicable tests plus `shellcheck -S error -x <changed-shell-files>` when available                  |

For bug fixes:

* reproduce the failure first when practical;
* preserve the reproducer or encode it as a regression test when practical;
* verify that the original failure no longer occurs;
* distinguish regression coverage from integration or hardware validation.

For model-quality failures, compilation success is not sufficient verification.

For performance-sensitive changes, functional tests are not sufficient evidence for performance claims.

When full verification requires the ROCK 5B+, RKNN/RGA SDKs, camera, LiDAR, Teensy, active ROS graph, runtime container, or physical robot, state the limitation explicitly.

Never describe target-hardware behavior as verified when only portable, simulated, host-simulator, compile-time, or static checks were run.

---

## Model Lifecycle and Quantization

Omniseer has a first-class model deployment pipeline:

```text
YOLO-World checkpoint
    -> ONNX
    -> RKNN conversion
    -> target runtime
    -> controlled comparison
    -> hardware validation
```

Treat each boundary separately.

### Deployment contract

The current YOLO-World v2 deployment path supports v2-S, v2-M, and v2-L.

Unless a task explicitly changes the model/runtime contract, preserve:

```text
images: [1, 3, 640, 640]
texts:  [1, 80, 512]
```

and the expected six detection outputs used by the native runtime.

Do not casually change:

* input names;
* shapes;
* preprocessing;
* normalization;
* layout assumptions;
* input data types;
* output ordering;
* post-processing assumptions;
* class capacity;
* padding behavior.

Changes to any of these are deployment-contract changes and must be traced through exporter, RKNN conversion, native runtime bindings, tests, and documentation.

### Calibration

YOLO-World is a multi-input model.

INT8 calibration must account for both:

* representative robot images;
* the text embedding input.

Do not reduce calibration reasoning to image selection alone.

When calibration images, calibration classes, CLIP snapshot, exporter behavior, or conversion settings change:

* regenerate the required calibration assets;
* verify the generated dataset;
* preserve enough provenance to reproduce the calibration;
* do not assume a previously compiled INT8 model remains comparable.

Do not describe a tiny convenience dataset as representative calibration unless the task is explicitly diagnostic.

### Host analysis

`scripts/omni model analyze` is a host-side RKNN Toolkit simulator diagnostic.

Its output can support statements about:

* simulated quantization error;
* layer-wise divergence;
* relative diagnostic behavior under the controlled input.

It cannot by itself support claims about:

* real RK3588 inference correctness;
* NPU latency;
* thermals;
* sustained throughput;
* runtime input binding correctness on the board;
* end-to-end detection quality on the robot.

Always label host-simulator conclusions as such.

### Deployment readiness

Do not treat any of the following as sufficient evidence that a model is deployment-ready:

* successful ONNX export;
* successful RKNN compilation;
* successful Toolkit host simulation;
* one successful image;
* one successful detection;
* lower model size alone.

Deployment readiness requires evidence appropriate to the claim, normally including controlled model comparison and real target-hardware execution.

### Generated model artifacts

`models/`, generated RKNN/ONNX outputs, and quantization-analysis outputs are generally local or ignored artifacts.

Do not commit large generated model outputs unless explicitly designated as durable repository evidence.

Do not overwrite generated model artifacts silently.

When an existing artifact is intentionally replaced, preserve or report the provenance necessary to distinguish the old and new artifacts.

---

## Runtime Container Lifecycle

Treat the runtime lifecycle as:

```text
source commit
    -> candidate image
    -> verification
    -> full target verification
    -> promotion
```

`scripts/omni runtime build` produces a candidate.

`scripts/omni runtime verify` performs the bounded smoke verification.

`scripts/omni runtime verify --stage full` is the target-hardware promotion boundary.

`scripts/omni runtime push` is a release operation.

Do not bypass the repository-managed promotion checks through manual `docker tag` or `docker push` commands merely to make a release succeed.

Promotion must preserve the repository guarantees that the pushed image corresponds to:

* a clean git tree;
* the current exact git commit;
* the same local image that passed full verification;
* recorded verification metadata.

Do not weaken those checks without an explicit task requiring a lifecycle-policy change.

A container that builds successfully is not automatically a verified robot runtime.

---

## RunBundle and Evidence Integrity

Treat RunBundles as experimental evidence, not disposable logs.

Preserve the distinction between:

```text
raw capture -> corrected/normalized derivative -> annotation -> aggregate/report
```

Raw evidence should remain inspectable.

Do not silently modify raw captured inputs to make downstream results look correct.

When repairing or transforming a captured artifact:

* retain the raw source;
* create a clearly named derived artifact;
* document or encode the transformation;
* ensure downstream tooling makes the distinction clear where relevant.

Examples include:

* timestamp alignment;
* video repair;
* coordinate correction;
* annotation;
* remuxing;
* downscaling;
* comparison overlays.

Do not infer model provenance from filenames when explicit RunBundle model metadata is available or can be recorded.

For comparative experiments, check for confounders before interpreting results. Relevant factors include:

* source frames or workload;
* class vocabulary;
* score threshold;
* NMS threshold;
* preprocessing;
* text embeddings;
* calibration dataset;
* runtime/model version;
* inference backend;
* precision;
* frame count;
* timing or dropped-frame behavior.

If two runs are not directly comparable, say so rather than producing a stronger conclusion than the evidence supports.

Preserve:

* experiment configuration;
* model and compiler provenance;
* dataset/calibration provenance where relevant;
* runtime/container provenance;
* aggregate metrics;
* error summaries;
* representative evidence;
* the minimal raw material necessary to audit important conclusions.

Do not delete RunBundles or ambiguous experiment data without explicit user instruction.

---

## Contracts and Interfaces

Prefer explicit control and status contracts over exposing implementation details.

When modifying protobuf, gRPC, ROS messages, services, actions, topics, parameters, or other inter-process contracts:

* preserve existing semantics unless the task explicitly requires a change;
* evolve contracts additively where possible;
* do not casually rename, repurpose, or overload fields;
* preserve removed protobuf field numbers where applicable;
* keep definitions, generated code, package metadata, tests, launch wiring, and consumers synchronized;
* document compatibility or migration implications.

Never silently change:

* control semantics;
* state-machine transitions;
* operator-visible statuses;
* API meanings;
* units;
* coordinate frames;
* timestamp domains;
* timing-sensitive behavior;
* startup or shutdown behavior;
* fault-handling behavior.

For ROS data, be explicit about units, frames, timestamp source, QoS assumptions, and ownership when those details affect correctness.

---

## Hardware and Operational Safety

Do not perform the following without explicit user instruction:

* flash firmware;
* command physical robot motion;
* start an autonomy behavior that can move the physical robot;
* bypass command arbitration;
* bypass proximity or safety limits;
* weaken watchdogs or safety stops;
* delete robot or local RunBundles;
* publish or promote runtime container images;
* modify hardware addresses;
* modify deployment targets;
* change credentials;
* alter persistent device configuration;
* trigger destructive cleanup outside ordinary repository build artifacts.

Changes affecting any of the following must be called out explicitly:

* motor commands;
* autonomy;
* command arbitration;
* watchdogs;
* timeouts;
* emergency or proximity stops;
* fault handling;
* firmware;
* physical device selection.

Preserve or improve logs, telemetry, and failure reporting when changing runtime behavior.

Fail safe where practical.

Do not hide a hardware failure by converting it into apparent success.

---

## Performance-Critical Code

Omniseer targets constrained edge hardware and real-time-ish robot workloads.

For hot paths, avoid:

* unnecessary memory copies;
* avoidable image format conversions;
* avoidable heap allocation;
* blocking mission-critical loops;
* uncontrolled queues;
* unbounded memory growth;
* unnecessary synchronization;
* additional control-loop jitter;
* unnecessary startup work;
* moving critical work onto less deterministic execution paths.

Pay particular attention to:

```text
camera
-> buffer ownership
-> preprocessing
-> RKNN input binding
-> inference
-> post-processing
-> ROS publication
```

and:

```text
sensor/control input
-> arbitration
-> autonomy decision
-> motor command
```

When a change plausibly affects latency, throughput, frame rate, inference rate, CPU, NPU, memory, startup time, copies, or control timing:

* describe the expected mechanism;
* measure it when practical;
* report the measurement boundary;
* do not use unsupported language such as “should be fine.”

Do not claim zero-copy unless the relevant ownership and memory-transfer path actually demonstrates it.

---

## Experiment Artifacts

Treat large ML, inference, profiling, and quantization outputs as temporary unless explicitly designated as durable evidence.

Prefer retaining:

* experiment configuration and manifests;
* source/model/compiler/dataset versions;
* hashes where useful;
* aggregate metrics;
* error summaries;
* plots and reports;
* representative samples needed to support conclusions.

Do not retain or commit large regenerable outputs such as:

* per-frame tensor dumps;
* per-layer tensor dumps;
* duplicated activations;
* duplicated reference tensors;
* temporary compiler intermediates;
* bulk debug output used only during investigation;
* generated model binaries already reproducible from documented inputs.

For investigations expected to generate substantial data:

* use ignored scratch/artifact locations;
* avoid duplicating shared reference data;
* summarize durable findings before cleanup;
* preserve reproduction inputs and provenance;
* remove large temporary artifacts when the investigation is complete and their role is unambiguous.

Do not delete ambiguous experiment data, source datasets, canonical model inputs, or RunBundles without explicit user instruction.

---

## Generated and Derived Files

Do not commit arbitrary:

* build outputs;
* cache directories;
* generated binaries;
* credentials;
* secrets;
* machine-specific files;
* host-only absolute paths;
* large ignored experiment outputs.

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

to verify documentation output and diagram links.

Do not edit rendered SVGs directly when a D2 source owns them.

---

## Dependencies

Do not add a dependency unless it clearly improves correctness, maintainability, verification, performance, or delivery speed.

When adding one:

* justify it briefly;
* prefer mature and maintained dependencies;
* avoid duplicating capabilities already present;
* account for the environment in which it must run;
* keep host-only tooling out of the robot runtime unless it is genuinely required there.

Evaluate availability across the applicable environments:

* Ubuntu development host;
* devcontainer;
* CI;
* model-builder container;
* ROCK 5B+ runtime container;
* Teensy toolchain.

Do not make the robot runtime heavier merely because a dependency is convenient for build-time tooling.

---

## Coding and Documentation Style

Match the local code and configuration before introducing new patterns.

Formatting and mechanical style are enforced through repository tooling including Ruff and clang-format.

Do not manually fight formatter output.

Prefer:

* explicit ownership and lifetimes;
* RAII in C++;
* simple control flow;
* narrow adapters;
* bounded queues;
* typed messages;
* bounded enums;
* explicit units;
* explicit timestamp domains;
* explicit frame semantics;
* clear failure behavior;
* semantic comments describing invariants rather than restating code.

Update the nearest relevant documentation when changing:

* operational commands;
* public interfaces;
* model deployment behavior;
* calibration behavior;
* architecture boundaries;
* deployment behavior;
* lifecycle-critical behavior;
* evidence semantics;
* verification procedures.

Documentation should describe the implemented system, not speculative future architecture.

---

## Working-Tree Rules

* Keep diffs focused and reviewable.
* Do not revert unrelated user changes.
* Inspect the current working tree before making broad edits.
* Do not modify files outside task scope without a clear reason.
* Prefer durable repository changes over one-off shell state.
* Avoid assumptions tied to one machine.
* Account for SSH, devcontainers, the robot SBC, and laptop-side tooling where relevant.
* Do not commit secrets, credentials, machine-specific absolute paths, or unreviewed generated outputs.
* Do not delete unrelated generated or ignored data to obtain a clean tree.
* Do not use destructive git commands to simplify an agent task.
* Do not overwrite user work merely because the repository differs from an expected state.

When generated outputs interfere with a task, identify exactly what they are before removing anything.

---

## CI/CD Policy

Unless explicitly requested:

* do not trigger remote CI/CD runs;
* do not poll GitHub Actions;
* do not wait for remote workflows;
* do not inspect remote CI logs;
* do not rerun workflows;
* do not publish packages or containers;
* do not treat remote CI completion as part of the task's done criteria.

Use targeted local checks instead.

Remote CI/CD remains controlled by the user unless the task explicitly concerns CI behavior or failures.

Repository CI is useful as a specification for portable checks, but passing CI does not prove target-hardware execution.

---

## Commits

When asked to commit:

* use a conventional prefix with an explicit scope and descriptive summary;
* include only intentional task-related changes;
* mention meaningful secondary changes when useful for review;
* do not bundle unrelated cleanup.

Before committing and pushing to `master`, run:

```bash
python3 -m pre_commit run --all-files
```

If hooks modify files:

1. review the modifications;
2. stage them intentionally;
3. rerun pre-commit;
4. run any affected targeted tests again when formatting or fixes could alter behavior;
5. commit only after the hooks pass.

Do not push unless the user asked for a push.

Runtime-container promotion is separate from a git push and requires explicit user instruction.

---

## Final Response

Final summaries must clearly distinguish:

* **Changed**
* **Locally verified**
* **Not verified**
* **Intentionally left to the user**

Include:

* exact commands run;
* relevant test results;
* generated artifacts when meaningful;
* hardware/SDK/container/ROS-graph limitations;
* whether validation was portable, simulated, host-model-simulator, ROCK 5B+, or physical-robot;
* any safety- or performance-relevant behavior changed.

Do not say “verified” without stating the boundary at which verification occurred.

For experiments and investigations, distinguish:

```text
observed
inferred
not yet tested
```

The final response should make it possible for the next engineer to understand both what changed and what evidence supports it.
