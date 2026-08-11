# Verification Evidence

This page ties Omniseer claims to inspectable checks, local verification commands,
and implemented evidence formats. It is an evidence catalog: each record states
what it supports, where it runs, whether it is public, and what it does not prove.

## Evidence Model

Omniseer uses five evidence categories with different strengths:

- Public CI evidence: GitHub Actions workflow results that anyone can inspect for
  the pushed commit.
- Local portable verification: repository commands that reproduce CI-equivalent or
  hardware-independent checks in a developer environment.
- Public target-hardware evidence: named RunBundles, reports, images, videos, or
  measurements from the ROCK 5B+ robot that are linked from this page.
- Non-public local evidence: local run artifacts or measurements that are useful
  to an operator but are not public review evidence.
- Implementation-backed capability: source, tests, and launch/configuration support
  for a capability without a linked execution artifact.

Implementation-backed capability is weaker than execution evidence. A source-level
claim says the capability is implemented; a target-hardware execution claim needs a
named run, report, video, annotated frame, or measurement.

## Evidence Inventory

| Evidence record | System claim | Supporting artifact or check | Environment | Publicly inspectable | Limits |
| --- | --- | --- | --- | --- | --- |
| [`ci` workflow](https://github.com/jburo1/omniseer/actions/workflows/ci.yml) | Portable software builds and tests pass for the covered paths | Ruff/pre-commit, Shellcheck, root pytest, portable ROS build/test, Gazebo smoke topics, portable native vision tests | GitHub Actions Ubuntu 24.04 and ROS Kilted container | Yes | Does not compile RKNN/RGA bridge or exercise camera, NPU, LiDAR, Teensy, micro-ROS transport, motor behavior, or target-hardware timing |
| [`runtime` workflow](https://github.com/jburo1/omniseer/actions/workflows/runtime.yml) | Hardware-independent runtime image packaging can build and expose the portable ROS runtime surface | `scripts/omni build runtime-container --target portable-runtime --image omniseer/robot-runtime:portable-ci`; container smoke requiring `/twist_mux` readiness | GitHub Actions Ubuntu 24.04 Docker runner | Yes | Uses disabled camera, RKNN/RGA, LiDAR, Teensy, gateway, boundary-topic waits, and pre-launch cleanup; does not publish a robot runtime image |
| [`firmware` workflow](https://github.com/jburo1/omniseer/actions/workflows/firmware.yml) | Teensy 4.1 firmware compiles for covered firmware paths | `scripts/omni build firmware` | GitHub Actions Ubuntu 24.04 | Yes | Compile-only; does not flash a board or validate motors, sensors, timing, watchdog behavior, or micro-ROS transport |
| [`docs` workflow](https://github.com/jburo1/omniseer/actions/workflows/docs.yml) | Public documentation builds strictly and deploys to GitHub Pages | `mkdocs build --strict`; `mkdocs gh-deploy --clean --force --verbose` after build success | GitHub Actions Ubuntu 24.04 | Yes | Documentation evidence only; does not exercise robot runtime behavior |
| Local docs build | Documentation and tracked diagram assets are internally consistent | `scripts/omni docs build` | Developer checkout | No | Local result depends on the developer environment; it is not a public artifact until the docs workflow runs |
| Local portable ROS check | Portable ROS packages, launch structure, and package tests pass locally | `scripts/omni test ros` | Developer checkout with ROS Kilted dependencies | No | Does not prove target-device camera, RKNN/RGA, LiDAR, Teensy, or physical robot behavior |
| Local simulation smoke check | The shared simulation bringup exposes asserted boundary topics | `scripts/omni test smoke-sim` | Developer checkout with ROS/Gazebo dependencies | No | Simulation-only; does not prove real sensor, actuator, or target-hardware timing behavior |
| Local portable vision check | Hardware-independent native vision components preserve their contracts | `scripts/omni test vision` | Developer checkout with portable native build dependencies | No | Does not exercise V4L2 camera capture, RGA preprocessing, RKNN inference, or NPU latency |
| Local firmware compile | Firmware sources compile outside CI | `scripts/omni build firmware` | Developer checkout with PlatformIO | No | Compile-only; no board flash or physical IO validation |
| RunBundle tooling | The implemented recorder, inspection, retrieval, annotation, and report code can create and review structured run evidence | `omniseer_experiments` tests in `scripts/omni test ros`; `scripts/omni runs inspect`, `annotate`, and `report` | Source tree and local runs | Partly: source and tests are public; individual local runs are not public unless linked | Tool support does not imply that autonomy, perception accuracy, or target hardware has been executed in a public run |
| Native target runtime implementation | The codebase contains the V4L2/RGA/RKNN perception path and ROS bridge integration | `vision/`, `ros_ws/src/omniseer_vision_bridge/`, launch/config files, and portable tests for hardware-independent pieces | Source tree and target runtime code | Yes for source; no linked execution artifact here | Implementation-backed capability only unless tied to a target-hardware run or measurement |
| Bounded autonomy implementation | The codebase contains bounded visual target acquisition and framing behavior | `ros_ws/src/omniseer_autonomy/`, `scripts/omni run autonomy`, controller/node tests, and RunBundle `autonomy.jsonl` support | Source tree and local/target runtime when launched | Yes for source; no linked execution artifact here | `autonomy.jsonl` support does not itself prove an autonomy run occurred |

## Public CI Evidence

The public workflows are the inspectable CI evidence for the default branch:

- [`ci`](https://github.com/jburo1/omniseer/actions/workflows/ci.yml) covers
  portable linting, root tests, portable ROS build/test, the Gazebo smoke boundary
  topics `/clock`, `/imu`, `/scan`, and `/mecanum_drive_controller/odometry`, and
  portable native vision tests.
- [`runtime`](https://github.com/jburo1/omniseer/actions/workflows/runtime.yml)
  covers hardware-independent runtime packaging and a positive `/twist_mux`
  readiness smoke check inside the portable image.
- [`firmware`](https://github.com/jburo1/omniseer/actions/workflows/firmware.yml)
  covers compile-only Teensy 4.1 firmware builds.
- [`docs`](https://github.com/jburo1/omniseer/actions/workflows/docs.yml) covers
  strict MkDocs build and GitHub Pages deployment.

These workflows verify portable contracts. They do not exercise target-device
camera capture, RKNN/RGA execution, NPU timing, LiDAR/range data, Teensy flashing,
micro-ROS transport, robot actuation, or public RunBundle recording.

## Local Portable Verification

The supported local verification surface mirrors the repository front door:

| Check | What it verifies | Boundary |
| --- | --- | --- |
| `scripts/omni test ros` | Portable ROS packages, launch tests, message/service integration, package tests, and RunBundle tool tests | Requires local ROS dependencies; does not prove target hardware |
| `scripts/omni test smoke-sim` | Headless simulation bringup and asserted boundary topics | Simulation-only |
| `scripts/omni test vision` | Portable native vision support code, including buffer, telemetry, and rolling-stat behavior | Hardware-independent only |
| `scripts/omni build firmware` | Teensy firmware compilation | Compile-only |
| `scripts/omni docs build` | Diagram freshness, Markdown/navigation validity, and strict MkDocs output | Documentation only |
| `scripts/omni runtime verify` | Local runtime container smoke verification for the selected stage | Portable smoke is not target-hardware execution; `--stage full` is meaningful only in the target runtime environment |

Local checks are useful evidence for the operator or developer who ran them. They
are not public evidence unless the run output is published as an artifact and linked
from this page.

## Target-Hardware Evidence

No public target-hardware RunBundle, static report, annotated frame, video, or
measurement is linked from this repository state. Target-hardware claims in this
catalog are therefore limited to implementation-backed capability unless a named
public artifact is listed in the inventory.

Non-public local target-hardware artifacts can still be operationally useful, but
they are not presented here as public review evidence.

## RunBundle Evidence Format

RunBundles are the implemented evidence format for robot runs. A bundle is a
directory, normally under `runs/<run_id>` on the robot or under an imported runs
directory on the laptop. The format supports additive artifacts, so a bundle may
contain only the streams that were enabled and available for that run.

Common bundle files:

| Artifact | Required by format | What it records | Evidence use |
| --- | --- | --- | --- |
| `manifest.yaml` | Yes | Schema version, run ID, start/end time, configured classes, explicit model family/variant/precision/backend and model paths, optional comparison ID/trial/workload, launch command/profile/mode/arguments, ROS distro, git SHA, container reference/digest, experiment metadata, topic names, and provenance hashes/copies when available | Provenance, configuration, run identity, and whether autonomy was requested |
| `detections.jsonl` | Yes for perception bundles | Typed `/yolo/detections` records, classes, scores, boxes, frame IDs, and timestamps | Perception outputs and class/score summaries |
| `perf.jsonl` | Yes for perception bundles | `/vision/perf` records, producer/consumer rates, stage timing summaries, source age, processed counts, and error counters | Performance and health claims from ROS vision summaries |
| `summary.json` | Expected after finalization | Final duration, message counts, detections by class, confidence summaries, performance summaries, errors, and dropped-record counts | Compact run status and inspection index |
| `system.jsonl` | Optional | Low-rate CPU, sampled per-process CPU attribution, memory, thermal, network, onboard battery, and `/battery` LiPo snapshots when sources are available | Resource and system-state context |
| `pipeline_telemetry.jsonl` | Optional | Native producer/consumer stage telemetry when pipeline JSONL telemetry is enabled | Detailed native timing, freshness, and stage-status analysis |
| `autonomy.jsonl` | Optional | Target-centering events when autonomy is launched and records events | Autonomy execution traces; absence must not be read as success or failure unless the manifest shows autonomy was requested |
| `evidence/evidence.jsonl` | Optional | Metadata for captured evidence frames, including frame IDs, capture reasons, timing, and target metadata | Connects visual evidence to run context |
| `evidence/frames/*.jpg` | Optional | Clean captured evidence frames | Visual inspection of scene and detections when paired with metadata |
| `evidence/annotated/*.jpg` | Derived optional | Annotated copies generated from clean frames and evidence metadata | Human review convenience; derived from raw frame evidence |
| `provenance/` | Optional | Small copied inputs such as vocabulary, class list, vision config, and experiment config when available and size-limited; real recorded runs add bridge-emitted `resolved_vision_config.yaml` | Reproducibility context; model binaries are hashed but not copied by default. The resolved artifact captures effective bridge settings after defaults and launch overrides. |
| `logs/bringup.log` | Optional | ROS launch stdout/stderr for recorded runs | Startup, shutdown, and runtime diagnostics |
| `report/index.html` | Derived optional | Static report generated from the raw bundle, annotated evidence, summaries, charts, issues, and artifact links | Human-readable review artifact; raw JSONL and manifest remain the primary evidence |

Conclusions normally require multiple files together. For example, a perception
claim should pair `manifest.yaml` with `detections.jsonl` and `perf.jsonl`; a
native timing claim should include `pipeline_telemetry.jsonl` and the launch/config
context; an autonomy claim should pair manifest launch arguments with
`autonomy.jsonl`, detections, relevant performance data, and terminal evidence
frames when available. A generated report is a useful review surface, but it should
be traceable back to the raw bundle artifacts.

## Evidence Boundaries

Public CI evidence establishes portable build, test, packaging, and documentation
contracts for the covered paths. Local portable verification can reproduce those
checks in a developer environment. Implementation-backed capability identifies code
that exists in the repository. Target-hardware execution claims require a named,
inspectable artifact or measurement from the robot environment.

The public workflows do not prove the entire robot works end to end. They also do
not prove detector accuracy, physical safety behavior, real sensor quality, actuator
response, target-device latency, long-duration stability, or public autonomy
execution. Those claims need evidence records with explicit artifacts, environments,
observed results, inspection status, and limitations.
