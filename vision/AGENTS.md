# AGENTS.md

Additional instructions for work under `vision/`.

The repository-root `AGENTS.md` still applies.

## Purpose

`vision/` owns the native camera-to-detection runtime and hardware-independent native replay/comparison support.

Before changing the pipeline, read:

```text
docs/perception/vision-pipeline.md
```

If a change touches model IO, preprocessing, text embeddings, output layout, or RKNN conversion assumptions, also read:

```text
docs/perception/yolo-world-model-deployment.md
```

## Hot-Path Invariants

The production pipeline is optimized for freshness and bounded resource use.

Preserve these properties unless the task explicitly changes the architecture:

- V4L2 capture slots have explicit ownership and must be returned exactly once;
- application model-input buffers have explicit producer/consumer ownership;
- the producer/consumer handoff is bounded and latest-wins rather than an accumulating frame queue;
- implemented hot-path code uses RAII leases so early exits do not leak capture or pool ownership;
- optional sinks must not retain references to borrowed image buffers after the callback returns;
- startup/preflight performs expensive validation and setup; steady-state ticks remain bounded and predictable;
- transient no-frame/no-buffer conditions remain distinguishable from fatal capture, preprocess, or inference failures.

Do not “simplify” ownership code by weakening these guarantees.

## Copies, Buffers, and Hardware Acceleration

Avoid introducing full-frame CPU copies, extra color conversions, per-frame RKNN rebinding, or heap allocation into the steady-state path without a demonstrated need.

Be explicit about memory ownership and layout when touching:

```text
V4L2 -> DMA-BUF -> RGA -> ImageBufferPool -> RKNN -> postprocess
```

Do not claim zero-copy merely because DMA-BUF is present somewhere in the path. Trace the actual transfer and ownership boundary.

Changes to pixel format, stride, layout, dtype, normalization, model input binding, or source/model coordinate remapping are contract changes. Trace them through preprocessing, inference, post-processing, telemetry, replay/probe tools, tests, ROS bridge expectations, and docs as applicable.

## Concurrency

Keep the hot-path concurrency model simple and bounded.

When changing `ImageBufferPool`, producer/consumer handoff, atomics, or lease behavior:

- reason explicitly about ownership states;
- preserve acquire/release ordering requirements;
- avoid blocking the producer behind slow inference merely to preserve every frame;
- add focused regression tests for ownership, latest-wins behavior, and early-exit cleanup.

Skipped frame IDs under load can be expected behavior; do not automatically “fix” them by adding an unbounded queue.

## Failure and Telemetry

Do not hide hardware or pipeline failures by converting them into apparent success.

Preserve useful stage/status telemetry when changing runtime behavior. New failure modes should be observable at the narrowest useful boundary.

Avoid throwing through steady-state hot loops when the existing interface expects structured status returns.

## Offline and Diagnostic Tools

`vision_replay`, raw RKNN probing, and RunBundle comparison tools are validation/diagnostic surfaces, not alternate production pipelines.

Prefer reusing production preprocessing, inference, post-processing, and data contracts instead of creating a second implementation solely for analysis.

Keep raw input artifacts immutable. Write corrected video, comparison output, tensor probes, and reports to explicit derived locations.

Large raw tensor/probe outputs are temporary unless explicitly designated as durable evidence.

## Performance Changes

For changes affecting capture, preprocessing, inference, post-processing, preview/evidence sinks, or buffer handoff, consider:

- producer and consumer rates;
- source age/freshness;
- inference latency;
- dropped/replaced frames;
- CPU and NPU use;
- memory and copies;
- startup/preflight cost.

Measure on the appropriate target when making target-performance claims.

## Verification

Portable native changes:

```bash
scripts/omni test vision
```

If the ROS bridge contract or ROS publications/configuration also change:

```bash
scripts/omni test ros
```

Use target-hardware checks for claims involving V4L2 devices, RGA, RKNN/NPU execution, actual camera behavior, or target latency/throughput.

A portable CMake/CTest pass does not verify those hardware-specific paths.
