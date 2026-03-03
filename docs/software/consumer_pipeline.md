# Consumer pipeline: ready pool -> RKNN -> detections -> release (thread boundary)

This document describes the **per-frame hot path** through the consumer side of the vision pipeline:

**ImageBufferPool latest-ready slot** -> **RKNN inference (NPU)** -> **deterministic postprocess (decode + NMS + inverse letterbox)** -> **publish canonical detections** -> **release pool slot**.

The v1 design goal is strict simplicity:

- **Latest-wins latency policy** (never build consumer queues).
- **One in-flight frame** per consumer thread.
- **No per-frame allocations** in steady state.
- **Minimal status surface** (`status + stage + errno`) instead of error taxonomy sprawl.

---

## Glossary

- **Latest-wins**: consumer always processes the freshest ready slot and ignores stale work.
- **ImageBufferPool**: SPSC handoff between producer and consumer with one atomic ready index.
- **RKNN input binding**: FD-backed tensor binding via `rknn_create_mem_from_fd` + `rknn_set_io_mem`.
- **Remap config**: immutable letterbox geometry (`scale`, `pad_x`, `pad_y`, source/model sizes) from producer preflight.
- **Detections packet**: compact output for downstream consumers (`class_id`, `score`, `bbox_px` + timing ids).

---

## 0) One-time setup (preflight / arm)

`ConsumerPipeline::preflight()` performs all heavy setup once:

1. Validate collaborators:
   - `ImageBufferPool` available
   - RKNN runner available/configured
   - optional telemetry sink
2. Query RKNN model I/O attributes (`n_input`, `n_output`, tensor attrs).
3. Configure required input binding:
   - FD-backed input via `rknn_create_mem_from_fd` + `rknn_set_io_mem`
4. Preallocate all long-lived buffers:
   - RKNN IO memory wrappers
   - output tensor storage (or preallocated output structs)
   - postprocess scratch arrays (`top_k`, `max_det` bounded)
5. Warm-up a small number of inferences.
6. Cache immutable remap geometry in one location (shared pipeline config, not duplicated per frame).

### Runtime observation from local probe

On this RKNN 2.3.0 stack, FD-backed binding worked only when `rknn_create_mem_from_fd` received a **valid mapped `virt_addr`** for the DMA-BUF. Passing `virt_addr = nullptr` failed in probe runs.

---

## 1) Acquire latest ready slot (latest-wins)

Consumer tick starts with:

- `pool.acquire_read(idx)`

Outcomes:

- **No slot ready** -> return `ConsumerTickStatus::NoReadyBuffer`, stage `AcquireRead`.
- **Success** -> read `pool.buffer_at(idx)` and capture per-frame metadata (`sequence`, `capture_ts_real_ns`, `pool_index`), then continue.

**Invariant:** every successful acquire must end in exactly one `publish_release(idx)`.

---

## 2) Infer stage (NPU)

Input is producer-written model-ready RGB/BGR data in DMA-BUF-backed `ImageBuffer`.

### FD-backed binding (required in v1)

- Bind DMA-BUF-backed tensor memory to RKNN input.
- Perform required cache sync (`rknn_mem_sync(..., RKNN_MEMORY_SYNC_TO_DEVICE)`) when needed.
- Call `rknn_run`.

Then:

- Retrieve outputs (`rknn_outputs_get`) into preallocated output storage.

On any inference failure:

- return `ConsumerTickStatus::InferError`, stage `Infer`, set `stage_errno` as available.

No per-frame allocations are allowed in this stage.

---

## 3) Postprocess stage (deterministic)

v1 postprocess is intentionally narrow and bounded:

1. Decode model outputs into fixed-capacity candidate arrays.
2. Apply confidence threshold early.
3. Apply class-wise NMS with deterministic caps:
   - `top_k_per_class`
   - `max_det`
4. Map boxes from model coordinates back to source image coordinates using immutable remap config:
   - `x_src = (x_net - pad_x) / scale`
   - `y_src = (y_net - pad_y) / scale`
   - clamp to `[0, src_w/src_h]`
5. Emit a compact canonical detections packet.

v1 excludes tracking/smoothing from the consumer hot path.

---

## 4) Publish stage (single canonical output)

Consumer publishes one canonical detections result per consumed frame.

v1 rule:

- `ConsumerPipeline` publishes through one interface boundary.
- ROS2/overlay/logging/triggers fan-out downstream, not in the hot path.

This keeps consumer timing isolated from sink backpressure.

---

## 5) Release stage (return slot to pool)

After publish (or on any early-return path after successful acquire):

- `pool.publish_release(idx)`

This transitions the slot back to free for producer reuse.

**Invariant:** release must happen exactly once for each successful acquire.

---

## 6) Telemetry and latency accounting

When telemetry is enabled, emit one `ConsumerSample` with:

- `acquire_read_ns`
- `infer_ns`
- `postprocess_ns`
- `publish_ns`
- `release_ns`
- `total_ns`
- `stage_mask`, `consumer_status`, `infer_status`, `infer_errno`
- `sequence` / `frame_id` when available

Derived metrics:

- consumer total latency
- NPU latency
- postprocess latency
- capture->publish latency (with producer timestamp correlation)
- effective drop/freshness behavior (produced vs consumed)

---

## 7) Failure policy and return contract

- `preflight()` may throw on startup/configuration failures.
- `run()` is `noexcept` and returns `ConsumerTick`.
- v1 keeps statuses minimal:
  - `Consumed`
  - `NoReadyBuffer`
  - `InferError`

Detailed diagnosis comes from:

- `stage` (`AcquireRead`, `Infer`, `Postprocess`, `Publish`, `Release`)
- `stage_errno`
- telemetry stage mask and status fields

---

## Definition of Done (v1)

- [ ] Single-thread consumer loop with latest-wins semantics.
- [ ] One in-flight frame max, no consumer backlog queue.
- [ ] No per-frame allocations in steady state.
- [ ] RKNN preflight includes IO query + warm-up.
- [ ] RKNN input path uses FD-backed `rknn_create_mem_from_fd` + `rknn_set_io_mem`.
- [ ] Deterministic postprocess bounds (`top_k_per_class`, `max_det`).
- [ ] Inverse letterbox mapping implemented and unit-tested.
- [ ] Single canonical publish boundary in consumer.
- [ ] Proper release of acquired pool slots on all exit paths.
- [ ] Telemetry emitted with consumer stage timings and status mask.
- [ ] Clean shutdown (stop loop, release resources, stop publishers).

---

## Compact diagram

```
Consumer thread:

  acquire_read(idx) --> ImageBuffer view (latest ready)
         |
         v
   RKNN infer (fd-backed set_io_mem path)
         |
         v
   decode -> threshold -> NMS -> inverse letterbox
         |
         v
   publish canonical detections
         |
         v
   publish_release(idx)
```
