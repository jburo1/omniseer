# Vision Telemetry and Profiling Spec (v1)

## Status

- Owner: vision pipeline implementation
- Version: `v1`
- Scope: producer + consumer runtime telemetry, emitted asynchronously
- Last updated: 2026-02-17

## 1) Purpose

Define a low-coupling, low-overhead telemetry architecture for the vision pipeline that:

- keeps producer and consumer hot paths non-blocking
- keeps profiling optional at runtime
- emits structured stage timing only when telemetry is active
- supports real-time and offline latency/jitter analysis
- preserves detector-stage latency/FPS under load
- avoids rework when consumer telemetry is added

## 2) Design Goals

1. **Optional profiling**:
- if telemetry is disabled, no stage timing work is done
- pipelines still keep cheap status counters

2. **No data-plane stalls**:
- producer and consumer must never block on telemetry
- queue overflow drops telemetry samples, never frames
b
3. **Loose coupling**:
- pipelines depend on a small telemetry interface, not sink details

4. **Cross-thread safety**:
- producer and consumer each publish to telemetry thread through bounded queues

5. **ROS2-friendly timestamps**:
- monotonic durations for measurement correctness
- realtime timestamp for cross-system correlation

6. **Low and predictable overhead**:
- fixed-shape samples on hot path (no `std::optional`, no formatting)
- one top-level timing gate per tick, then scoped stage timing inside that branch

## 3) Non-Goals (v1)

- perfect reliability of telemetry delivery
- dynamic attach/detach of telemetry while running
- profiling of preflight/startup path
- multi-process telemetry transport
- high-cost online analytics UI inside data-plane threads

## 4) Threading Model

The system has up to four threads:

1. **Orchestrator/App thread**
- owns lifecycle of telemetry hub and pipeline threads

2. **Producer thread**
- runs producer pipeline tick loop
- emits producer telemetry samples asynchronously

3. **Consumer thread**
- runs consumer pipeline tick loop
- emits consumer telemetry samples asynchronously

4. **Telemetry thread**
- drains producer and consumer queues
- writes JSONL sink
- computes optional rolling stats snapshots

## 5) Architecture

### 5.1 Interface boundary

Pipelines depend on one minimal interface (name can be adjusted):

```cpp
class ITelemetry {
public:
  virtual ~ITelemetry() = default;

  // True only when stage timing + sample emission are active.
  virtual bool timing_enabled() const noexcept = 0;

  // Always non-blocking, never throws, best-effort.
  virtual void emit_producer(const ProducerSample& sample) noexcept = 0;
  virtual void emit_consumer(const ConsumerSample& sample) noexcept = 0;
};
```

### 5.2 Nullability and activation

- `ITelemetry* telemetry == nullptr` means telemetry integration is absent.
- `telemetry != nullptr && telemetry->timing_enabled() == true` means stage timing + sample emission are active.
- Counters do not require active timing.

### 5.3 Counters vs samples

1. **Always-on cheap counters**:
- per-pipeline status counters (e.g., produced, no-frame, preprocess-error)
- no formatting, no allocations, no file writes

2. **Conditional timing + samples**:
- only when `timing_enabled() == true`
- stage duration measurement + sample object creation + queue push

### 5.4 Hot-path sample representation

For producer/consumer in-memory sample structs:

- use fixed-width fields (`uint64_t` durations default to `0`)
- include `stage_mask` bitset indicating which stages executed
- avoid `std::optional` and avoid formatting work in data-plane threads
- JSON conversion (`stage_mask` -> `null` for missing stage) happens only in telemetry thread

### 5.5 Timing instrumentation style

Implementation pattern for each pipeline tick:

1. Evaluate `timing_on = (telemetry != nullptr && telemetry->timing_enabled())` once at tick start.
2. If `timing_on == false`, run pipeline logic with counters only.
3. If `timing_on == true`, use scoped RAII stage timers that write directly into sample fields.

This keeps instrumentation readable while preserving near-zero disabled-path overhead.

## 6) Queueing and Backpressure

### 6.1 Queue topology

- producer -> telemetry thread: one bounded SPSC queue
- consumer -> telemetry thread: one bounded SPSC queue

This avoids MPSC complexity and aligns with current lock-free SPSC primitives.

### 6.2 Queue behavior

- `try_push` only; never block producer/consumer
- on full queue: drop sample and increment a drop counter
- no retries on hot path

### 6.3 Sink behavior

- telemetry thread batches writes to sink (JSONL)
- flush by size and/or time interval
- sink I/O must not run on producer/consumer threads

### 6.4 Capacity sizing rule

- size each SPSC queue from worst burst and sink pause budget:
  - `capacity >= worst_burst_rate_hz * max_sink_pause_s`
- default v1 starting point: `512` per queue unless measured data suggests otherwise

## 7) Failure Policy

v1 uses **fail-open** policy:

- telemetry/sink errors must not stop producer/consumer pipeline execution
- on sink fault, telemetry hub marks itself faulted, increments sink-error counter, and disables timing emission
- pipeline keeps running with cheap counters

Rationale: pipeline availability is prioritized over telemetry completeness.

## 8) Producer Emission Rules (v1)

### 8.1 Tick inclusion

- Emit samples for:
  - `Produced`
  - `NoWritableBuffer`
  - `CaptureRetryableError`
  - `CaptureFatalError`
  - `PreprocessError`
- Do **not** emit per-tick `NoFrame` sample by default (counter-only), to avoid high-rate log noise.

### 8.2 Identifiers

- `frame_id`: producer-assigned monotonic ID at `publish_ready`, stored in buffer metadata, and propagated into consumer telemetry
- `tick_id`: always present (monotonic counter in producer thread)
- `sequence`: present when available from captured frame

`frame_id` is the cross-thread correlation key for end-to-end latency analysis.

### 8.3 Stage partitioning

Producer stage durations are partitioned as:

1. `dequeue_ns`
2. `acquire_write_ns`
3. `preprocess_ns`
4. `publish_ready_ns`
5. `requeue_ns`
6. `total_ns`

If a stage did not execute due to early return, stage duration is `null` in JSON.

### 8.4 Early-return semantics

- Early returns are normal outcomes, not process-fatal.
- Sample contains status fields that explain path taken.

## 9) Clock and Time Semantics

1. Stage durations:
- measured from monotonic clock
- stored as nanoseconds

2. Event timestamp:
- realtime nanoseconds for cross-correlation with ROS2 stamps/logs

3. Duration math:
- always uses monotonic domain
- never compute duration from realtime values

## 10) Data Contract (JSONL v1)

Each line is one JSON object. Required top-level fields:

```json
{
  "schema_version": 1,
  "source": "producer",
  "frame_id": 4421,
  "tick_id": 12345,
  "sequence": 9988,
  "event_ts_real_ns": 1739700000000000000,
  "producer_status": "produced",
  "capture_status": "ok",
  "preprocess_status": "ok",
  "capture_errno": 0,
  "dur_ns": {
    "dequeue": 12000,
    "acquire_write": 900,
    "preprocess": 1450000,
    "publish_ready": 700,
    "requeue": 11000,
    "total": 1490000
  }
}
```

Notes:

- `frame_id` may be `null` for paths that never reached `publish_ready`.
- `sequence` may be `null` for paths without a valid dequeued frame.
- any non-executed stage in `dur_ns` is `null`.
- consumer events use `source: "consumer"` and consumer-specific status/stage fields.

## 11) Interface and Ownership

### 11.1 Ownership

- app/orchestrator owns telemetry hub and sink objects
- producer/consumer receive non-owning pointer/reference to interface

### 11.2 Lifetime

- telemetry hub is created before starting producer/consumer loops
- telemetry hub is stopped and joined after producer/consumer loops exit

### 11.3 No runtime toggling (v1)

- telemetry configuration is fixed for process lifetime
- no hot attach/detach requirement

## 12) Flush and Shutdown Policy

1. Flush trigger:
- batch size threshold (e.g., `N` samples), or
- periodic timer threshold (e.g., `T` milliseconds)

2. Shutdown:
- stop producer and consumer loops
- drain remaining telemetry queues
- flush sink
- stop telemetry thread and join

## 13) Performance Requirements

When telemetry inactive:

- no stage clock calls
- no event allocations
- no queue operations
- only cheap status counters

When telemetry active:

- one fixed-shape sample build per emitted tick (stack/local object; no heap required)
- one non-blocking queue push attempt per sample
- stage timing guarded by a single top-level `timing_on` branch
- no blocking I/O on data-plane threads

## 14) Minimal Test Requirements

1. Interface disabled path:
- no timing samples emitted when telemetry is null or inactive

2. Producer stage coverage:
- produced path populates expected durations and statuses
- early-return paths correctly set missing stages to null

3. `frame_id` propagation:
- producer assigns `frame_id` at publish
- consumer sample for same image carries the same `frame_id`

4. Queue overflow:
- full queue causes sample drop counter increment, no blocking

5. Sink failure:
- sink error transitions telemetry to faulted/disabled mode
- producer/consumer loops continue

6. Schema stability:
- JSON keys/types match spec and remain backward compatible within v1

## 15) Suggested Implementation Slices

1. Introduce telemetry interface + no-op/null behavior + cheap counters.
2. Add `frame_id` to image metadata and propagate it producer -> consumer.
3. Implement telemetry hub thread with dual SPSC ingest queues.
4. Implement JSONL sink with batched flush.
5. Wire producer/consumer stage timing with top-level `timing_on` gate + RAII scoped timers.
6. Add tests for inactive/active/overflow/fault/`frame_id`/schema.

## 16) Open Questions for v2+

- strict mode option (fail-closed) for CI/debug builds
- online p50/p95/p99 export endpoint
- compression/rotation strategy for long-running JSONL logs
- additional queue/buffer pressure fields if needed for debugging
