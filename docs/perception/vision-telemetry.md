# Vision Telemetry Contract

_Status: implemented; native JSONL schema version `3`_

This page defines the current telemetry contract for the native vision pipeline.
The contract covers producer and consumer runtime samples, rolling in-process
statistics, JSONL export, and offline analysis. Telemetry is optional for native
harness runs, but the ROS bridge always keeps rolling telemetry active for live
performance status.

Mission-critical producer and consumer behavior does not depend on telemetry file
I/O. When telemetry is unavailable, faulted, or backpressured, frame capture,
preprocess, inference, postprocess, and publication continue on the normal pipeline
path.

## Emitted Records

The telemetry interface is `omniseer::vision::ITelemetry`:

```cpp
class ITelemetry {
public:
  virtual ~ITelemetry() = default;
  virtual bool timing_enabled() const noexcept = 0;
  virtual void emit_producer(const ProducerSample& sample) noexcept = 0;
  virtual void emit_consumer(const ConsumerSample& sample) noexcept = 0;
};
```

Pipeline loops check `timing_enabled()` once at tick start. When it is false, the
tick skips stage timers and sample emission.

JSONL export writes one JSON object per line. All JSONL records include:

| Field | Meaning |
|---|---|
| `schema_version` | Native telemetry schema version. Current value is `3`. |
| `source` | `producer` or `consumer`. |
| `frame_id` | Producer-assigned frame correlation ID, or `null` before one exists. |
| `tick_id` | Per-thread monotonic telemetry tick ID. Producer and consumer tick ID spaces are independent. |
| `sequence` | Source capture sequence, or `null` before one exists. |
| `event_ts_real_ns` | Realtime event timestamp in nanoseconds. For frame-backed records, this is the capture timestamp propagated from the source frame. |
| `stage_mask` | Bitmask of stages that completed for this sample. |
| `dur_ns` | Object containing per-stage durations plus `total`, all in nanoseconds. |

Producer records are emitted for these tick statuses:

| `producer_status` | Emitted | Notes |
|---|---:|---|
| `produced` | yes | Frame was published to the ready buffer and the capture slot was requeued. |
| `no_writable_buffer` | yes | A frame was dequeued, but no destination buffer was free. |
| `capture_retryable_error` | yes | Capture returned a retryable error. |
| `capture_fatal_error` | yes | Capture returned a fatal error. |
| `preprocess_error` | yes | RGA preprocess returned a non-OK status. |
| `no_frame` | no | Counter-only path; no per-tick JSONL sample is emitted. |

Producer-specific fields:

| Field | Meaning |
|---|---|
| `source_age_dequeue_ns` | Realtime age of the source frame when producer dequeue completed. |
| `source_age_publish_ready_ns` | Realtime age of the source frame when the producer published the ready buffer. |
| `producer_status` | Producer tick result string. |
| `capture_status` | Capture result string. |
| `preprocess_status` | Preprocess result string. |
| `capture_errno` | System errno associated with capture work, or `0`. |

Producer `dur_ns` keys are `dequeue`, `acquire_write`, `preprocess`,
`publish_ready`, `requeue`, and `total`.

Consumer records are emitted for these tick statuses:

| `consumer_status` | Emitted | Notes |
|---|---:|---|
| `consumed` | yes | A ready buffer was acquired, inferred, postprocessed, optionally published, and released. |
| `infer_error` | yes | The consumer was not armed or inference returned a non-OK status. |
| `no_ready_buffer` | no | Counter-only path; no per-tick JSONL sample is emitted. |

Consumer-specific fields:

| Field | Meaning |
|---|---|
| `consumer_start_ts_real_ns` | Realtime timestamp at consumer tick start, or `null` when unavailable. |
| `consumer_end_ts_real_ns` | Realtime timestamp at sample emission, or `null` when unavailable. |
| `source_age_start_ns` | Realtime age of the source frame at consumer tick start. |
| `source_age_end_ns` | Realtime age of the source frame at consumer sample emission. |
| `consumer_status` | Consumer tick result string. |
| `infer_status` | Inference result string. |
| `postprocess_status` | `ok` when postprocess ran, otherwise `not_run`. |
| `infer_errno` | System errno associated with inference work, or `0`. |

Consumer `dur_ns` keys are `acquire_read`, `infer`, `postprocess`, `publish`,
`release`, and `total`.

Example producer record:

```json
{"schema_version":3,"source":"producer","frame_id":42,"tick_id":1,"sequence":9,"event_ts_real_ns":1000,"source_age_dequeue_ns":25,"source_age_publish_ready_ns":37,"producer_status":"produced","capture_status":"ok","preprocess_status":"ok","capture_errno":0,"stage_mask":31,"dur_ns":{"dequeue":10,"acquire_write":11,"preprocess":12,"publish_ready":13,"requeue":14,"total":60}}
```

Example consumer record:

```json
{"schema_version":3,"source":"consumer","frame_id":42,"tick_id":2,"sequence":9,"event_ts_real_ns":1000,"consumer_start_ts_real_ns":1100,"consumer_end_ts_real_ns":1200,"source_age_start_ns":99,"source_age_end_ns":199,"consumer_status":"consumed","infer_status":"ok","postprocess_status":"ok","infer_errno":0,"stage_mask":31,"dur_ns":{"acquire_read":20,"infer":21,"postprocess":22,"publish":23,"release":24,"total":110}}
```

## Queue And Backpressure Behavior

`JsonlTelemetry` owns one bounded single-producer/single-consumer path for producer
samples and one bounded single-producer/single-consumer path for consumer samples.
The default capacity is `512` samples per source and can be overridden through
`JsonlTelemetryConfig`.

Hot-path emission is best effort:

- producer and consumer threads call non-blocking `emit_*` methods;
- each call first checks whether the sink is still enabled;
- when no free telemetry slot is available, the sample is dropped and the
  corresponding internal drop counter is incremented;
- the producer and consumer do not retry, sleep, allocate, or perform file I/O in
  response to telemetry backpressure.

The telemetry worker thread drains ready samples, writes JSONL, returns slots to the
free queues, and flushes after writing. When no records are available, it sleeps for
5 ms before checking again. On destruction, the sink asks the worker to stop, drains
ready samples, flushes, joins, and closes the file.

Drop counters are currently internal to `JsonlTelemetry`; they are not exported in
the JSONL schema.

## Timestamp Semantics

Durations and correlation timestamps use different clock domains:

| Field class | Clock domain | Contract |
|---|---|---|
| `dur_ns.*` and `dur_ns.total` | Monotonic clock | Use for duration and latency calculations within one process. |
| `event_ts_real_ns` | Realtime clock | Use for cross-system correlation with ROS stamps, logs, and RunBundle streams. |
| `consumer_start_ts_real_ns` and `consumer_end_ts_real_ns` | Realtime clock | Use for alignment and source-age calculation, not monotonic duration math. |
| `source_age_*_ns` | Realtime timestamp delta | `0` means the source timestamp was unavailable or later than the sampling timestamp. |

Do not compute stage durations by subtracting realtime fields. The emitted
monotonic durations are the authoritative timing measurements.

`frame_id` is assigned by the producer at `publish_ready`, stored in the image
buffer metadata, and propagated into consumer telemetry. It is the primary
correlation key for joining producer and consumer records for the same processed
frame.

## Status And Duration Fields

Status fields are stable strings derived from bounded native enums:

| Field | Values |
|---|---|
| `producer_status` | `produced`, `no_frame`, `capture_retryable_error`, `capture_fatal_error`, `no_writable_buffer`, `preprocess_error`, `unknown` |
| `capture_status` | `ok`, `no_frame`, `retryable_error`, `fatal_error`, `unknown` |
| `preprocess_status` | `ok`, `invalid_config`, `source_size_mismatch`, `invalid_source_descriptor`, `invalid_destination_descriptor`, `imcheck_failed`, `improcess_failed`, `unknown_error`, `unknown` |
| `consumer_status` | `consumed`, `no_ready_buffer`, `infer_error`, `unknown` |
| `infer_status` | `ok`, `not_armed`, `invalid_input_descriptor`, `rknn_error`, `unknown` |
| `postprocess_status` | `not_run`, `ok`, `unknown` |

`stage_mask` determines which stage durations are present. A duration for a stage
that did not complete is serialized as `null`. `dur_ns.total` is always serialized
as an integer for emitted samples.

Producer stage-mask bits:

| Bit | JSON duration key |
|---:|---|
| `1 << 0` | `dequeue` |
| `1 << 1` | `acquire_write` |
| `1 << 2` | `preprocess` |
| `1 << 3` | `publish_ready` |
| `1 << 4` | `requeue` |

Consumer stage-mask bits:

| Bit | JSON duration key |
|---:|---|
| `1 << 0` | `acquire_read` |
| `1 << 1` | `infer` |
| `1 << 2` | `postprocess` |
| `1 << 3` | `publish` |
| `1 << 4` | `release` |

Source-age fields are not controlled by `stage_mask`. They are serialized as
integers when the backing timestamp field is available to the JSON writer, and
`null` otherwise.

## Failure Behavior

Telemetry is fail-open:

- failure to construct `JsonlTelemetry` because the path is empty, a queue capacity
  is zero, or the file cannot be opened raises during setup;
- after setup, JSONL write failure disables that sink by setting
  `timing_enabled()` false;
- once disabled, later producer and consumer `emit_*` calls return immediately;
- producer and consumer pipeline loops continue running after telemetry sink
  failure;
- telemetry samples may be missing after queue overflow or sink failure.

The native harness only enables telemetry when `--telemetry-jsonl <path>` is
provided. The ROS bridge always uses `RollingTelemetryStats`; when
`telemetry.pipeline_jsonl_path` is set, it also creates `JsonlTelemetry` and fans
samples out through `CompositeTelemetry`.

## Analysis Tools

Use `vision/tools/analyze_telemetry.py` for offline JSONL summaries and
run-to-run comparisons.

Summarize one run:

```bash
vision/tools/analyze_telemetry.py summary runs/<run_id>/pipeline_telemetry.jsonl
```

Compare two runs:

```bash
vision/tools/analyze_telemetry.py compare before.jsonl after.jsonl --label-a before --label-b after
```

The analyzer reports schema versions, producer and consumer sample counts, status
counts, stage latency quantiles, source-age/freshness breakdowns, and frame or
sequence delta behavior. It joins producer and consumer records by `frame_id` when
computing cross-thread freshness metrics.

Run bundles store native pipeline telemetry as `pipeline_telemetry.jsonl` when the
native vision node is launched with a pipeline telemetry path. Run inspection and
report tooling treat this file as optional, but malformed JSONL is reported as an
inspection issue when the file is present.

`RollingTelemetryStats` is the live in-process summary path. It tracks produced,
buffer-pressure, capture-error, preprocess-error, consumed, and infer-error counts,
plus the latest producer, preprocess, consumer, infer, postprocess, publish, and
source-age timings. The native preview overlay and ROS bridge performance messages
read snapshots from this rolling state.

## Verification

Use the smallest supported verification that covers the change:

```bash
scripts/omni test vision
```

This covers portable native telemetry behavior, including JSONL record emission,
schema version `3`, rolling telemetry counters, composite fan-out, stage masks, and
producer/consumer timing sample propagation.

For ROS bridge parameter wiring, RunBundle integration, or launch behavior:

```bash
scripts/omni test ros
```

For target-hardware timing, camera, RGA, or RKNN claims, collect evidence from the
ROCK 5B+ runtime or a RunBundle. Host-side tests verify the portable contract only;
they do not prove target-device frame rate, accelerator latency, camera behavior, or
end-to-end robot runtime timing.
