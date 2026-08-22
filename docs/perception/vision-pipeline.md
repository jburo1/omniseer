# Vision Pipeline

_Status: implemented; portable checks cover the software contract, while V4L2,
RGA, RKNN, camera, and full ROS graph behavior require target hardware or the
runtime container._

## Purpose and Scope

The vision pipeline converts camera frames into canonical object detections for
Omniseer's perception experiments and bounded target-centering behavior. It is
optimized for robot state freshness: when inference is slower than capture, the
consumer processes the newest preprocessed frame available instead of draining an
old queue.

This page is the reference for the native C++ pipeline and its ROS bridge
adapter. It covers camera capture, RGA preprocessing, DMA-BUF-backed model input
buffers, RKNN inference, YOLO-World post-processing, detection publication,
preview/evidence hooks, telemetry, and failure behavior. It does not describe
autonomy policy, operator dashboards, cloud-side report generation, or model
training. Related telemetry field details live in
[Vision Telemetry](vision-telemetry.md).

## End-to-End Data Path

The implemented data path is:

```text
camera sensor / ISP
  -> V4L2 driver ring slot exported as a DMA-BUF fd
  -> V4l2Capture::dequeue_lease()
  -> FrameDescriptor
  -> RgaPreprocess::run()
  -> ImageBufferPool slot containing RGB888 model input
  -> ImageBufferPool::publish_ready()
  -> ImageBufferPool::acquire_read_lease()
  -> RknnRunner::infer(pool_index)
  -> decode_yolo_world_detections()
  -> DetectionsFrame
  -> IDetectionsSink / IFramePreviewSink / ROS publications
```

`V4l2Capture` owns the camera device and the exported V4L2 DMA-BUF file
descriptors while streaming. Each successful dequeue borrows one kernel-owned
ring slot and exposes it as a `FrameDescriptor`. The producer passes that
descriptor to `RgaPreprocess`, which writes the model-sized RGB image into an
application-owned `ImageBuffer` slot. `RknnRunner` pre-binds every pool slot
during startup, so steady-state inference can select a pool index rather than
constructing RKNN memory bindings per frame.

The pipeline avoids full-frame CPU copies across the capture, preprocess, and
inference handoff by sharing file-descriptor-backed DMA buffers with the hardware
blocks that support them. `DmaHeapAllocator::allocate(int, int, PixelFormat)`
creates the application-owned model input buffers, and `DmabufAllocation` keeps
the backing file descriptor lifetime tied to the pool slot metadata.

The consumer publishes detections as `DetectionsFrame`, whose bounding boxes are
in source-image pixel coordinates. The ROS bridge converts those frames to
`yolo_msgs::msg::DetectionArray` on `/yolo/detections`, publishes rolling
performance summaries on `/vision/perf`, and exposes `/vision/capture_frame` for
evidence capture when an `EvidenceFrameSink` is configured.

## Threading and Ownership Model

The native pipeline has two hot-path workers:

- The producer worker runs `ProducerPipeline::run()` repeatedly. Its stage order
  is `V4l2Capture::dequeue_lease()`, `ImageBufferPool::acquire_write_lease()`,
  `RgaPreprocess::run()`, `ImageBufferPool::WriteLease::publish()`, and
  `V4l2Capture::FrameLease::release()`.
- The consumer worker runs `ConsumerPipeline::run()` repeatedly. Its stage order
  is `ImageBufferPool::acquire_read_lease()`, `RknnRunner::infer()`,
  `decode_yolo_world_detections()`, optional `IFramePreviewSink::publish()` and
  `IDetectionsSink::publish()`, then `ImageBufferPool::ReadLease::release()`.

`VisionBridgeRuntime` in the ROS bridge owns startup orchestration and launches
the producer and consumer threads. `VisionBridgeNode` owns ROS publishers,
service handlers, timers, and process shutdown. `JsonlTelemetry` may own its own
writer implementation behind `JsonlTelemetry::Impl`; callers interact only
through the non-throwing `ITelemetry` interface. `RollingTelemetryStats` uses
atomic counters and last-value fields for periodic summaries.

The hot path is single-producer/single-consumer at the `ImageBufferPool`
boundary. Optional sinks are called synchronously from the consumer path, before
the read lease is released, and must not retain references into the borrowed
`ImageBuffer`.

Both manual and RAII APIs are available at this boundary. The implemented
pipelines use `acquire_write_lease()` and `acquire_read_lease()` so early exits
preserve ownership automatically. Lower-level tests and adapters may use the
manual API when they need direct stage accounting, but each successful
`acquire_write(int&)` must end in `publish_ready(int)` or `cancel_write(int)`,
and each successful `acquire_read(int&)` must end in `publish_release(int)`.

## Pipeline Stages

| Stage | Thread | Primary API | Contract |
| --- | --- | --- | --- |
| Capture | Producer | `V4l2Capture::dequeue_lease()` or `dequeue(FrameDescriptor&)` | Borrow one filled V4L2 slot. A successful borrow must be returned with `FrameLease::release()` or `requeue(uint32_t)`. |
| Preprocess | Producer | `RgaPreprocess::run()` | Convert NV12 source DMA-BUF to RGB888 model input using the configured letterbox geometry. |
| Publish ready | Producer | `ImageBufferPool::WriteLease::publish()` or `publish_ready(int)` | Publish a fully written pool slot as the newest ready model input. |
| Acquire read | Consumer | `ImageBufferPool::acquire_read_lease()` or `acquire_read(int&)` | Atomically claim the newest ready slot, if one exists. |
| Inference | Consumer | `RknnRunner::infer(int32_t)` | Run RKNN inference against a pre-bound pool slot and refresh output views. |
| Postprocess | Consumer | `decode_yolo_world_detections()` | Decode YOLO-World outputs into a bounded `DetectionsFrame`. |
| Publish | Consumer | `IDetectionsSink::publish()` and `IFramePreviewSink::publish()` | Publish detections and optional preview/evidence while the image lease is still valid. |
| Release | Consumer | `ImageBufferPool::ReadLease::release()` or `publish_release(int)` | Return the consumed pool slot to the free ring. |

Startup work is intentionally separated from the hot path. `V4l2Capture::start()`
opens and configures the camera, allocates and exports the V4L2 ring, queues
buffers, and starts streaming. `ImageBufferPool::allocate_all()` creates the
fixed DMA-BUF-backed pool. `RgaPreprocess::prefill()` initializes padding once,
and `RgaPreprocess::preflight()` validates descriptors with a smoke pass.
`ProducerPipeline::preflight()` validates capture/preprocess handoff and records
the source-to-model remap. `RknnRunner::preflight()` loads the RKNN model,
pre-binds image pool slots and static text embeddings, allocates output storage,
and runs warmup passes. `ConsumerPipeline::preflight()` validates text embedding
metadata, model output layout, active class count, and remap geometry.

## Latest-Wins Behavior

`ImageBufferPool` maintains one atomic ready slot, `ready_idx`, plus a
single-producer/single-consumer free ring. `publish_ready(int)` exchanges the
newly written slot into `ready_idx` with release semantics. If an older ready
slot was waiting there, the producer drops it into a producer-local free stash.
That overwrite is the latest-wins policy: a slow consumer does not accumulate
stale model inputs.

`acquire_read(int&)` exchanges `ready_idx` back to `-1` with acquire semantics.
If no slot is ready, it reports failure and the bridge consumer loop sleeps for
1 ms before trying again. If a slot is ready, that slot is owned by the consumer
until `publish_release(int)` or `ReadLease::release()` returns it to the free
ring.

This policy bounds latency by limiting the cross-thread handoff to one pending
frame. It may skip frame IDs and V4L2 sequence numbers under load; those gaps are
expected evidence that old work was discarded to preserve freshness.

## Buffer Lifetime Invariants

V4L2 slots are kernel-owned. Userspace may access a dequeued slot only between
`V4l2Capture::dequeue()` and `V4l2Capture::requeue(uint32_t)`, or while a
`V4l2Capture::FrameLease` is valid. The producer returns the capture slot after
RGA finishes reading it. `FrameLease` provides exactly-once requeue behavior and
attempts release from its destructor if the caller did not release explicitly.

Model input buffers are application-owned by `ImageBufferPool`. A producer may
write a slot only after `acquire_write(int&)` or `acquire_write_lease()` succeeds.
If a write is abandoned, manual callers use `cancel_write(int)` and RAII callers
let `WriteLease` destruct without `publish()`. A consumer may read a slot only
after `acquire_read(int&)` or `acquire_read_lease()` succeeds. Manual callers
must release with `publish_release(int)`. RAII callers release with
`ReadLease::release()`, or by letting the lease destructor run.

Pool slots are addressed by integer index. The index is the ownership token
passed between RGA output, RKNN input selection, telemetry, and tests. Buffer
metadata such as `sequence`, `capture_ts_real_ns`, and `frame_id` is written
before `publish_ready()` so the consumer sees a coherent descriptor.

## Publication and Telemetry Contracts

The native detection contract is `DetectionsFrame`. It carries `frame_id`,
source capture `sequence`, `capture_ts_real_ns`, `active_class_count`,
`source_size`, a bounded detection count, and up to
`DetectionsFrame::capacity` detections. Each `Detection` contains a zero-based
class index, confidence score, and source-space bounding box coordinates.

`IDetectionsSink::publish(const DetectionsFrame&)` is the single native boundary
for detection publication and must not throw. The ROS bridge implements this
boundary in `RosYoloDetectionsSink` and publishes `/yolo/detections`.
`IFramePreviewSink::publish(const ImageBuffer&, const DetectionsFrame&, const
PipelineRemapConfig&)` is a synchronous preview/evidence callback. The image
reference is valid only during that call.

`ITelemetry` exposes `timing_enabled()`, `emit_producer(const ProducerSample&)`,
and `emit_consumer(const ConsumerSample&)`. `ProducerSample` records frame
identity, source age, dequeue/acquire/preprocess/publish/requeue durations,
completed `ProducerStageMask` bits, errno, and status codes. `ConsumerSample`
records frame identity, source age, acquire/infer/postprocess/publish/release
durations, completed `ConsumerStageMask` bits, errno, and status codes.
`RollingTelemetryStats` feeds `/vision/perf`; `JsonlTelemetry` writes the
optional `telemetry.pipeline_jsonl_path`; `CompositeTelemetry` fans samples out
to both when configured.

## Failure Behavior

Startup validation is fail-fast. Configuration errors, missing model or class
inputs, camera setup failures, DMA-BUF allocation failures, RGA preflight
failures, RKNN setup failures, and invalid consumer startup metadata are reported
by throwing from startup or `preflight()` calls. `VisionBridgeNode` logs startup
exceptions as fatal and exits with failure.

Steady-state producer and consumer ticks are non-throwing and return structured
status. The bridge treats `NoFrame`, `CaptureRetryableError`,
`NoWritableBuffer`, and `NoReadyBuffer` as transient and sleeps for 1 ms before
retrying. `CaptureFatalError`, `PreprocessError`, and `InferError` are fatal in
`VisionBridgeRuntime`: the first fatal message is recorded, stop is requested,
and the node health timer logs the message and shuts ROS down.

No failure path should bypass buffer ownership obligations. RAII leases are used
in the implemented producer and consumer paths so camera slots and pool slots are
returned when early exits occur.

## Verification Boundary

Use the narrowest check that matches the change. Portable native pipeline changes
are covered by:

```bash
scripts/omni test vision
```

ROS bridge contracts, parameters, publications, and service behavior are covered
by:

```bash
scripts/omni test ros
```

Documentation-only changes are covered by:

```bash
scripts/omni docs build
```

These checks do not prove live camera, RGA, RKNN, ROCK 5B+, or full runtime
container behavior unless they are run in an environment with the corresponding
hardware, SDKs, devices, and ROS graph.

## Offline Detector Replay

`vision_replay` is a target-side, sequential detector replay tool. It decodes one MP4,
CPU-letterboxes each source frame into the existing 640×640 RGB DMA-backed input pool,
then uses the normal `ConsumerPipeline`, `RknnRunner`, and YOLO-World postprocess path.
It writes exactly one canonical JSONL record for every decoded frame, including frames
with no detections. `frame_index` is the replay identity; the JSONL contains no wall-clock
or inference-latency fields.

This is an offline validation tool and is not part of the camera/ROS production runtime.

## Primary Implementation Files

- `vision/include/omniseer/vision/v4l2_capture.hpp` and
  `vision/src/v4l2_capture.cpp`
- `vision/include/omniseer/vision/rga_preprocess.hpp` and
  `vision/src/rga_preprocess.cpp`
- `vision/include/omniseer/vision/image_buffer_pool.hpp` and
  `vision/src/image_buffer_pool.cpp`
- `vision/include/omniseer/vision/rknn_runner.hpp` and
  `vision/src/rknn_runner.cpp`
- `vision/include/omniseer/vision/producer_pipeline.hpp` and
  `vision/src/producer_pipeline.cpp`
- `vision/include/omniseer/vision/consumer_pipeline.hpp` and
  `vision/src/consumer_pipeline.cpp`
- `vision/apps/vision_replay.cpp`
- `vision/include/omniseer/vision/letterbox.hpp` and `vision/src/letterbox.cpp`
- `vision/include/omniseer/vision/replay_jsonl.hpp` and `vision/src/replay_jsonl.cpp`
- `vision/include/omniseer/vision/dma_heap_alloc.hpp`
- `vision/include/omniseer/vision/yolo_world_postprocess.hpp` and
  `vision/src/yolo_world_postprocess.cpp`
- `vision/include/omniseer/vision/detections.hpp`
- `vision/include/omniseer/vision/detections_sink.hpp`
- `vision/include/omniseer/vision/frame_preview_sink.hpp`
- `vision/include/omniseer/vision/telemetry.hpp`
- `vision/include/omniseer/vision/rolling_telemetry.hpp`
- `vision/include/omniseer/vision/jsonl_telemetry.hpp`
- `vision/include/omniseer/vision/composite_telemetry.hpp`
- `ros_ws/src/omniseer_vision_bridge/src/vision_bridge_runtime.cpp`
- `ros_ws/src/omniseer_vision_bridge/src/vision_bridge_node.cpp`
