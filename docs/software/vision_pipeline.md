“Frames come from V4L2. We allocate a ring of DMA-capable buffers and run the capture device in streaming mode with enqueue/dequeue semantics. Each dequeued frame is handed off downstream without memcpy by sharing the same underlying allocation (DMA-BUF) into the hardware preprocessor (RGA) and then into the NPU runtime. A lock-free/SPSC queue carries buffer handles across threads, and backpressure is enforced by queue depth so we don’t drop frames or blow latency. We measure end-to-end latency per frame with timestamps at capture, post-preprocess, and post-inference.”

# Vision Pipeline

This document describes a multi-stage, low-latency, zero copy-ish, real-time computer vision pipeline for the Rockchip RK3588 SoC. The pipeline transforms raw pixel data acquired by a camera into a format that is compatible with downstream neural network inference.

#TODO when online, write some measurements here, e.g. inference achieves xfps at Y format here, maybe link to profiler JSONL dump etcs

## Major Design Considerations

- Latency-first: pipeline is "latest-wins": consumer always takes freshest processed frame, older frames are dropped.

- Accelerator-first: all suitable ops are offloaded to ISP/RGA/NPU instead of clogging CPU: ISP/V4L2 delivers 1280x720 NV12 formatted images at 60 FPS, RGA does image resizing/letterboxing/colorspace manipulatoin to ready model input, NPU does inference.

- Zero-copy-ish: direct memory access buffers (DMA-BUF) connect pipeline stages to avoid CPU memcpys of full frames.

- Multithreaded: two threads operate the pipeline, a producer owns image capture and preprocess, and a consumer owns inference + postprocess.

- Observable: performance information emitted at every stage.

## Stages

| Stage | Thread | Input | Output | Primary Code |
|---|---|---|---|---|
| Capture (V4L2) | Producer | `/dev/video*` | `FrameDescriptor` (borrowed V4L2 slot) | `v4l2_capture.hpp/cpp` |
| Preprocess (RGA) | Producer | `FrameDescriptor` (NV12 DMA-BUF) | `ImageBuffer` (RGB DMA-BUF) | `rga_preprocess.hpp/cpp` |
| Buffering/Drop policy | Cross-thread boundary | Pool indices | “latest wins” ready buffer index | `image_buffer_pool.hpp/cpp` |
| Inference (NPU) | Consumer | `ImageBuffer` (RGB DMA-BUF) | model outputs | `vision/include/omniseer/vision/rknn_runner.hpp` (TODO), `vision/src/rknn_runner.cpp` (TODO) |
| Postprocess/Publish | Consumer | model outputs | ROS msgs, telemetry | TODO |

## Diagram

```
                         photons
                            |
                            v
                        +-------+
                        |radxa 4k camera       |
                        |       |
                        +-------+
                            |
                            v

  +----------------------------------------------+
  | Camera / ISP -> V4L2 N slot capture ring     |
  |   exported as DMA-BUF fd per slot via EXPBUF |
  +--------------------------+-------------------+
                             |
                             v  VIDIOC_DQBUF (nonblocking)
                  +----------+-----------+
                  | V4l2Capture          |
                  | owns: slot DMA-BUF fds|
                  +----------+-----------+
                             |
                             | FrameDescriptor (borrow slot i)
                             v
                  +----------+-------------------------+
                  | RgaPreprocess (librga / im2d)      |
                  | NV12 (DMA-BUF) -> RGB888 (DMA-BUF) |
                  | mode: Letterbox                   |
                  +----------+-------------------------+
                             |
                             | publish_ready(pool_idx)
                             v
                  +----------+------------------+
                  | ImageBufferPool (SPSC)      |
                  | free_ring + ready_idx       |
                  | policy: latest wins         |
                  +----------+------------------+
                             |
                             v acquire_read(pool_idx)
                  +----------+-----------+
                  | RKNN Runner          |
                  | reads RGB888 DMA-BUF |
                  +----------+-----------+
                             |
                             v
                    model outputs / detections
                             |
                             v
                      ROS publish / logging

  TODO: create a nicer visual than this simple ascii
```

## Interfaces

### Core data types

Defined in `types.hpp`:

- `FrameDescriptor`: content description of a V4L2 ring buffer slot (owned by the kernel driver). Handle for ISP output/RGA input.
- `ImageBuffer`: DMA-BUF fd-backed buffer (owned by the application). Handle for RGA output/RKNN(NPU) input.

### Capture: `V4l2Capture`

Manages the V4L2 (Video for Linux 2) streaming lifecycle and defines a borrow-token style API for accessing ISP output from downstram devices in a zero-copy fashion.

Defined/implemented in `v4l2_capture.hpp/cpp`.

- `start()`:
  - Opens device path (e.g. `/dev/video12`), negotiates image format, allocates a driver-managed ring buffer,
    exports each slot as a DMA-BUF fd, queues all slots, and starts streaming.

- `dequeue(FrameDescriptor& out)`:
  - Dequeues the most recently filled V4L2 ring slot, populates `out` with a borrow-token (v4l2_index, DMA-BUF fd, layout, metadata). Thse caller must, after performing its work, `requeue(out.v4l2_index)` to return the slot to the driver so it can refill it with a fresh frame.

- `requeue(uint32_t index)`:
  - Return the specified V4L2 ring buffer slot at `index` to the driver so it can be filled with a fresh frame.

### Preprocess: `RgaPreprocess`

Manages the RGA (2D blitter) transformations from ISP output to correct model input.

Defined/implemented in `rga_preprocess.hpp/cpp`.

- `run(const FrameDescriptor& src_nv12, ImageBuffer& dst_rgb, ...)`:
  - Runs the RGA hardware pipeline to convert a captured NV12 DMA-BUF frame into a RGB888 destination buffer. Synchronous.

- `prefill(ImageBuffer& dst_rgb)`:
  - The RK3588's RGA device does not support colorfilling a RGB888 buffer, so callers must do it themselves. This must only be called once at buffer init time.


### Buffering: `ImageBufferPool`

This is the boundary between the producer and consumer threads. It facilitates the data handoff between the RGA output and the NPU input in a lock-free fashion. It implements a "freshest-first" policy, where the consumer only has access to the latest processed image.

Defined/implemented in `image_buffer_pool.hpp/cpp`

The usage is as follows:

- Producer: `acquire_write()` -> RGA writes -> `publish_ready()`
- Consumer: `acquire_read()`  -> RKNN reads -> `publish_release()`

- `acquire_write(int& idx)`
  - Obtains a free buffer index into `idx` for the producer to write into.

- `publish_ready(int idx)`
  - Publishes `idx` as the newest ready buffer.
  - Should be called after performing the write.

- `acquire_read(int& idx)`
  - Atomically grabs the currently ready buffer index into `idx`.

- `publish_release(int idx)`
  - Returns the consumed buffer index `idx` back to the pool.
  - Should be called after performing the read + consumption.

- `buffer_at(int idx)`
  - Accessor function for buffer at `pool[idx]`
  - Required to access data once ownership established
  - Comes in non-const/const flavours for producer/consumer

### Buffer Allocation: `DmaHeapAllocator` + `DmaHeapAllocation`

"Video malloc" allocator + allocation classes that create shareable RGB image buffers for zero-copy-ish data movement between RGA and RKNN. Resource-safe bridge between kernel memory and accelerators.

Defined and implemented in `dma_heap_alloc.hpp/cpp`.

`DmaHeapAllocator`:
- `DmaHeapAllocator()`
  - Create factory

- `allocate(int width, int height, PixelFormat fmt)`
  - Allocate a DMA-BUF suitable for RGA write / RKNN read and return an ImageBuffer
  and descriptor that points at it.



## Ownership & Lifetime Rules for Buffers

### V4L2 ring slots (`FrameDescriptor`)

- Owned by: kernel driver.
- Userspace handle lifetime:
  - The exported DMA-BUF fds are owned by `V4l2Capture` for the duration of streaming.
  - Each `dequeue()` borrows one ring slot at index `v4l2_index`.
  - A `requeue(v4l2_index)` must occur for every successful `dequeue()` to allow slot to be refilled. This should happen after RGA is finished using the buffer.

### Model input buffers (`ImageBufferPool`)

- Owned by: `ImageBufferPool` (backing `DmabufAllocation`s are RAII).
- Cross-thread rule:
  - Producer may only write to a buffer index after `acquire_write(idx)` returns true.
  - Consumer may only read from a buffer index after `acquire_read(idx)` returns true.
  - Consumer must call `publish_release(idx)` once it is done reading.

## Overview of Producer Responsibilities

Own the upstream clock: drive the loop cadence (dequeue frames) and decide when to drop work to maintain “latest-wins” latency.

Dequeue from V4L2: call DQBUF, receive the newest captured slot, and package it into a FrameDescriptor (fd(s), strides, w/h, format, timestamp, slot index).

Respect V4L2 slot lifetime: treat the dequeued slot as borrowed from the driver; do not hold it longer than necessary.

Acquire an output buffer: get a writable ImageBuffer slot from ImageBufferPool::acquire_write(idx) (or decide to skip processing if none are available).

Run preprocess on accelerators: invoke RGA to transform NV12 DMA-BUF → RGB/BGR DMA-BUF, including resize + letterbox/stretch policy, and produce LetterboxMeta if needed.

Write output metadata: fill ImageBuffer fields (fd, stride, dims, pixel format, timestamp, letterbox params, sequence number).

Publish the newest buffer: call publish_ready(idx) with release semantics so the consumer sees a fully-written frame.

Recycle old ready frames: if publish_ready “steals” the previous ready buffer (because latest-wins), ensure it goes back into the producer/free path so buffers don’t leak.

Return camera buffers promptly: QBUF the V4L2 slot back to the driver as soon as RGA is done with it (or immediately if you drop the frame).

Maintain steady-state buffer hygiene: one-time prefill/padding initialization for destination buffers (your RGB888 imfill limitation means you may do a CPU prefill fallback).

Instrumentation: emit per-stage timings (DQBUF wait, RGA submit/complete, publish cost), drop counters, and queue depths.

Error containment: handle transient failures (EINTR/EAGAIN, occasional RGA errors) without wedging the pipeline; perform clean shutdown (stop streaming, close fds, free allocations).

## Consumer Responsibilities

- Acquire the newest frame (latest-wins)

- Run RKNN inference (NPU)

Initialize RKNN once at startup (load .rknn, init runtime).

Per frame:

Feed the input tensor (usually uint8 NHWC or NCHW depending on how you exported/configured).

Call inference.

Read output tensors.

Why this structure matters: your inference FPS will be lower than camera FPS, so consumer naturally drops frames and always processes “most recent state,” which is exactly what you want for robotics.

- Postprocess (CPU, usually)

Decode YOLO head outputs → candidate boxes + scores + class ids

Apply thresholding + NMS (non-max suppression)

Undo letterbox/resize to map boxes back to 1280×720 (or whatever your original frame is)

- Publish results downstream

Provide a simple struct like:

timestamp, list of {class_id, score, x1,y1,x2,y2} in original image coordinates

Feed tracking / “seek-and-capture” logic.

- Release buffer

pool.release(idx) so RGA can reuse it.

A minimal consumer loop looks like:

acquire → infer → decode → publish → release
(no queue buildup, no waiting on stale frames)

## Threading Model (Current Intended)

Two threads:

1) Capture/Preprocess thread (producer):
   - `cap.dequeue(frame)` (nonblocking loop/poll)
   - `pool.acquire_write(idx)`; if false, immediately `cap.requeue(frame.v4l2_index)` and continue
   - `rga.run(frame, pool.buffer_at(idx), &meta)`
   - `pool.publish_ready(idx)`
   - `cap.requeue(frame.v4l2_index)`

2) Inference thread (consumer):
   - `pool.acquire_read(idx)` (nonblocking loop/condition variable)
   - `rknn.infer(pool.buffer_at(idx), ...)` (TODO)
   - `pool.publish_release(idx)`

Notes:
- The pool policy intentionally drops frames if inference cannot keep up (“latest wins”).
- `V4l2Capture::dequeue()` is currently nonblocking; production code should prefer `poll()`/`select()`
  over spin-sleep loops.

## Failure Modes & Handling

### Capture

- `start()` throws:
  - bad device path / permissions
  - missing V4L2 streaming or MPLANE caps
  - driver rejects requested size or format
  - ioctl failures (REQBUFS/QUERYBUF/EXPBUF/QBUF/STREAMON)

- `dequeue()`:
  - returns `false` on `EAGAIN` (no frame ready)
  - throws on other errors

Recommended handling:
- Treat `start()` failures as fatal at node startup (log and exit or retry with backoff).
- Treat repeated dequeue `EAGAIN` as “no data”; use `poll()` for readiness.
- Any early exit after a successful dequeue must still `requeue(v4l2_index)`.

### Preprocess (RGA)

`run()` returns `false` if:
- config is invalid (dst_w/dst_h <= 0)
- input descriptor is not NV12 or does not match the contiguous NV12 layout assumptions
- destination buffer is invalid (fd/stride mismatch)
- librga rejects parameters (`imcheck` failure) or processing fails (`improcess` < 0)

Recommended handling:
- On preprocess failure, log once per N frames, drop the frame, and continue:
  - `cap.requeue(v4l2_index)` must still happen.
  - If the destination pool index was acquired, it should be returned to the pool (either via
    a dedicated “abort” path or by publishing/releasing consistently).

### Buffer Pool

- Producer `acquire_write()` can fail if the consumer has not released any buffers and the
  producer stash is empty.
- “Latest wins” means drops are expected under load; this is not an error.

Recommended handling:
- If `acquire_write()` fails, drop the current frame (requeue immediately) to protect latency.

### Inference (RKNN) — TODO

Expected failure classes:
- model load failures (missing `.rknn`, incompatible runtime)
- tensor layout mismatch (RGB vs BGR, NCHW vs NHWC, quantization)
- device/runtime errors during inference

Recommended handling:
- Fail fast at init if the model cannot be loaded.
- If inference fails mid-run, drop that frame and continue (don’t stall capture).

## Hardware/Platform Assumptions

- V4L2 node exposes NV12 in a single exported DMA-BUF allocation with UV data located at
  `stride_bytes * height`. This is validated in `RgaPreprocess` and in the V4L2/RGA tests.
- librga (im2d) is available and functional (`/dev/rga` present on target).
- libdrm is available and accessible for dumb-buffer DMA-BUF allocation.

## Current Repo Status (as of 2026-01-27)

- Implemented and tested (hardware-dependent tests may skip at runtime):
  - `V4l2Capture`
  - `DrmDmabufAllocator`
  - `ImageBufferPool` + `SpscRing`
  - `RgaPreprocess` (NV12 -> RGB888, letterbox)
- Not implemented yet:
  - Pipeline orchestration (`vision/src/pipeline.cpp`)
  - Profiling implementation (`vision/src/profiler.cpp`)
  - RKNN runner (`vision/src/rknn_runner.cpp`)
  - A real `vision_harness` executable (currently a stub placeholder)
