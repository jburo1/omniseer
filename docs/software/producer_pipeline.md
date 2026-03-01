# Producer pipeline: camera → RGA → output pool (thread boundary)

This document describes the **per-frame hot path** through the producer side of the vision pipeline:

**V4L2 camera capture (kernel ring)** → **DMA-BUF-backed source frame** → **RGA preprocess (src→dst blit/convert/resize/letterbox)** → **publish into output buffer pool** (handoff to consumer).

The core design pattern is **scope-bound capability tokens** (RAII) enforcing protocols:

- **Borrow (camera ring slot):** `DQBUF` must pair with exactly one `QBUF`.
- **Lease (pool slot):** `acquire_write` must end in either `publish()` (commit) or `cancel` (rollback).
- **Own (fd lifetime):** `DmabufAllocation` owns/close()s the DMA-BUF fd.

---

## Glossary

- **DMA-BUF fd**: a file descriptor that refers to a shareable memory allocation (potentially used by multiple devices).
- **FrameDescriptor**: *non-owning metadata view* over the camera DMA-BUF (format, strides, offsets, dims, timestamp).
- **ImageBuffer**: *non-owning metadata view* over an output DMA-BUF (format, strides, dims).
- **FrameLease**: a move-only token representing “I currently hold a dequeued camera ring slot and must requeue it.”
- **WriteLease**: a move-only token representing “I have exclusive write access to one output pool slot until I publish or cancel.”

---

## 0) One-time setup (build the rings)

### Camera side (kernel-owned ring)
Typical V4L2 streaming setup (conceptual):

1. `open()` device
2. `VIDIOC_S_FMT` (e.g., NV12)
3. `VIDIOC_REQBUFS` (N capture slots)
4. For each slot:
   - `VIDIOC_QUERYBUF` (get size/index)
   - optionally `VIDIOC_EXPBUF` (export a DMA-BUF fd for that slot)
   - `VIDIOC_QBUF` (enqueue for capture)
5. `VIDIOC_STREAMON`

Result: the kernel owns a ring of **N capture slots**. Your app temporarily borrows slots between `DQBUF` and `QBUF`.

### Output side (app-owned pool)
Allocate **M output buffers** up front (DMA-HEAP → DMA-BUF fds), store them in `ImageBufferPool`.

Each pool slot is managed by a small state machine (conceptual):

- `Free` → (producer acquire) → `Writing` → (publish) → `Published` → (consumer acquire) → `Reading` → (release) → `Free`

This pool is the **thread boundary** between producer and consumer.

---

## 1) Dequeue a camera frame (borrow a kernel ring slot)

Producer calls `dequeue_lease()` which performs `VIDIOC_DQBUF` internally.

Possible outcomes:

- **EAGAIN / no frame ready** → `NoFrame` (idle tick)
- **Success** → you receive a `v4l2_buffer` with:
  - `index` (which ring slot)
  - timestamp/sequence metadata
  - bytesused/per-plane info (driver-dependent)

On success, you create:

- `FrameDescriptor` (metadata view: fd + layout + dims + timestamp)
- `FrameLease` (borrow token: must requeue this `index` exactly once)

**Invariant:** Every successful `DQBUF` must be paired with exactly one `QBUF` of the same `index`.

---

## 2) Acquire an output buffer (lease exclusive write access)

Producer calls `acquire_write_lease()` on `ImageBufferPool`.

On success you receive a `WriteLease` token with:

- destination `ImageBuffer` view (fd + layout + dims)
- exclusive right to write into that pool slot

If the producer exits early (return/error/throw), `WriteLease` destructor must roll back:

- if `publish()` was not called → `cancel_write()` → slot returns to `Free`

**Invariant:** Every successful write-acquire ends in exactly one of:
- `publish()` (commit)
- auto-cancel on destructor (rollback)

---

## 3) Synchronization / coherency (DMA-BUF “physics”)

Even without CPU touching pixels, the devices must observe correct ordering:

- RGA must not read **src** before the camera finished writing it.
- Consumer must not read **dst** before RGA finished writing it.

Two broad approaches:

- **Implicit sync** (fences handled by drivers)
- **Explicit sync** via `DMA_BUF_IOCTL_SYNC` (start/end for read/write)

Whether explicit sync is required depends on the exact driver stack and memory path, but this stage is where you put those boundaries if needed.

---

## 4) RGA preprocess (the data movement)

Inputs:

- **src**: `FrameDescriptor` describing camera DMA-BUF (often NV12, multi-plane)
- **dst**: `ImageBuffer` describing pool DMA-BUF (often RGB/BGR, model-ready)

RGA performs a hardware blit with operations like:

- colorspace conversion (NV12 → RGB/BGR)
- resize / scale
- crop
- letterbox/padding (fit aspect ratio into model input size)
- rotation (if needed)

Conceptually:

```
RGA:
  src(dmabuf fd + offsets/strides + WxH + fmt)
    --> dst(dmabuf fd + offsets/strides + W'H' + fmt)
```

At the end of this stage, the transformed pixels physically reside in the **destination pool slot**.

---

## 5) Publish (commit the write lease; cross the thread boundary)

If RGA succeeds, the producer calls `dst_lease.publish()`.

A good `publish()` typically does:

1. Attach/finalize per-frame metadata:
   - timestamp/sequence
   - original dims and model dims
   - letterbox transform parameters (scale + pad offsets), if used
2. Transition state: `Writing → Published`
3. Notify consumer (optional): condvar/eventfd or “latest index” atomic

**This is the exact moment the buffer becomes visible to the consumer.**

After publish:
- producer must treat the slot as immutable
- consumer is allowed to acquire it for reading

If RGA fails:
- do **not** publish
- `WriteLease` destructor auto-cancels → slot returns to `Free`

---

## 6) Requeue the camera slot (return the borrowed ring slot)

After RGA is done reading the camera buffer, the producer must `VIDIOC_QBUF` the same ring `index`.

With RAII:
- `FrameLease` destructor calls `release()` which performs `QBUF`

This prevents ring starvation even on early returns.

> Note: if you want to *surface* `QBUF` failures into a return status, you must ensure cleanup runs **before** returning from the function (e.g., by using an inner scope). Destructors can’t reliably “return a status” after the caller has already received the function’s return value.

---

## 7) Consumer side (other side of the boundary)

Consumer obtains a read capability (conceptually a `ReadLease`) from the pool:

- `acquire_read()` / `acquire_latest()` → get `ImageBuffer` view over the published DMA-BUF
- run inference / postprocess
- release read lease → slot transitions back to `Free` (or your chosen recycling policy)

**Consumer must treat published buffers as read-only.**

---

## Summary timeline (one frame)

1. `DQBUF` → `FrameLease(src)`
2. `acquire_write_lease()` → `WriteLease(dst)`
3. (optional) DMA-BUF sync start
4. RGA preprocess: `src → dst`
5. (optional) DMA-BUF sync end
6. `dst.publish()` ✅ **thread boundary crossed here**
7. `FrameLease` releases → `QBUF`

---

## Compact diagram

```
Producer thread:

  [V4L2 device] --DQBUF--> FrameLease{FrameDescriptor -> src dmabuf}
        |
        v
  acquire_write_lease()
        |
        v
  WriteLease{ImageBuffer -> dst dmabuf} --RGA--> (dst filled)
        |
        v
     publish()  ✅ boundary
        |
        v
  FrameLease dtor -> QBUF  (return ring slot)

Consumer thread:

  acquire_read() -> ReadLease{ImageBuffer -> dst dmabuf} -> infer -> release
```
