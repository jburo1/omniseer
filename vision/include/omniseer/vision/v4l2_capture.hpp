#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "omniseer/vision/types.hpp"

namespace omniseer::vision
{

  /**
   * @brief Result category for V4L2 capture operations.
   *
   * Semantics:
   * - Ok: operation succeeded.
   * - NoFrame: dequeue found no frame available.
   * - RetryableError: transient failure (retry possible).
   * - FatalError: unrecoverable failure (restart required).
   */
  enum class CaptureStatus : uint8_t
  {
    Ok,
    NoFrame,
    RetryableError,
    FatalError,
  };

  /**
   * @brief Status + errno payload for capture operations.
   *
   * `sys_errno == 0` means “no OS errno captured”.
   */
  struct CaptureResult
  {
    CaptureStatus status{CaptureStatus::Ok};
    int           sys_errno{0};

    bool ok() const noexcept
    {
      return status == CaptureStatus::Ok;
    }
  };

  /**
   * @brief V4L2 camera capture with driver-managed buffer ring and DMA-BUF exports.
   *
   * Contract:
   * - Lifecycle:
   *   1) Construct with Config.
   *   2) start(): open/configure device, allocate ring (REQBUFS), export DMA-BUF fds (EXPBUF),
   *      queue all slots (QBUF), then STREAMON.
   *   3) Hot path:
   *      - dequeue(...) / dequeue_lease(): borrows one filled ring slot from the driver (DQBUF).
   *      - requeue(index) / FrameLease::release(): returns that slot to the driver (QBUF).
   *   4) stop(): STREAMOFF and cleanup.
   *
   * Ownership & lifetime:
   * - The driver owns the ring slots; the app only borrows a slot between DQBUF and QBUF using tthe
   * lease.
   * - FrameDescriptor is a view for the next stage.
   *
   * Error policy:
   * - start() may throw on fatal configuration/IO failures (setup is not the hot path).
   * - Hot-path APIs are noexcept and return CaptureResult.
   *
   * Performance:
   * - Hot-path methods are non-allocating/non-throw and intended for high-FPS steady-state
   * operation.
   */
  class V4l2Capture
  {
  public:
    /**
     * @brief RAII token representing a borrowed camera ring slot (DQBUF .. QBUF).
     *
     * Invariants:
     * - Exactly one live FrameLease owns the obligation to requeue that slot.
     *
     * Ownership:
     * - Does not own the camera device fd or the DMA-BUF fd.
     * - Owns the *responsibility* to return the slot to the driver.
     *
     * Destruction:
     * - ~FrameLease() is noexcept; if the lease is still valid, it will attempt to requeue.
     *   (Errors cannot be surfaced from a destructor; prefer calling release() explicitly
     *    if you want to observe failures / emit telemetry.)
     */
    class FrameLease
    {
    public:
      FrameLease() = default;
      ~FrameLease() noexcept;

      FrameLease(const FrameLease&)            = delete;
      FrameLease& operator=(const FrameLease&) = delete;

      FrameLease(FrameLease&& other) noexcept;
      FrameLease& operator=(FrameLease&& other) noexcept;

      /// @brief Read-only Access the borrowed frame metadata.
      const FrameDescriptor& frame() const noexcept;

      /// @brief Mutable access to borrowed frame metadata.
      FrameDescriptor& frame() noexcept;

      /**
       * @brief Requeue the underlying ring slot back to the driver and invalidate this lease.
       *
       * @return CaptureResult::Ok on successful requeue.
       */
      CaptureResult release() noexcept;

      /// @brief True if this lease currently represents a borrowed slot.
      explicit operator bool() const noexcept;

    private:
      friend class V4l2Capture;

      /// @brief Construct a valid lease bound to a capture instance and one specific slot.
      FrameLease(V4l2Capture* capture, FrameDescriptor frame) noexcept;

      /// @brief Invalidate without requeue (used internally after explicit release/move).
      void _reset() noexcept;

      V4l2Capture*    _capture{nullptr};
      FrameDescriptor _frame{};
    };

    /**
     * @brief Capture configuration.
     *
     * Fields:
     * - device: device tree path like "/dev/video0"
     * - width/height: negotiated capture resolution
     * - fourcc: negotiated pixel format (e.g., NV12)
     * - buffer_count: number of driver ring slots to allocate (REQBUFS)
     */
    struct Config
    {
      std::string device;
      uint32_t    width, height;
      uint32_t    fourcc;
      uint32_t    buffer_count;
    };

    /**
     * @brief Result of dequeue_lease(): status + optional RAII lease.
     *
     * ok() means:
     * - capture.ok() AND a lease is present
     */
    struct DequeueLeaseResult
    {
      CaptureResult              capture{};
      std::optional<FrameLease> lease{};

      bool ok() const noexcept
      {
        return capture.ok() && lease.has_value();
      }
    };

    /// @brief Construct with configuration only.
    explicit V4l2Capture(Config config);

    /// @brief Ensures streaming is stopped and resources are released.
    ~V4l2Capture();

    // Non-copyable/movable
    V4l2Capture(const V4l2Capture&)            = delete;
    V4l2Capture& operator=(const V4l2Capture&) = delete;
    V4l2Capture(V4l2Capture&&)                 = delete;
    V4l2Capture& operator=(V4l2Capture&&)      = delete;

    /**
     * @brief Start streaming: open/configure device, allocate ring, export DMA-BUF fds, queue all,
     * STREAMON.
     *
     * @throws (by policy) on fatal setup errors (bad device, invalid format, ioctl failures, etc.)
     * @post On success, streaming is active and hot-path dequeue/requeue calls are valid.
     */
    void start();

    /**
     * @brief Stop streaming (STREAMOFF) and release driver resources.
     */
    void stop() noexcept;

    /**
     * @brief Dequeue one filled slot and describe it in `out`.
     *
     * Behavior:
     * - On Ok: `out` is populated with DMA-BUF fd + layout metadata + a slot index for later
     * requeue.
     * - On NoFrame: no frame available right now.
     *
     * @param out Output metadata view describing the borrowed buffer.
     * @return CaptureResult indicating outcome.
     *
     * @pre Streaming is active.
     * @post On Ok: caller must eventually call requeue(out.v4l2_index) to return the slot.
     * @note Non-allocating, noexcept, hotpath.
     */
    CaptureResult dequeue(FrameDescriptor& out) noexcept;

    /**
     * @brief Hot-path: return a previously dequeued slot to the driver (QBUF).
     *
     * @param index Slot index obtained from a successful dequeue.
     * @return CaptureResult::Ok on success.
     *
     * @pre index refers to a currently-borrowed slot from this capture instance.
     * @note Non-allocating, noexcept, hotpath.
     */
    CaptureResult requeue(uint32_t index) noexcept;

    /**
     * @brief Hot-path convenience: dequeue and return an RAII lease.
     *
     * Preferred usage:
     * - Use FrameLease when you want “exactly-once requeue” enforced structurally.
     * - Use dequeue/requeue when you need maximal explicit control/telemetry.
     *
     * @return DequeueLeaseResult with status + optional lease.
     * @note Non-allocating, noexcept.
     */
    DequeueLeaseResult dequeue_lease() noexcept;

  private:
    struct Slot
    {
      int      dmabuf_fd{-1}; // VIDIOC_EXPBUF; expected stable for capture lifetime
      uint32_t alloc_size{0}; // VIDIOC_QUERYBUF (plane 0)
    };

    std::string       _device{};
    uint32_t          _buffer_count{0};
    int               _fd{-1}; // camera device fd
    uint32_t          _width{0}, _height{0};
    uint32_t          _fourcc{0};
    uint32_t          _bytesperline{0}; // stride from G_FMT readback
    std::vector<Slot> _slots{};         // size == _buffer_count
    bool              _streaming{false};
  };

} // namespace omniseer::vision
