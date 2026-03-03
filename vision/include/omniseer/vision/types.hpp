#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace omniseer::vision
{
  /**
   * @brief Result category for one preprocess operation.
   */
  enum class PreprocessStatus : uint8_t
  {
    Ok,
    InvalidConfig,
    SourceSizeMismatch,
    InvalidSourceDescriptor,
    InvalidDestinationDescriptor,
    ImcheckFailed,
    ImprocessFailed,
    UnknownError,
  };

  /**
   * @brief Status payload for preprocess operations.
   */
  struct PreprocessResult
  {
    PreprocessStatus status{PreprocessStatus::Ok};

    bool ok() const noexcept
    {
      return status == PreprocessStatus::Ok;
    }
  };

  /**
   * @brief Pixel formats used throughout the vision pipeline.
   *
   * Semantics:
   * - NV12: 2 planes (plane 0 = Y, plane 1 = interleaved UV, 4:2:0).
   * - RGB888: 1 plane, 3 bytes per pixel (RGB order).
   * - BGR888: 1 plane, 3 bytes per pixel (BGR order).
   */
  enum class PixelFormat : uint32_t
  {
    NV12,
    RGB888,
    BGR888,
  };

  /**
   * @brief Width/height container used for frame and buffer dimensions.
   */
  struct Size
  {
    int w = 0;
    int h = 0;
  };

  /**
   * @brief Rectangle container used for letterboxing and cropping.
   */
  struct Rect
  {
    int x = 0;
    int y = 0;
    int w = 0;
    int h = 0;
  };

  /**
   * @brief Metadata for one image plane backed by a DMA-BUF file descriptor.
   */
  struct DMABufPlane
  {
    /// @brief DMA-BUF file descriptor.
    int fd = -1;
    /// @brief Bytes per line for this plane.
    uint32_t stride = 0;
    /// @brief Byte offset within `fd` where this plane starts.
    uint32_t offset = 0;

    /// @brief Allocated capacity for this plane in bytes.
    size_t alloc_size = 0;
    /// @brief Valid bytes for this frame (from V4L2 DQBUF).
    size_t bytesused = 0;
  };

  /**
   * @brief Descriptor for one frame borrowed from the V4L2 capture ring.
   *
   * Typical lifetime:
   * - dequeue(...) borrows slot `i` from the driver (DQBUF) and fills this descriptor.
   * - Processing stages read from `planes`.
   * - requeue(v4l2_index) returns slot `i` to the driver (QBUF).
   */
  struct FrameDescriptor
  {
    /// @brief Frame dimensions negotiated by V4L2.
    Size size{};
    /// @brief Frame pixel format.
    PixelFormat fmt = PixelFormat::NV12;

    /// @brief Number of valid entries in `planes`.
    uint32_t num_planes = 0;
    /// @brief Plane metadata view for this frame.
    std::array<DMABufPlane, 2> planes{};

    /// @brief Capture timestamp mapped to realtime nanoseconds (for ROS2 `header.stamp`).
    uint64_t capture_ts_real_ns = 0;

    /// @brief V4L2 ring slot index currently backing this frame (required for requeue).
    uint32_t v4l2_index = std::numeric_limits<uint32_t>::max();

    /// @brief Monotonic frame sequence from V4L2 (useful for drop/reorder detection).
    uint32_t sequence = 0;
  };

  /**
   * @brief Reusable application-owned buffer slot from a pool.
   *
   * Typical lifetime:
   * - acquire() obtains a free slot from the pool.
   * - RGA writes transformed pixels into this buffer.
   * - RKNN reads this buffer as model input.
   * - release() returns the slot to the pool for reuse.
   */
  struct ImageBuffer
  {
    /// @brief Buffer dimensions (for example, 640x640).
    Size size{};
    /// @brief Buffer pixel format (typically model input format).
    PixelFormat fmt = PixelFormat::RGB888;

    /// @brief Number of valid entries in `planes`.
    uint32_t num_planes = 0;
    /// @brief Plane metadata for this buffer slot.
    std::array<DMABufPlane, 2> planes{};

    /// @brief Aggregate allocated capacity across planes.
    size_t total_alloc_size = 0;
    /// @brief Per-frame V4L2 sequence number copied at handoff.
    uint32_t sequence = 0;
    /// @brief Per-frame capture timestamp mapped to realtime nanoseconds (ROS2 `header.stamp`).
    uint64_t capture_ts_real_ns = 0;
  };

} // namespace omniseer::vision
