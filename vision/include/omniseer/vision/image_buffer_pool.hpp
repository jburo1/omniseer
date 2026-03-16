#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <queue>

#include "omniseer/vision/dma_heap_alloc.hpp"
#include "omniseer/vision/spsc_ring.hpp"
#include "omniseer/vision/types.hpp"

namespace omniseer::vision
{
  /**
   * @brief Lock-free SPSC image buffer pool between RGA and RKNN.
   *
   * Flow:
   * - Producer (manual): acquire_write() -> RGA writes -> publish_ready().
   * - Consumer (manual): acquire_read() -> RKNN reads -> publish_release().
   * - Producer (RAII preferred): acquire_write_lease() -> write -> WriteLease::publish().
   *   If not published, WriteLease destructor cancels and returns slot to free list.
   * - Consumer (RAII preferred): acquire_read_lease() -> read -> optional ReadLease::release().
   *   If not explicitly released, ReadLease destructor releases slot to free list.
   *
   * Policy:
   * - Latest-wins (single ready slot).
   * - If producer publishes a newer ready buffer while an older ready buffer is pending,
   *   the older one is dropped back to free.
   */
  class ImageBufferPool
  {
  public:
    /**
     * @brief RAII lease for a producer-owned writable pool slot.
     *
     * A valid lease may be published exactly once
     */
    class WriteLease
    {
    public:
      WriteLease() = default;
      ~WriteLease() noexcept;

      WriteLease(const WriteLease&)            = delete;
      WriteLease& operator=(const WriteLease&) = delete;

      WriteLease(WriteLease&& other) noexcept;
      WriteLease& operator=(WriteLease&& other) noexcept;

      /// @brief Mutable access to the leased image buffer.
      ImageBuffer& buffer() noexcept;
      /// @brief Pool slot index for the leased buffer.
      int          index() const noexcept;

      /// @brief Publish this written slot as the latest ready buffer.
      void publish() noexcept;

      /// @brief True when this lease currently owns a writable slot.
      explicit operator bool() const noexcept;

    private:
      friend class ImageBufferPool;
      /// @brief Construct a valid lease for one pool slot.
      WriteLease(ImageBufferPool* pool, int idx) noexcept;
      /// @brief Invalidate without publishing (used after move/cancel).
      void _reset() noexcept;

      ImageBufferPool* _pool{nullptr};
      int              _idx{-1};
    };

    /**
     * @brief RAII lease for a consumer-owned readable pool slot.
     *
     * A valid lease may be explicitly released once, otherwise destructor releases it.
     */
    class ReadLease
    {
    public:
      ReadLease() = default;
      ~ReadLease() noexcept;

      ReadLease(const ReadLease&)            = delete;
      ReadLease& operator=(const ReadLease&) = delete;

      ReadLease(ReadLease&& other) noexcept;
      ReadLease& operator=(ReadLease&& other) noexcept;

      /// @brief Read-only access to the leased image buffer.
      const ImageBuffer& buffer() const noexcept;
      /// @brief Pool slot index for the leased buffer.
      int index() const noexcept;

      /// @brief Release this consumed slot back to the free list.
      void release() noexcept;

      /// @brief True when this lease currently owns a readable slot.
      explicit operator bool() const noexcept;

    private:
      friend class ImageBufferPool;
      /// @brief Construct a valid lease for one pool slot.
      ReadLease(ImageBufferPool* pool, int idx) noexcept;
      /// @brief Invalidate and release when owned.
      void _reset() noexcept;

      ImageBufferPool* _pool{nullptr};
      int              _idx{-1};
    };

  private:
    /// @brief Fixed pool size.
    static const int size{5};

    /// @brief Backing allocations for each pool slot (owns DMA-BUF fds).
    std::array<DmabufAllocation, size> allocations{};

    /// @brief Preallocated image buffer metadata for each slot.
    std::array<ImageBuffer, size> pool;

    /// @brief SPSC free-index ring shared between producer and consumer.
    SpscRing free_ring{size};

    /// @brief Producer-only stash for indices dropped by latest-wins overwrite.
    std::array<int, size> producer_free{};
    int                   producer_free_n{0};

    /// @brief Index of the freshest ready image buffer, or -1 when none.
    std::atomic<int> ready_idx{-1};

  public:
    /// @brief Construct an empty pool; call allocate_all() before use.
    ImageBufferPool();

    /**
     * @brief Allocate DMA-BUF memory for all pool slots.
     *
     * @throws On invalid arguments or allocation failures.
     * @pre Must be called before using buffers with RGA/RKNN.
     */
    void allocate_all(DmaHeapAllocator& allocator, int width, int height, PixelFormat fmt);

    /**
     * @brief Producer path: acquire a free writable slot index.
     *
     * @param idx Output pool slot index.
     * @return True on success; false when no free slots are available.
     */
    bool acquire_write(int& idx);
    /// @brief Producer path convenience: acquire a writable RAII lease.
    std::optional<WriteLease> acquire_write_lease() noexcept;

    /// @brief Producer path: publish a written slot as ready.
    void publish_ready(int idx);
    /// @brief Producer path: cancel a write and return slot to free.
    void cancel_write(int idx) noexcept;

    /**
     * @brief Consumer path: acquire the latest ready slot index for reading.
     *
     * @param idx Output pool slot index.
     * @return True on success; false when no slot is ready.
     */
    bool acquire_read(int& idx);
    /// @brief Consumer path convenience: acquire a readable RAII lease.
    std::optional<ReadLease> acquire_read_lease() noexcept;

    /// @brief Consumer path: release a consumed slot back to the free list.
    void publish_release(int idx);

    /// @brief Access preallocated buffer metadata by pool index.
    ImageBuffer&       buffer_at(int idx);
    /// @brief Read-only access to preallocated buffer metadata by pool index.
    const ImageBuffer& buffer_at(int idx) const;

    /// @brief Fixed number of pool slots.
    static constexpr int capacity() noexcept
    {
      return size;
    }
  };

} // namespace omniseer::vision
