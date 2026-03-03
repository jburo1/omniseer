#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "omniseer/vision/image_buffer_pool.hpp"

using namespace std::chrono_literals;

TEST(ImageBufferPool, LatestWins)
{
  omniseer::vision::ImageBufferPool pool;

  int idx0 = -1;
  int idx1 = -1;
  int idx2 = -1;

  ASSERT_TRUE(pool.acquire_write(idx0));
  pool.buffer_at(idx0).size.w = 1;
  pool.publish_ready(idx0);

  ASSERT_TRUE(pool.acquire_write(idx1));
  pool.buffer_at(idx1).size.w = 2;
  pool.publish_ready(idx1);

  int ridx = -1;
  ASSERT_TRUE(pool.acquire_read(ridx));
  EXPECT_EQ(ridx, idx1);
  EXPECT_EQ(pool.buffer_at(ridx).size.w, 2);
  pool.publish_release(ridx);

  ASSERT_TRUE(pool.acquire_write(idx2));
  EXPECT_EQ(idx2, idx0);
  pool.buffer_at(idx2).size.w = 3;
  pool.publish_ready(idx2);

  int ridx2 = -1;
  ASSERT_TRUE(pool.acquire_read(ridx2));
  EXPECT_EQ(ridx2, idx2);
  EXPECT_EQ(pool.buffer_at(ridx2).size.w, 3);
  pool.publish_release(ridx2);
}

TEST(ImageBufferPool, ProducerConsumerLatestWins)
{
  omniseer::vision::ImageBufferPool pool;
  constexpr int iterations = 20000;

  std::atomic<bool> start{false};
  std::atomic<bool> done{false};
  std::atomic<bool> failed{false};
  std::atomic<int>  consumed{0};
  std::atomic<int>  last_seen{-1};

  std::thread producer([&]() {
    while (!start.load(std::memory_order_acquire))
    {
      std::this_thread::yield();
    }

    for (int seq = 0; seq < iterations; ++seq)
    {
      int idx = -1;
      while (!pool.acquire_write(idx))
      {
        std::this_thread::yield();
      }

      pool.buffer_at(idx).size.w = seq;
      pool.publish_ready(idx);
    }

    done.store(true, std::memory_order_release);
  });

  std::thread consumer([&]() {
    while (!start.load(std::memory_order_acquire))
    {
      std::this_thread::yield();
    }

    const auto deadline = std::chrono::steady_clock::now() + 5s;

    int seen       = -1;
    int idle_spins = 0;
    constexpr int idle_limit = 10000;

    while (true)
    {
      int idx = -1;
      if (pool.acquire_read(idx))
      {
        const int value = pool.buffer_at(idx).size.w;
        if (value < seen)
        {
          failed.store(true, std::memory_order_relaxed);
        }
        seen = value;
        pool.publish_release(idx);
        consumed.fetch_add(1, std::memory_order_relaxed);
        idle_spins = 0;
        continue;
      }

      if (std::chrono::steady_clock::now() > deadline)
      {
        failed.store(true, std::memory_order_relaxed);
        break;
      }

      if (done.load(std::memory_order_acquire))
      {
        if (++idle_spins > idle_limit)
        {
          break;
        }
      }

      std::this_thread::yield();
    }

    last_seen.store(seen, std::memory_order_relaxed);
  });

  start.store(true, std::memory_order_release);
  producer.join();
  consumer.join();

  EXPECT_FALSE(failed.load(std::memory_order_relaxed));
  EXPECT_GT(consumed.load(std::memory_order_relaxed), 0);
  EXPECT_EQ(last_seen.load(std::memory_order_relaxed), iterations - 1);
}

TEST(ImageBufferPool, ReadLeaseAutoReleasesSlot)
{
  omniseer::vision::ImageBufferPool pool;

  int ready_idx = -1;
  ASSERT_TRUE(pool.acquire_write(ready_idx));
  pool.publish_ready(ready_idx);

  auto read_lease = pool.acquire_read_lease();
  ASSERT_TRUE(read_lease.has_value());
  EXPECT_EQ(read_lease->index(), ready_idx);

  std::vector<int> drained{};
  drained.reserve(4);
  for (int i = 0; i < 4; ++i)
  {
    int idx = -1;
    ASSERT_TRUE(pool.acquire_write(idx));
    drained.push_back(idx);
  }

  int should_fail = -1;
  EXPECT_FALSE(pool.acquire_write(should_fail))
      << "leased read slot should not be available before release";

  read_lease.reset(); // destructor path -> publish_release()

  int after_release_idx = -1;
  EXPECT_TRUE(pool.acquire_write(after_release_idx))
      << "read lease release should return slot to producer free list";

  // Cleanup: explicitly cancel all temporary write acquisitions.
  for (int idx : drained)
  {
    pool.cancel_write(idx);
  }
  if (after_release_idx >= 0)
  {
    pool.cancel_write(after_release_idx);
  }
}
