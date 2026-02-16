#pragma once

#include <cstdint>
#include <ctime>
#include <sys/time.h>

namespace omniseer::vision
{
  uint64_t clock_now_ns(clockid_t clock_id);
  uint64_t realtime_now_ns();
  uint64_t timeval_to_ns(const timeval& tv);
  // Returns false when timestamp flags/layout cannot be mapped to realtime.
  bool make_v4l2_capture_timestamp_real_ns(const timeval& driver_timestamp, uint32_t driver_flags,
                                           uint64_t& out_capture_ts_real_ns) noexcept;
} // namespace omniseer::vision
