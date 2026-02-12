#pragma once

#include <cstdint>
#include <ctime>
#include <sys/time.h>

namespace omniseer::vision
{
  uint64_t clock_now_ns(clockid_t clock_id);
  uint64_t realtime_now_ns();
  uint64_t timeval_to_ns(const timeval& tv);
  uint64_t make_v4l2_capture_timestamp_real_ns(const timeval& driver_timestamp,
                                               uint32_t driver_flags);
} // namespace omniseer::vision
