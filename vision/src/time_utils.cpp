#include "omniseer/vision/time_utils.hpp"

#include <ctime>
#include <linux/videodev2.h>
#include <stdexcept>

namespace omniseer::vision
{
namespace
{
  uint64_t monotonic_now_ns()
  {
    return clock_now_ns(CLOCK_MONOTONIC);
  }

  uint64_t monotonic_to_realtime_ns(uint64_t ts_mono_ns, uint64_t snap_mono_ns,
                                    uint64_t snap_real_ns)
  {
    if (ts_mono_ns == 0 || snap_mono_ns == 0 || snap_real_ns == 0)
      return 0;

    if (snap_mono_ns >= ts_mono_ns)
      return snap_real_ns - (snap_mono_ns - ts_mono_ns);

    return snap_real_ns + (ts_mono_ns - snap_mono_ns);
  }
} // namespace

uint64_t clock_now_ns(clockid_t clock_id)
{
  timespec ts{};
  if (::clock_gettime(clock_id, &ts) != 0)
    return 0;

  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

uint64_t realtime_now_ns()
{
  return clock_now_ns(CLOCK_REALTIME);
}

uint64_t timeval_to_ns(const timeval& tv)
{
  return static_cast<uint64_t>(tv.tv_sec) * 1000000000ULL +
         static_cast<uint64_t>(tv.tv_usec) * 1000ULL;
}

uint64_t make_v4l2_capture_timestamp_real_ns(const timeval& driver_timestamp,
                                             uint32_t driver_flags)
{
  const uint64_t driver_ts_ns = timeval_to_ns(driver_timestamp);
  const bool driver_ts_is_mono =
      (driver_flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

  if (!driver_ts_is_mono || driver_ts_ns == 0)
    throw std::runtime_error("time_utils: expected monotonic non-zero V4L2 timestamp");

  const uint64_t mono_before = monotonic_now_ns();
  const uint64_t real_now    = realtime_now_ns();
  const uint64_t mono_after  = monotonic_now_ns();
  const uint64_t snap_mono_ns =
      (mono_after >= mono_before) ? (mono_before + ((mono_after - mono_before) / 2ULL))
                                  : mono_after;

  const uint64_t capture_ts_real_ns =
      monotonic_to_realtime_ns(driver_ts_ns, snap_mono_ns, real_now);
  if (capture_ts_real_ns == 0)
    throw std::runtime_error("time_utils: failed to map capture timestamp to realtime");

  return capture_ts_real_ns;
}
} // namespace omniseer::vision
