#include "omniseer/vision/time_utils.hpp"

#include <ctime>
#include <linux/videodev2.h>

namespace omniseer::vision
{
namespace
{
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

bool make_v4l2_capture_timestamp_real_ns(const timeval& driver_timestamp, uint32_t driver_flags,
                                         uint64_t& out_capture_ts_real_ns) noexcept
{
  out_capture_ts_real_ns = 0;

  const uint64_t driver_ts_ns = timeval_to_ns(driver_timestamp);
  const bool driver_ts_is_mono =
      (driver_flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

  if (!driver_ts_is_mono || driver_ts_ns == 0)
    return false;

  const uint64_t mono_now = clock_now_ns(CLOCK_MONOTONIC);
  const uint64_t real_now = realtime_now_ns();
  if (mono_now == 0 || real_now == 0)
    return false;

  const uint64_t capture_ts_real_ns = monotonic_to_realtime_ns(driver_ts_ns, mono_now, real_now);
  if (capture_ts_real_ns == 0)
    return false;

  out_capture_ts_real_ns = capture_ts_real_ns;
  return true;
}
} // namespace omniseer::vision
