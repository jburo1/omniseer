#pragma once

#include <cstdint>
#include <ctime>
#include <sys/time.h>

namespace omniseer::vision
{
  /**
   * @brief Read one POSIX clock and return nanoseconds since its epoch.
   *
   * @param clock_id POSIX clock identifier.
   * @return Clock value in nanoseconds, or 0 on failure.
   */
  uint64_t clock_now_ns(clockid_t clock_id);

  /**
   * @brief Read `CLOCK_REALTIME` in nanoseconds.
   *
   * @return Realtime nanoseconds since Unix epoch, or 0 on failure.
   */
  uint64_t realtime_now_ns();

  /**
   * @brief Convert POSIX `timeval` to nanoseconds.
   *
   * @param tv Timestamp in seconds + microseconds.
   * @return Converted timestamp in nanoseconds.
   */
  uint64_t timeval_to_ns(const timeval& tv);

  /**
   * @brief Map a V4L2 monotonic capture timestamp to realtime nanoseconds.
   *
   * @param driver_timestamp Timestamp from `v4l2_buffer::timestamp`.
   * @param driver_flags Flags from `v4l2_buffer::flags`.
   * @param out_capture_ts_real_ns Output mapped realtime timestamp.
   * @return True when mapping succeeds; false when flags/layout cannot be mapped.
   */
  bool make_v4l2_capture_timestamp_real_ns(const timeval& driver_timestamp, uint32_t driver_flags,
                                           uint64_t& out_capture_ts_real_ns) noexcept;
} // namespace omniseer::vision
