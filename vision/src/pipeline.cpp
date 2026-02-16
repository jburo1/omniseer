#include "omniseer/vision/pipeline.hpp"

#include <chrono>
#include <cerrno>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace omniseer::vision
{
  namespace
  {
    const char* capture_status_name(CaptureStatus status) noexcept
    {
      switch (status)
      {
      case CaptureStatus::Ok:
        return "ok";
      case CaptureStatus::NoFrame:
        return "no-frame";
      case CaptureStatus::RetryableError:
        return "retryable-error";
      case CaptureStatus::FatalError:
        return "fatal-error";
      }
      return "unknown";
    }

    const char* preprocess_status_name(PreprocessStatus status) noexcept
    {
      switch (status)
      {
      case PreprocessStatus::Ok:
        return "ok";
      case PreprocessStatus::InvalidConfig:
        return "invalid-config";
      case PreprocessStatus::SourceSizeMismatch:
        return "source-size-mismatch";
      case PreprocessStatus::InvalidSourceDescriptor:
        return "invalid-source-descriptor";
      case PreprocessStatus::InvalidDestinationDescriptor:
        return "invalid-destination-descriptor";
      case PreprocessStatus::ImcheckFailed:
        return "imcheck-failed";
      case PreprocessStatus::ImprocessFailed:
        return "improcess-failed";
      }
      return "unknown";
    }

    [[noreturn]] void throw_capture_preflight_failure(const char* context,
                                                      const CaptureResult& capture)
    {
      throw std::runtime_error(
          std::string("ProducerPipeline::preflight: ") + context + " (status=" +
          capture_status_name(capture.status) + ", errno=" + std::to_string(capture.sys_errno) + ")");
    }

    [[noreturn]] void throw_preprocess_preflight_failure(const PreprocessResult& preprocess,
                                                         int                     pool_index)
    {
      throw std::runtime_error(std::string("ProducerPipeline::preflight: preprocess failed (status=") +
                               preprocess_status_name(preprocess.status) +
                               ", pool_index=" + std::to_string(pool_index) + ")");
    }

    ProducerTickResult make_not_armed_result() noexcept
    {
      ProducerTickResult out{};
      out.status  = ProducerTickStatus::CaptureFatalError;
      out.capture = {CaptureStatus::FatalError, EPERM};
      return out;
    }

    ProducerTickResult make_dequeue_failure_result(
        const V4l2Capture::DequeueLeaseResult& dq) noexcept
    {
      ProducerTickResult out{};
      out.capture = dq.capture;

      if (dq.capture.status == CaptureStatus::NoFrame)
      {
        out.status = ProducerTickStatus::NoFrame;
        return out;
      }

      if (dq.capture.status == CaptureStatus::RetryableError)
      {
        out.status = ProducerTickStatus::CaptureRetryableError;
        return out;
      }

      if (out.capture.status == CaptureStatus::Ok)
      {
        out.capture = {CaptureStatus::FatalError, EPROTO};
      }
      out.status = ProducerTickStatus::CaptureFatalError;
      return out;
    }

    ProducerTickResult finalize_with_frame_release(V4l2Capture::DequeueLeaseResult& dq,
                                                   ProducerTickResult& out,
                                                   ProducerTickStatus status) noexcept
    {
      out.status = status;
      const CaptureResult release_result = dq.lease->release();
      if (!release_result.ok())
      {
        out.capture = release_result;
        if (release_result.status == CaptureStatus::RetryableError ||
            release_result.status == CaptureStatus::NoFrame)
        {
          out.status = ProducerTickStatus::CaptureRetryableError;
        }
        else
        {
          out.status = ProducerTickStatus::CaptureFatalError;
        }
      }
      return out;
    }

    bool should_retry_preflight_capture(const CaptureResult& capture) noexcept
    {
      return capture.status == CaptureStatus::NoFrame ||
             capture.status == CaptureStatus::RetryableError;
    }

    void reclaim_ready_buffers(ImageBufferPool& pool) noexcept
    {
      int idx = -1;
      while (pool.acquire_read(idx))
      {
        pool.publish_release(idx);
      }
    }

    std::vector<ImageBufferPool::WriteLease> acquire_all_writable_leases(ImageBufferPool& pool)
    {
      std::vector<ImageBufferPool::WriteLease> leases{};
      for (;;)
      {
        auto lease = pool.acquire_write_lease();
        if (!lease)
          break;
        leases.emplace_back(std::move(*lease));
      }
      return leases;
    }

    void release_preflight_frame_or_throw(V4l2Capture::DequeueLeaseResult& dq)
    {
      const CaptureResult release_result = dq.lease->release();
      if (!release_result.ok())
      {
        throw_capture_preflight_failure("capture requeue failed", release_result);
      }
    }
  } // namespace

  ProducerPipeline::ProducerPipeline(V4l2Capture& capture, RgaPreprocess& preprocess,
                                     ImageBufferPool& pool)
      : _capture(capture), _preprocess(preprocess), _pool(pool)
  {
  }

  void ProducerPipeline::preflight()
  {
    if (_armed)
      return;

    reclaim_ready_buffers(_pool);

    constexpr int max_attempts = 2000;
    for (int attempt = 0; attempt < max_attempts; ++attempt)
    {
      auto dq = _capture.dequeue_lease();
      if (should_retry_preflight_capture(dq.capture))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      if (!dq.ok())
      {
        throw_capture_preflight_failure("capture dequeue failed", dq.capture);
      }

      auto writable_leases = acquire_all_writable_leases(_pool);
      if (writable_leases.empty())
      {
        release_preflight_frame_or_throw(dq);
        throw std::runtime_error("ProducerPipeline::preflight: no writable buffer in pool");
      }

      for (auto& dst_lease : writable_leases)
      {
        PreprocessResult preprocess_result = _preprocess.preflight(dq.lease->frame(), dst_lease.buffer());
        if (!preprocess_result.ok())
        {
          release_preflight_frame_or_throw(dq);
          throw_preprocess_preflight_failure(preprocess_result, dst_lease.index());
        }
      }

      release_preflight_frame_or_throw(dq);
      _armed = true;
      return;
    }

    throw std::runtime_error("ProducerPipeline::preflight: timeout waiting for capture frame");
  }

  // Non-blocking producer tick.
  ProducerTickResult ProducerPipeline::run() noexcept
  {
    if (!_armed)
    {
      return make_not_armed_result();
    }

    auto dq = _capture.dequeue_lease();
    if (!dq.ok())
    {
      return make_dequeue_failure_result(dq);
    }

    ProducerTickResult out{};
    out.capture = dq.capture;

    auto dst_lease = _pool.acquire_write_lease();
    if (!dst_lease)
    {
      return finalize_with_frame_release(dq, out, ProducerTickStatus::NoWritableBuffer);
    }

    const FrameDescriptor& src = dq.lease->frame();
    ImageBuffer&           dst = dst_lease->buffer();
    out.preprocess             = _preprocess.run(src, dst);
    if (!out.preprocess.ok())
    {
      return finalize_with_frame_release(dq, out, ProducerTickStatus::PreprocessError);
    }

    dst.sequence           = src.sequence;
    dst.capture_ts_real_ns = src.capture_ts_real_ns;
    dst_lease->publish();

    return finalize_with_frame_release(dq, out, ProducerTickStatus::Produced);
  }
} // namespace omniseer::vision
