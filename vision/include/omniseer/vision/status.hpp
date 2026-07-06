#pragma once

#include <cstdint>

namespace omniseer::vision
{
  enum class CaptureStatus : uint8_t
  {
    Ok             = 0,
    NoFrame        = 1,
    RetryableError = 2,
    FatalError     = 3,
  };

  enum class PreprocessStatus : uint8_t
  {
    Ok                           = 0,
    InvalidConfig                = 1,
    SourceSizeMismatch           = 2,
    InvalidSourceDescriptor      = 3,
    InvalidDestinationDescriptor = 4,
    ImcheckFailed                = 5,
    ImprocessFailed              = 6,
    UnknownError                 = 7,
  };

  enum class InferStatus : uint8_t
  {
    Ok                     = 0,
    NotArmed               = 1,
    InvalidInputDescriptor = 2,
    RknnError              = 3,
  };

  enum class PostprocessStatus : uint8_t
  {
    NotRun = 0,
    Ok     = 1,
  };

  enum class ProducerTickStatus : uint8_t
  {
    Produced              = 0,
    NoFrame               = 1,
    CaptureRetryableError = 2,
    CaptureFatalError     = 3,
    NoWritableBuffer      = 4,
    PreprocessError       = 5,
  };

  enum class ConsumerTickStatus : uint8_t
  {
    Consumed      = 0,
    NoReadyBuffer = 1,
    InferError    = 2,
  };
} // namespace omniseer::vision
