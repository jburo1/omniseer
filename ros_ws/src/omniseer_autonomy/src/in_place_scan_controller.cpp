#include "omniseer_autonomy/in_place_scan_controller.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace omniseer_autonomy
{
namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
}

InPlaceScanController::InPlaceScanController(InPlaceScanConfig config)
: _config(std::move(config))
{
  if (!std::isfinite(_config.yaw_rate_rad_s) || _config.yaw_rate_rad_s <= 0.0) {
    throw std::invalid_argument("yaw_rate_rad_s must be finite and positive");
  }
  if (!std::isfinite(_config.revolutions) || _config.revolutions <= 0.0) {
    throw std::invalid_argument("revolutions must be finite and positive");
  }
}

InPlaceScanOutput InPlaceScanController::update_heading(double heading_rad)
{
  if (_complete) {
    return command();
  }

  if (_last_heading_rad.has_value()) {
    const auto delta = std::atan2(std::sin(heading_rad - *_last_heading_rad),
                                    std::cos(heading_rad - *_last_heading_rad));
    _yaw_travel_rad += std::abs(delta);
    if (_yaw_travel_rad >= (_config.revolutions * kTwoPi)) {
      _complete = true;
    }
  }
  _last_heading_rad = heading_rad;
  return command();
}

InPlaceScanOutput InPlaceScanController::command() const noexcept
{
  return InPlaceScanOutput{0.0,
    _complete || !_last_heading_rad.has_value() ? 0.0 : _config.yaw_rate_rad_s};
}

bool InPlaceScanController::complete() const noexcept
{
  return _complete;
}

double InPlaceScanController::yaw_travel_rad() const noexcept
{
  return _yaw_travel_rad;
}
} // namespace omniseer_autonomy
