#pragma once

#include <optional>

namespace omniseer_autonomy
{
  struct InPlaceScanConfig
  {
    double yaw_rate_rad_s{0.20};
    double revolutions{1.0};
  };

  struct InPlaceScanOutput
  {
    double linear_x_m_s{0.0};
    double angular_z_rad_s{0.0};
  };

  class InPlaceScanController
  {
  public:
    explicit InPlaceScanController(InPlaceScanConfig config);

    InPlaceScanOutput update_heading(double heading_rad);
    InPlaceScanOutput command() const noexcept;

    bool   complete() const noexcept;
    double yaw_travel_rad() const noexcept;

  private:
    InPlaceScanConfig     _config{};
    std::optional<double> _last_heading_rad{};
    double                _yaw_travel_rad{0.0};
    bool                  _complete{false};
  };
} // namespace omniseer_autonomy
