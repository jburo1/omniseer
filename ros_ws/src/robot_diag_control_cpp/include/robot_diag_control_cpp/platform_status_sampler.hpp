#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

#include "robot_diag_control_cpp/gateway_state.hpp"

namespace robot_diag_control_cpp
{
class PlatformStatusSampler
{
public:
  explicit PlatformStatusSampler(
    std::filesystem::path sys_root = "/sys",
    std::filesystem::path proc_root = "/proc",
    std::filesystem::path disk_path = "/");

  ComputeStatusSnapshot sample_compute();
  NetworkStatusSnapshot sample_network() const;
  BatteryStatusSnapshot sample_onboard_battery() const;

private:
  struct CpuTotals
  {
    uint64_t idle{0};
    uint64_t total{0};
  };

  std::optional<CpuTotals> read_cpu_totals() const;
  std::optional<double> read_cpu_temperature_c() const;
  std::optional<bool> read_thermal_throttled() const;
  void read_memory(uint64_t & used_bytes, uint64_t & total_bytes, double & used_percent) const;
  std::optional<double> read_disk_used_percent() const;
  std::optional<std::string> select_wireless_interface() const;
  std::optional<int32_t> read_wifi_signal_dbm(const std::string & interface_name) const;
  std::optional<uint32_t> read_wifi_link_quality_percent(const std::string & interface_name) const;
  std::optional<std::filesystem::path> select_onboard_battery_path() const;

  std::filesystem::path _sys_root{};
  std::filesystem::path _proc_root{};
  std::filesystem::path _disk_path{};
  std::optional<CpuTotals> _last_cpu_totals{};
};
} // namespace robot_diag_control_cpp
