#include "robot_diag_control_cpp/platform_status_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <exception>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

namespace robot_diag_control_cpp
{
namespace
{
std::optional<std::string> read_first_line(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  if (!stream) {
    return std::nullopt;
  }

  std::string line;
  std::getline(stream, line);
  return line;
}

std::optional<int64_t> read_int64(const std::filesystem::path & path)
{
  const auto line = read_first_line(path);
  if (!line.has_value()) {
    return std::nullopt;
  }

  try {
    return std::stoll(*line);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<double> read_double(const std::filesystem::path & path)
{
  const auto line = read_first_line(path);
  if (!line.has_value()) {
    return std::nullopt;
  }

  try {
    return std::stod(*line);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::string lower_copy(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char character)
    {
      return static_cast<char>(std::tolower(character));
    });
  return value;
}
} // namespace

PlatformStatusSampler::PlatformStatusSampler(
  std::filesystem::path sys_root, std::filesystem::path proc_root, std::filesystem::path disk_path)
: _sys_root(std::move(sys_root)),
  _proc_root(std::move(proc_root)),
  _disk_path(std::move(disk_path))
{
}

ComputeStatusSnapshot PlatformStatusSampler::sample_compute()
{
  ComputeStatusSnapshot snapshot;
  snapshot.available = true;

  const auto current_cpu = read_cpu_totals();
  if (current_cpu.has_value() && _last_cpu_totals.has_value() &&
    current_cpu->total > _last_cpu_totals->total)
  {
    const auto total_delta = current_cpu->total - _last_cpu_totals->total;
    const auto idle_delta = current_cpu->idle - _last_cpu_totals->idle;
    snapshot.cpu_percent = 100.0 * static_cast<double>(total_delta - idle_delta) /
      static_cast<double>(total_delta);
  }
  if (current_cpu.has_value()) {
    _last_cpu_totals = current_cpu;
  }

  const auto cpu_temperature = read_cpu_temperature_c();
  if (cpu_temperature.has_value()) {
    snapshot.cpu_temperature_available = true;
    snapshot.cpu_temperature_c = *cpu_temperature;
  }

  const auto throttled = read_thermal_throttled();
  if (throttled.has_value()) {
    snapshot.thermal_throttled_available = true;
    snapshot.thermal_throttled = *throttled;
  }

  read_memory(snapshot.ram_used_bytes, snapshot.ram_total_bytes, snapshot.ram_used_percent);

  const auto disk_used_percent = read_disk_used_percent();
  if (disk_used_percent.has_value()) {
    snapshot.disk_available = true;
    snapshot.disk_used_percent = *disk_used_percent;
  }

  return snapshot;
}

NetworkStatusSnapshot PlatformStatusSampler::sample_network() const
{
  NetworkStatusSnapshot snapshot;
  const auto interface_name = select_wireless_interface();
  if (!interface_name.has_value()) {
    return snapshot;
  }

  snapshot.available = true;
  snapshot.interface_name = *interface_name;
  snapshot.connected = true;

  const auto signal_dbm = read_wifi_signal_dbm(*interface_name);
  if (signal_dbm.has_value()) {
    snapshot.wifi_signal_available = true;
    snapshot.wifi_signal_dbm = *signal_dbm;
  }

  const auto link_quality = read_wifi_link_quality_percent(*interface_name);
  if (link_quality.has_value()) {
    snapshot.link_quality_available = true;
    snapshot.link_quality_percent = *link_quality;
  }

  return snapshot;
}

BatteryStatusSnapshot PlatformStatusSampler::sample_onboard_battery() const
{
  BatteryStatusSnapshot snapshot;
  const auto battery_path = select_onboard_battery_path();
  if (!battery_path.has_value()) {
    return snapshot;
  }

  snapshot.available = true;
  snapshot.source = battery_path->filename().string();
  snapshot.present = true;

  const auto present = read_int64(*battery_path / "present");
  if (present.has_value()) {
    snapshot.present = *present != 0;
  }

  const auto voltage_now = read_double(*battery_path / "voltage_now");
  if (voltage_now.has_value()) {
    snapshot.voltage_available = true;
    snapshot.voltage = *voltage_now / 1000000.0;
  }

  const auto capacity = read_double(*battery_path / "capacity");
  if (capacity.has_value()) {
    snapshot.percentage_available = true;
    snapshot.percentage = *capacity;
  }

  const auto status = read_first_line(*battery_path / "status");
  if (status.has_value()) {
    const auto lowered = lower_copy(*status);
    snapshot.charging_available = true;
    snapshot.charging = lowered == "charging";
  }

  return snapshot;
}

std::optional<PlatformStatusSampler::CpuTotals> PlatformStatusSampler::read_cpu_totals() const
{
  const auto line = read_first_line(_proc_root / "stat");
  if (!line.has_value()) {
    return std::nullopt;
  }

  std::istringstream stream(*line);
  std::string label;
  uint64_t user = 0;
  uint64_t nice = 0;
  uint64_t system = 0;
  uint64_t idle = 0;
  uint64_t iowait = 0;
  uint64_t irq = 0;
  uint64_t softirq = 0;
  uint64_t steal = 0;
  stream >> label >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;
  if (!stream || label != "cpu") {
    return std::nullopt;
  }

  return CpuTotals{
    idle + iowait,
    user + nice + system + idle + iowait + irq + softirq + steal,
  };
}

std::optional<double> PlatformStatusSampler::read_cpu_temperature_c() const
{
  const auto thermal_root = _sys_root / "class" / "thermal";
  if (!std::filesystem::exists(thermal_root)) {
    return std::nullopt;
  }

  for (const auto & entry : std::filesystem::directory_iterator(thermal_root)) {
    const auto temp_milli_c = read_double(entry.path() / "temp");
    if (temp_milli_c.has_value()) {
      return *temp_milli_c / 1000.0;
    }
  }
  return std::nullopt;
}

std::optional<bool> PlatformStatusSampler::read_thermal_throttled() const
{
  const auto value = read_first_line(_sys_root / "devices" / "system" / "cpu" / "cpu0" /
    "cpufreq" / "throttle_stats" / "throttled_time");
  if (!value.has_value()) {
    return std::nullopt;
  }

  try {
    return std::stoull(*value) > 0;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

void PlatformStatusSampler::read_memory(
  uint64_t & used_bytes, uint64_t & total_bytes, double & used_percent) const
{
  std::ifstream stream(_proc_root / "meminfo");
  if (!stream) {
    return;
  }

  uint64_t mem_total_kb = 0;
  uint64_t mem_available_kb = 0;
  std::string key;
  uint64_t value = 0;
  std::string unit;
  while (stream >> key >> value >> unit) {
    if (key == "MemTotal:") {
      mem_total_kb = value;
    } else if (key == "MemAvailable:") {
      mem_available_kb = value;
    }
  }

  if (mem_total_kb == 0 || mem_available_kb > mem_total_kb) {
    return;
  }

  total_bytes = mem_total_kb * 1024U;
  used_bytes = (mem_total_kb - mem_available_kb) * 1024U;
  used_percent = 100.0 * static_cast<double>(used_bytes) / static_cast<double>(total_bytes);
}

std::optional<double> PlatformStatusSampler::read_disk_used_percent() const
{
  std::error_code error;
  const auto space = std::filesystem::space(_disk_path, error);
  if (error || space.capacity == 0 || space.free > space.capacity) {
    return std::nullopt;
  }

  return 100.0 * static_cast<double>(space.capacity - space.free) /
         static_cast<double>(space.capacity);
}

std::optional<std::string> PlatformStatusSampler::select_wireless_interface() const
{
  const auto wireless_root = _sys_root / "class" / "net";
  if (!std::filesystem::exists(wireless_root)) {
    return std::nullopt;
  }

  for (const auto & entry : std::filesystem::directory_iterator(wireless_root)) {
    if (!std::filesystem::exists(entry.path() / "wireless")) {
      continue;
    }
    const auto operstate = read_first_line(entry.path() / "operstate").value_or("");
    if (operstate == "up" || operstate == "unknown") {
      return entry.path().filename().string();
    }
  }
  return std::nullopt;
}

std::optional<int32_t> PlatformStatusSampler::read_wifi_signal_dbm(
  const std::string & interface_name) const
{
  std::ifstream stream(_proc_root / "net" / "wireless");
  if (!stream) {
    return std::nullopt;
  }

  std::string line;
  while (std::getline(stream, line)) {
    if (line.find(interface_name + ":") == std::string::npos) {
      continue;
    }

    std::istringstream fields(line.substr(line.find(':') + 1));
    std::string status;
    double link = 0.0;
    double level = 0.0;
    fields >> status >> link >> level;
    if (!fields) {
      return std::nullopt;
    }
    return static_cast<int32_t>(std::lround(level));
  }
  return std::nullopt;
}

std::optional<uint32_t> PlatformStatusSampler::read_wifi_link_quality_percent(
  const std::string & interface_name) const
{
  std::ifstream stream(_proc_root / "net" / "wireless");
  if (!stream) {
    return std::nullopt;
  }

  std::string line;
  while (std::getline(stream, line)) {
    if (line.find(interface_name + ":") == std::string::npos) {
      continue;
    }

    std::istringstream fields(line.substr(line.find(':') + 1));
    std::string status;
    double link = 0.0;
    fields >> status >> link;
    if (!fields) {
      return std::nullopt;
    }
    return static_cast<uint32_t>(std::clamp(std::lround(link * 100.0 / 70.0), 0L, 100L));
  }
  return std::nullopt;
}

std::optional<std::filesystem::path> PlatformStatusSampler::select_onboard_battery_path() const
{
  const auto power_supply_root = _sys_root / "class" / "power_supply";
  if (!std::filesystem::exists(power_supply_root)) {
    return std::nullopt;
  }

  for (const auto & entry : std::filesystem::directory_iterator(power_supply_root)) {
    const auto type = lower_copy(read_first_line(entry.path() / "type").value_or(""));
    if (type == "battery") {
      return entry.path();
    }
  }
  return std::nullopt;
}
} // namespace robot_diag_control_cpp
