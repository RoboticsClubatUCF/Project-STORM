// system_monitor_node.cpp: onboard computer + WiFi stats on /telemetry/system

#include <sys/statvfs.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "storm_interfaces/msg/system_telemetry.hpp"

namespace
{

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

struct CpuTimes
{
  uint64_t idle{0};
  uint64_t total{0};
};

std::optional<CpuTimes> read_cpu_times()
{
  std::ifstream f("/proc/stat");
  std::string label;
  if (!(f >> label) || label != "cpu") return std::nullopt;

  // user nice system idle iowait irq softirq steal
  uint64_t v[8] = {};
  for (auto & x : v) {
    if (!(f >> x)) return std::nullopt;
  }
  CpuTimes t;
  t.idle = v[3] + v[4];
  for (auto x : v) t.total += x;
  return t;
}

float read_load_avg()
{
  std::ifstream f("/proc/loadavg");
  float load;
  return (f >> load) ? load : kNaN;
}

float read_mem_used_percent()
{
  std::ifstream f("/proc/meminfo");
  std::string key;
  uint64_t value;
  std::string unit;
  uint64_t total = 0, available = 0;
  while (f >> key >> value >> unit) {
    if (key == "MemTotal:") total = value;
    else if (key == "MemAvailable:") available = value;
  }
  if (total == 0) return kNaN;
  return 100.0f * static_cast<float>(total - available) / static_cast<float>(total);
}

float read_disk_used_percent(const std::string & path)
{
  struct statvfs s;
  if (::statvfs(path.c_str(), &s) != 0 || s.f_blocks == 0) return kNaN;
  // Matches df: used / (used + available to non-root)
  const double used = static_cast<double>(s.f_blocks - s.f_bfree);
  const double avail = static_cast<double>(s.f_bavail);
  return static_cast<float>(100.0 * used / (used + avail));
}

float read_max_thermal_c()
{
  namespace fs = std::filesystem;
  float max_c = kNaN;
  std::error_code ec;
  for (const auto & entry : fs::directory_iterator("/sys/class/thermal", ec)) {
    if (entry.path().filename().string().rfind("thermal_zone", 0) != 0) continue;
    std::ifstream f(entry.path() / "temp");
    long milli_c;
    if (!(f >> milli_c)) continue;
    const float c = static_cast<float>(milli_c) / 1000.0f;
    if (std::isnan(max_c) || c > max_c) max_c = c;
  }
  return max_c;
}

double read_uptime()
{
  std::ifstream f("/proc/uptime");
  double up;
  return (f >> up) ? up : std::numeric_limits<double>::quiet_NaN();
}

struct WifiStats
{
  std::string iface;
  float quality{kNaN};
  float signal_dbm{kNaN};
};

// /proc/net/wireless: 2 header lines, then "wlan0: 0000   70.  -40.  -256 ..."
WifiStats read_wifi(const std::string & wanted)
{
  WifiStats w;
  std::ifstream f("/proc/net/wireless");
  std::string line;
  std::getline(f, line);
  std::getline(f, line);
  while (std::getline(f, line)) {
    std::istringstream ss(line);
    std::string iface, status;
    float quality, level;
    if (!(ss >> iface >> status >> quality >> level)) continue;
    if (!iface.empty() && iface.back() == ':') iface.pop_back();
    if (!wanted.empty() && iface != wanted) continue;
    w.iface = iface;
    w.quality = quality / 70.0f;
    w.signal_dbm = level;
    break;
  }
  return w;
}

}  // namespace

class SystemMonitorNode : public rclcpp::Node
{
public:
  SystemMonitorNode()
  : rclcpp::Node("system_monitor_node")
  {
    const double period = declare_parameter<double>("update_period", 1.0);
    wifi_interface_ = declare_parameter<std::string>("wifi_interface", "");
    disk_path_ = declare_parameter<std::string>("disk_path", "/");

    pub_ = create_publisher<storm_interfaces::msg::SystemTelemetry>("/telemetry/system", 10);
    last_cpu_ = read_cpu_times();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int64_t>(period * 1000.0)),
      [this]() { publish(); });
  }

private:
  void publish()
  {
    storm_interfaces::msg::SystemTelemetry msg;
    msg.header.stamp = now();

    msg.cpu_percent = kNaN;
    const auto cpu = read_cpu_times();
    if (cpu && last_cpu_ && cpu->total > last_cpu_->total) {
      const double d_total = static_cast<double>(cpu->total - last_cpu_->total);
      const double d_idle = static_cast<double>(cpu->idle - last_cpu_->idle);
      msg.cpu_percent = static_cast<float>(100.0 * (1.0 - d_idle / d_total));
    }
    last_cpu_ = cpu;

    msg.load_avg_1m = read_load_avg();
    msg.cpu_temp_c = read_max_thermal_c();
    msg.mem_used_percent = read_mem_used_percent();
    msg.disk_used_percent = read_disk_used_percent(disk_path_);
    msg.uptime_s = read_uptime();

    const auto wifi = read_wifi(wifi_interface_);
    msg.wifi_interface = wifi.iface;
    msg.wifi_link_quality = wifi.quality;
    msg.wifi_signal_dbm = wifi.signal_dbm;

    pub_->publish(msg);
  }

  std::string wifi_interface_;
  std::string disk_path_;
  std::optional<CpuTimes> last_cpu_;

  rclcpp::Publisher<storm_interfaces::msg::SystemTelemetry>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
