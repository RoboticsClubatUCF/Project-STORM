// =============================================================================
// can_health_node.cpp
//
// Standalone diagnostics node for the drivetrain CAN bus. Publishes
// diagnostic_msgs/DiagnosticArray on /diagnostics so the CAN bus shows up in
// rqt_robot_monitor and can be logged/monitored while driving.
//
// IMPORTANT: this node NEVER opens a CAN socket. The ros2_control hardware
// plugin (SparkCanHardware) already owns the bus via sparkcan; a second bus
// opener could contend on transmit. Instead we read health passively from:
//   - sysfs   (/sys/class/net/<iface>/...): operstate + rx/tx error counters
//   - netlink (`ip -details -statistics link show <iface>`): CAN controller
//             state (ERROR-ACTIVE/WARNING/PASSIVE/BUS-OFF) + berr counters,
//             which are NOT exposed in plain sysfs.
//
// Note: in CAN, "ERROR-ACTIVE" is the *normal* healthy operating state.
// =============================================================================

#include <array>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <memory>
#include <optional>
#include <regex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"

namespace
{

// Read a whole sysfs file into a trimmed string.
std::optional<std::string> read_sysfs(const std::string & path)
{
  std::ifstream f(path);
  if (!f) return std::nullopt;
  std::string v;
  std::getline(f, v);
  // trim trailing whitespace/newline
  while (!v.empty() && (v.back() == '\n' || v.back() == '\r' || v.back() == ' ')) {
    v.pop_back();
  }
  return v;
}

std::optional<uint64_t> read_sysfs_u64(const std::string & path)
{
  auto s = read_sysfs(path);
  if (!s || s->empty()) return std::nullopt;
  try {
    return static_cast<uint64_t>(std::stoull(*s));
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

// Run a command and capture stdout. Returns nullopt if the command can't run.
std::optional<std::string> run_capture(const std::string & cmd)
{
  std::array<char, 512> buf{};
  std::string out;
  FILE * pipe = ::popen(cmd.c_str(), "r");
  if (!pipe) return std::nullopt;
  while (std::fgets(buf.data(), static_cast<int>(buf.size()), pipe) != nullptr) {
    out += buf.data();
  }
  ::pclose(pipe);
  return out;
}

}  // namespace

class CanHealthNode : public rclcpp::Node
{
public:
  CanHealthNode()
  : rclcpp::Node("can_health_node"),
    updater_(this)
  {
    iface_ = this->declare_parameter<std::string>("interface", "can0");
    const double period = this->declare_parameter<double>("update_period", 1.0);
    // WARN if rx+tx errors climb by more than this between updates.
    error_delta_warn_ = this->declare_parameter<int>("error_delta_warn", 5);

    updater_.setHardwareID(iface_);
    updater_.add("CAN link (" + iface_ + ")", this, &CanHealthNode::link_diag);
    updater_.add("CAN controller (" + iface_ + ")", this, &CanHealthNode::controller_diag);
    updater_.add("CAN traffic (" + iface_ + ")", this, &CanHealthNode::traffic_diag);

    const auto period_ms = std::chrono::milliseconds(
      static_cast<int64_t>(period * 1000.0));
    timer_ = this->create_wall_timer(
      period_ms, [this]() { updater_.force_update(); });

    RCLCPP_INFO(get_logger(),
      "can_health_node monitoring '%s' every %.1fs (sysfs/netlink only).",
      iface_.c_str(), period);
  }

private:
  std::string sysfs(const std::string & leaf) const
  {
    return "/sys/class/net/" + iface_ + "/" + leaf;
  }

  // --- Link up/down (pure sysfs) ---
  void link_diag(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    auto oper = read_sysfs(sysfs("operstate"));
    if (!oper) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "interface not found. Is the CAN adapter plugged in?");
      stat.add("operstate", "unavailable");
      return;
    }
    stat.add("operstate", *oper);
    // SocketCAN links usually report "up" (or "unknown" on some drivers while
    // carrying traffic); "down" is the unambiguous failure.
    if (*oper == "down") {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        iface_ + " is DOWN, run can0_up.sh / enable storm-can0.service");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "up");
    }
  }

  // --- CAN controller state (netlink via `ip`) ---
  void controller_diag(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    auto out = run_capture("ip -details -statistics link show " + iface_ +
                           " 2>/dev/null");
    if (!out || out->empty()) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "could not query controller state via ip");
      return;
    }

    std::smatch m;
    std::string state = "unknown";
    if (std::regex_search(*out, m, std::regex(R"(can state (\S+))"))) {
      state = m[1].str();
    }
    stat.add("state", state);

    if (std::regex_search(*out, m,
        std::regex(R"(berr-counter tx (\d+) rx (\d+))"))) {
      stat.add("berr_tx", m[1].str());
      stat.add("berr_rx", m[2].str());
    }
    if (std::regex_search(*out, m, std::regex(R"(restart-ms (\d+))"))) {
      stat.add("restart_ms", m[1].str());
    }

    // ERROR-ACTIVE is the normal healthy state in CAN.
    if (state == "BUS-OFF") {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "BUS-OFF, controller offline (auto-recovers if restart-ms set)");
    } else if (state == "ERROR-PASSIVE" || state == "ERROR-WARNING") {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "degraded: " + state);
    } else if (state == "ERROR-ACTIVE") {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "ERROR-ACTIVE (normal)");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, state);
    }
  }

  // --- Traffic / error counters (pure sysfs, delta-based) ---
  void traffic_diag(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    const auto rx_err = read_sysfs_u64(sysfs("statistics/rx_errors"));
    const auto tx_err = read_sysfs_u64(sysfs("statistics/tx_errors"));
    const auto rx_pkt = read_sysfs_u64(sysfs("statistics/rx_packets"));
    const auto tx_pkt = read_sysfs_u64(sysfs("statistics/tx_packets"));

    if (!rx_err || !tx_err) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "error counters unavailable");
      return;
    }

    const uint64_t total_err = *rx_err + *tx_err;
    stat.add("rx_errors", std::to_string(*rx_err));
    stat.add("tx_errors", std::to_string(*tx_err));
    if (rx_pkt) stat.add("rx_packets", std::to_string(*rx_pkt));
    if (tx_pkt) stat.add("tx_packets", std::to_string(*tx_pkt));

    // First pass: establish a baseline, report OK.
    if (!have_baseline_) {
      last_total_err_ = total_err;
      have_baseline_ = true;
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "monitoring started");
      return;
    }

    const uint64_t delta = (total_err >= last_total_err_)
      ? total_err - last_total_err_ : 0;
    last_total_err_ = total_err;
    stat.add("errors_since_last", std::to_string(delta));

    if (delta > static_cast<uint64_t>(error_delta_warn_)) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "rising CAN errors (+" + std::to_string(delta) + " since last check)");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
        "errors stable (total " + std::to_string(total_err) + ")");
    }
  }

  std::string iface_;
  int error_delta_warn_{5};
  bool have_baseline_{false};
  uint64_t last_total_err_{0};

  diagnostic_updater::Updater updater_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanHealthNode>());
  rclcpp::shutdown();
  return 0;
}
