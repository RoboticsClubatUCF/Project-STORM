// battery_monitor_node.cpp: pack voltage from the SPARKs on /telemetry/battery
//
// SparkCanHardware owns the CAN bus, so voltage comes from its "voltage" state
// interface, which joint_state_broadcaster publishes on /dynamic_joint_states.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "storm_interfaces/msg/battery_status.hpp"

using storm_interfaces::msg::BatteryStatus;

class BatteryMonitorNode : public rclcpp::Node
{
public:
  BatteryMonitorNode()
  : rclcpp::Node("battery_monitor_node")
  {
    const double period = declare_parameter<double>("update_period", 1.0);
    stale_timeout_ = declare_parameter<double>("stale_timeout", 2.0);
    // 0 disables the level check
    low_voltage_ = declare_parameter<double>("low_voltage", 0.0);
    critical_voltage_ = declare_parameter<double>("critical_voltage", 0.0);
    // Smooths out sag from acceleration so the level tracks charge, not load
    filter_tau_ = declare_parameter<double>("filter_time_constant", 5.0);

    pub_ = create_publisher<BatteryStatus>("/telemetry/battery", 10);
    sub_ = create_subscription<control_msgs::msg::DynamicJointState>(
      "/dynamic_joint_states", rclcpp::SensorDataQoS(),
      [this](control_msgs::msg::DynamicJointState::ConstSharedPtr msg) { on_states(*msg); });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int64_t>(period * 1000.0)),
      [this]() { publish(); });
  }

private:
  void on_states(const control_msgs::msg::DynamicJointState & msg)
  {
    BatteryStatus s;
    s.min_voltage = std::numeric_limits<float>::infinity();
    s.max_voltage = -std::numeric_limits<float>::infinity();
    double sum = 0.0;

    for (const auto & iv : msg.interface_values) {
      std::optional<double> voltage;
      bool online = false;
      for (size_t i = 0; i < iv.interface_names.size() && i < iv.values.size(); ++i) {
        if (iv.interface_names[i] == "voltage") voltage = iv.values[i];
        else if (iv.interface_names[i] == "online") online = iv.values[i] > 0.5;
      }
      if (!voltage) continue;
      ++s.sources_total;
      if (!online) continue;
      ++s.sources_online;
      sum += *voltage;
      s.min_voltage = std::min(s.min_voltage, static_cast<float>(*voltage));
      s.max_voltage = std::max(s.max_voltage, static_cast<float>(*voltage));
    }

    const rclcpp::Time t = now();
    if (s.sources_online > 0) {
      s.voltage = static_cast<float>(sum / s.sources_online);
      if (std::isnan(filtered_min_) || filter_tau_ <= 0.0) {
        filtered_min_ = s.min_voltage;
      } else {
        const double dt = (t - last_rx_).seconds();
        filtered_min_ += dt / (filter_tau_ + dt) * (s.min_voltage - filtered_min_);
      }
    } else {
      s.voltage = s.min_voltage = s.max_voltage = std::numeric_limits<float>::quiet_NaN();
      filtered_min_ = std::numeric_limits<double>::quiet_NaN();
    }
    latest_ = s;
    last_rx_ = t;
  }

  void publish()
  {
    BatteryStatus msg;
    const bool fresh = latest_ && (now() - last_rx_).seconds() < stale_timeout_;
    if (fresh) {
      msg = *latest_;
      msg.level = level_for(msg);
    } else {
      msg.voltage = msg.min_voltage = msg.max_voltage = std::numeric_limits<float>::quiet_NaN();
      msg.level = BatteryStatus::LEVEL_UNKNOWN;
    }
    msg.header.stamp = now();
    pub_->publish(msg);
  }

  uint8_t level_for(const BatteryStatus & s) const
  {
    if (s.sources_online == 0 || low_voltage_ <= 0.0 || std::isnan(filtered_min_)) {
      return BatteryStatus::LEVEL_UNKNOWN;
    }
    if (critical_voltage_ > 0.0 && filtered_min_ <= critical_voltage_) {
      return BatteryStatus::LEVEL_CRITICAL;
    }
    if (filtered_min_ <= low_voltage_) return BatteryStatus::LEVEL_LOW;
    return BatteryStatus::LEVEL_OK;
  }

  double stale_timeout_;
  double low_voltage_;
  double critical_voltage_;
  double filter_tau_;
  double filtered_min_{std::numeric_limits<double>::quiet_NaN()};

  std::optional<BatteryStatus> latest_;
  rclcpp::Time last_rx_;

  rclcpp::Publisher<BatteryStatus>::SharedPtr pub_;
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
