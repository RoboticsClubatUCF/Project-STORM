// spark_monitor_node.cpp: per-SPARK online status on /diagnostics
//
// Reads the plugin's "online" and "voltage" state interfaces from
// /dynamic_joint_states. A dropped SPARK doesn't stop the rover (limp mode),
// so this is where it shows up.

#include <chrono>
#include <map>
#include <memory>
#include <string>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;

namespace
{

KeyValue kv(const std::string & key, const std::string & value)
{
  KeyValue k;
  k.key = key;
  k.value = value;
  return k;
}

}  // namespace

class SparkMonitorNode : public rclcpp::Node
{
public:
  SparkMonitorNode()
  : rclcpp::Node("spark_monitor_node")
  {
    const double period = declare_parameter<double>("update_period", 1.0);
    stale_timeout_ = declare_parameter<double>("stale_timeout", 2.0);

    pub_ = create_publisher<DiagnosticArray>("/diagnostics", 10);
    sub_ = create_subscription<control_msgs::msg::DynamicJointState>(
      "/dynamic_joint_states", rclcpp::SensorDataQoS(),
      [this](control_msgs::msg::DynamicJointState::ConstSharedPtr msg) { on_states(*msg); });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int64_t>(period * 1000.0)),
      [this]() { publish(); });
  }

private:
  struct Spark
  {
    bool online{false};
    bool ever_online{false};
    double voltage{0.0};
    int drops{0};
  };

  void on_states(const control_msgs::msg::DynamicJointState & msg)
  {
    for (size_t j = 0; j < msg.joint_names.size() && j < msg.interface_values.size(); ++j) {
      const auto & iv = msg.interface_values[j];
      bool has_online = false;
      bool online = false;
      double voltage = 0.0;
      for (size_t i = 0; i < iv.interface_names.size() && i < iv.values.size(); ++i) {
        if (iv.interface_names[i] == "online") {
          has_online = true;
          online = iv.values[i] > 0.5;
        } else if (iv.interface_names[i] == "voltage") {
          voltage = iv.values[i];
        }
      }
      if (!has_online) continue;

      Spark & s = sparks_[msg.joint_names[j]];
      if (s.online && !online) ++s.drops;
      s.online = online;
      s.ever_online = s.ever_online || online;
      s.voltage = voltage;
    }
    last_rx_ = now();
    have_rx_ = true;
  }

  void publish()
  {
    DiagnosticArray arr;
    arr.header.stamp = now();
    const bool fresh = have_rx_ && (now() - last_rx_).seconds() < stale_timeout_;

    size_t online_count = 0;
    for (const auto & [joint, s] : sparks_) {
      DiagnosticStatus st;
      st.name = "SPARK " + joint;
      st.hardware_id = joint;
      if (!fresh) {
        st.level = DiagnosticStatus::STALE;
        st.message = "no data from ros2_control";
      } else if (s.online) {
        st.level = DiagnosticStatus::OK;
        st.message = "online";
        ++online_count;
      } else {
        st.level = DiagnosticStatus::ERROR;
        st.message = s.ever_online ? "OFFLINE, limping" : "never seen";
      }
      st.values.push_back(kv("online", s.online ? "true" : "false"));
      st.values.push_back(kv("voltage", std::to_string(s.voltage)));
      st.values.push_back(kv("drops", std::to_string(s.drops)));
      arr.status.push_back(st);
    }

    DiagnosticStatus summary;
    summary.name = "Drivetrain";
    summary.hardware_id = "drivetrain";
    const std::string count = std::to_string(online_count) + "/" + std::to_string(sparks_.size());
    if (!fresh) {
      summary.level = DiagnosticStatus::STALE;
      summary.message = "no data from ros2_control";
    } else if (online_count == sparks_.size()) {
      summary.level = DiagnosticStatus::OK;
      summary.message = count + " SPARKs online";
    } else if (online_count > 0) {
      summary.level = DiagnosticStatus::WARN;
      summary.message = "limp mode: " + count + " SPARKs online";
    } else {
      summary.level = DiagnosticStatus::ERROR;
      summary.message = "no SPARKs online";
    }
    summary.values.push_back(kv("online", count));
    arr.status.push_back(summary);

    pub_->publish(arr);
  }

  double stale_timeout_;
  std::map<std::string, Spark> sparks_;
  rclcpp::Time last_rx_;
  bool have_rx_{false};

  rclcpp::Publisher<DiagnosticArray>::SharedPtr pub_;
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SparkMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
