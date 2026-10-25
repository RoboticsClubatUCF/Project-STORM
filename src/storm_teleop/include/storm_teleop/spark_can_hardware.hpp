#ifndef STORM_TELEOP__SPARK_CAN_HARDWARE_HPP_
#define STORM_TELEOP__SPARK_CAN_HARDWARE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

// sparkcan, installed via PPA (sudo apt install sparkcan)
#include <SparkMax.hpp>

#include "storm_teleop/spark_status_reader.hpp"

namespace storm_teleop
{

/// Per-wheel bookkeeping: sparkcan handle + ROS interface values
struct WheelHandle
{
  std::string joint_name;
  uint8_t can_id{0};
  bool inverted{false};

  // sparkcan motor controller object (created in on_configure)
  std::unique_ptr<SparkMax> spark;

  // Values exchanged with ros2_control each cycle
  double cmd_vel{0.0};    // command: wheel angular velocity [rad/s]
  double state_pos{0.0};  // state:   wheel angular position [rad]
  double state_vel{0.0};  // state:   wheel angular velocity [rad/s]
  double state_voltage{0.0};  // state: SPARK input (battery) voltage [V]
  double state_online{0.0};   // state: 1 if status frames are arriving

  bool online{false};
  bool needs_config{false};  // set when a SPARK comes back or reports a reset
  std::chrono::steady_clock::time_point configured_at{};
};

class SparkCanHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SparkCanHardware)

  // --- Lifecycle callbacks ---
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  // --- Interface exports ---
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // --- Real-time loop ---
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Wheels (populated in on_init from URDF params)
  std::vector<WheelHandle> wheels_;

  // Config read from URDF <hardware> params
  std::string can_interface_;
  double gear_ratio_{9.0};

  // When true, persist the applied config to each SPARK's flash in
  // on_configure. Leave false for normal use; on_configure re-applies the
  // full config over CAN every launch, so flashing each boot only wears the
  // limited flash write cycles. Set true for a single launch when you want to
  // commit a new persistent baseline to the devices.
  bool burn_flash_{false};

  // Pre-flight gate: when true, on_configure fails (controller won't activate)
  // if any SPARK is missing from the bus. Default true; set the
  // require_all_sparks URDF param to false to allow coming up with fewer.
  bool require_all_sparks_{true};

  // Status 1 arrives every ~20 ms, so 100 ms is 5 missed frames
  int status_timeout_ms_{100};

  // Sticky fault bit set after the SPARK reboots (REV FaultID kHasReset)
  static constexpr uint16_t kFaultHasReset = 1u << 9;

  SparkStatusReader status_reader_;

  // Conversion constants (computed once in on_init)
  //   wheel_rad_s = motor_rpm * rads_per_rpm
  //   motor_rpm   = wheel_rad_s * rpm_per_rads
  double rads_per_rpm_{0.0};  // (2π / 60) / gear_ratio
  double rpm_per_rads_{0.0};  // (60 / 2π) * gear_ratio

  //   wheel_rad = motor_rotations * rad_per_rot
  double rad_per_rot_{0.0};   // 2π / gear_ratio

  // SPARK PID starting values, fallbacks used only when config/sparks.yaml is
  // missing/unreadable or omits the matching key (see load_pid_from_yaml).
  // Keep these in step with sparks.yaml so a missing file doesn't silently
  // change behavior.
  //   kF = 1 / free_speed_RPM ≈ 1/5676 for NEO V1.1
  float pid_kp_{0.0001f};
  float pid_ki_{0.0f};
  float pid_kd_{0.0f};
  float pid_kf_{0.000176f};

  /// Load PID gains from the SPARK config YAML at `path`. Any key that is
  /// absent (or an unreadable/malformed file) leaves the corresponding member
  /// at its header default. Called from on_init.
  void load_pid_from_yaml(const std::string & path);

  /// Push motor type, brake mode, control mode, inversion and PID to one SPARK.
  void apply_config(WheelHandle & wheel, bool burn_flash);

  /// Stop all motors immediately (duty cycle 0)
  void stop_all();
};

}  // namespace storm_teleop

#endif  // STORM_TELEOP__SPARK_CAN_HARDWARE_HPP_
