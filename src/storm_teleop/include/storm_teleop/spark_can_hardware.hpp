#ifndef STORM_TELEOP__SPARK_CAN_HARDWARE_HPP_
#define STORM_TELEOP__SPARK_CAN_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

// sparkcan — installed via PPA (sudo apt install sparkcan)
#include <SparkMax.hpp>

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
  double gear_ratio_{81.0};

  // Conversion constants (computed once in on_init)
  //   wheel_rad_s = motor_rpm * rads_per_rpm
  //   motor_rpm   = wheel_rad_s * rpm_per_rads
  double rads_per_rpm_{0.0};  // (2π / 60) / gear_ratio
  double rpm_per_rads_{0.0};  // (60 / 2π) * gear_ratio

  //   wheel_rad = motor_rotations * rad_per_rot
  double rad_per_rot_{0.0};   // 2π / gear_ratio

  // SPARK PID starting values
  float pid_kp_{0.00005f};
  float pid_ki_{0.0f};
  float pid_kd_{0.0f};
  float pid_kf_{0.000176f};

  /// Stop all motors immediately (duty cycle 0)
  void stop_all();
};

}  // namespace storm_teleop

#endif  // STORM_TELEOP__SPARK_CAN_HARDWARE_HPP_
