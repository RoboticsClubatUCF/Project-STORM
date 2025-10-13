#ifndef HARDWARE_INTERFACE__DRIVETRAIN_HARDWARE_HPP_
#define HARDWARE_INTERFACE__DRIVETRAIN_HARDWARE_HPP_

#include <vector>
#include <string>
#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/macros.hpp"

//sparkcan motor API
#include "sparkcan/motor.hpp"

namespace hardware_interface
{

class RoverHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RoverHardware)

  RoverHardware() = default;

  // Called when plugin is first loaded
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  // Lifecycle transitions
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  // Interface definitions
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  //Command mode switch
  hardware_interface::CallbackReturn prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::CallbackReturn perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  // Main read/write hardware functions
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<std::shared_ptr<sparkcan::Motor>> motors;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> commands;

  std::string can_interface;
  std::vector<uint32_t> motor_ids;

  double wheel_separation;
  double wheel_radius;
  double left_wheel_radius_multiplier;
  double right_wheel_radius_multiplier;

  std::vector<std::string> left_wheel_names;
  std::vector<std::string> right_wheel_names;

  bool position_feedback;

};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__ROVER_HARDWARE_HPP_
