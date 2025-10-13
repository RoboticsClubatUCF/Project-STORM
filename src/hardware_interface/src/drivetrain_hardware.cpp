#include "drivetrain_hardware/drivetrain_hardware.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hardware_interface
{

// Called when the hardware plugin is loaded
// Initialize motors, CAN interface, and state vectors
hardware_interface::CallbackReturn DrivetrainHardware::on_init(
  const hardware_interface::HardwareInfo & /* info */)
{
  RCLCPP_INFO(rclcpp::get_logger("DrivetrainHardware"), "Initializing DrivetrainHardware...");

  // Hardcoded CAN interface and motor IDs
  can_interface = "can0";
  motor_ids = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

  // Create SPARK motor objects for each motor ID
  motors.clear();
  for (auto id : motor_ids)
    motors.push_back(std::make_shared<sparkcan::Motor>(id, can_interface));

  // Initialize state vectors for positions, velocities, and commands
  const size_t n_motors = motors.size();
  positions.resize(n_motors, 0.0);
  velocities.resize(n_motors, 0.0);
  commands.resize(n_motors, 0.0);

  // Assume motors provide position feedback
  position_feedback = true;

  RCLCPP_INFO(rclcpp::get_logger("DrivetrainHardware"), "Initialization complete.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// Lifecycle callbacks: configure, cleanup, shutdown
// Minimal implementation, return SUCCESS
hardware_interface::CallbackReturn DrivetrainHardware::on_configure(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DrivetrainHardware::on_cleanup(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DrivetrainHardware::on_shutdown(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

// Called when hardware interface is activated
// Reset commands to zero for safety
hardware_interface::CallbackReturn DrivetrainHardware::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  std::fill(commands.begin(), commands.end(), 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// Called when hardware interface is deactivated
// Stop all motors to prevent movement
hardware_interface::CallbackReturn DrivetrainHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  for (auto & motor : motors)
    motor->set_duty_cycle(0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// Called when an error occurs
// Stop motors and return FAILURE
hardware_interface::CallbackReturn DrivetrainHardware::on_error(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  for (auto & motor : motors)
    motor->set_duty_cycle(0.0);
  return hardware_interface::CallbackReturn::FAILURE;
}

// Export state interfaces (readable by controllers)
// Position and velocity for each motor
std::vector<hardware_interface::StateInterface> DrivetrainHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < motors.size(); ++i)
  {
    state_interfaces.emplace_back("motor_" + std::to_string(motor_ids[i]), "position", &positions[i]);
    state_interfaces.emplace_back("motor_" + std::to_string(motor_ids[i]), "velocity", &velocities[i]);
  }
  return state_interfaces;
}

// Export command interfaces (writable by controllers)
// Velocity commands for each motor
std::vector<hardware_interface::CommandInterface> DrivetrainHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < motors.size(); ++i)
    command_interfaces.emplace_back("motor_" + std::to_string(motor_ids[i]), "velocity", &commands[i]);
  return command_interfaces;
}

// Prepare command mode switch (optional)
// Always return SUCCESS since all motors use the same mode
hardware_interface::CallbackReturn DrivetrainHardware::prepare_command_mode_switch(
  const std::vector<std::string> & /* start_interfaces */,
  const std::vector<std::string> & /* stop_interfaces */)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

// Perform command mode switch (optional)
// Always return SUCCESS
hardware_interface::CallbackReturn DrivetrainHardware::perform_command_mode_switch(
  const std::vector<std::string> & /* start_interfaces */,
  const std::vector<std::string> & /* stop_interfaces */)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

// Read motor states from hardware
// Updates positions and velocities vectors
hardware_interface::return_type DrivetrainHardware::read(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  for (size_t i = 0; i < motors.size(); ++i)
  {
    velocities[i] = motors[i]->get_velocity();
    if (position_feedback)
      positions[i] = motors[i]->get_position();
  }
  return hardware_interface::return_type::OK;
}

// Write commands to motors
// Sends values from commands vector to each motor
hardware_interface::return_type DrivetrainHardware::write(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  for (size_t i = 0; i < motors.size(); ++i)
    motors[i]->set_duty_cycle(commands[i]);
  return hardware_interface::return_type::OK;
}

}  // namespace hardware_interface
