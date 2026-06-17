// =============================================================================
// spark_can_hardware.cpp
//
// ros2_control SystemInterface plugin that bridges diff_drive_controller
// to 6× REV SPARK MAX motor controllers via sparkcan (Linux SocketCAN).
//
// Data flow each cycle:
//   read()  — GetVelocity/GetPosition from each SPARK, convert motor
//             RPM/rotations → wheel rad/s and rad
//   write() — convert wheel rad/s → motor RPM, call SetVelocity on each SPARK
//
// The SPARK MAX runs its own onboard velocity PID, so we send velocity
// setpoints (not duty cycle). This keeps the control loop fast (~1 kHz on
// the SPARK) regardless of the ros2_control update rate (50 Hz).
// =============================================================================

#include "storm_teleop/spark_can_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace storm_teleop
{

// Helper

static constexpr double kTwoPi = 2.0 * M_PI;

// on_init
// This is called once when the controller_manager first loads the plugin, it 
// parses all parameters from the URDF <ros2_control> block

hardware_interface::CallbackReturn SparkCanHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // Call base class (stores info_ for later use)
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // These are Hardware-level params (from <hardware><param> tags)
  can_interface_ = info_.hardware_parameters.at("can_interface");
  gear_ratio_    = std::stod(info_.hardware_parameters.at("gear_ratio"));
  pid_kp_ = std::stof(info_.hardware_parameters.at("pid_kp"));
  pid_ki_ = std::stof(info_.hardware_parameters.at("pid_ki"));
  pid_kd_ = std::stof(info_.hardware_parameters.at("pid_kd"));
  pid_kf_ = std::stof(info_.hardware_parameters.at("pid_kf"));

  // These are the conversion constants for our drive train
  // motor_rpm = wheel_rad_s =        rpm x (2pi/60) / gear_ratio
  // wheel_rad_s = motor_rpm =        rad_s x (60/2pi) x gear_ratio
  // motor_rotations = wheel_rad =    rotations x 2pi / gear_ratio
  rads_per_rpm_ = (kTwoPi / 60.0) / gear_ratio_;
  rpm_per_rads_ = (60.0 / kTwoPi) * gear_ratio_;
  rad_per_rot_  = kTwoPi / gear_ratio_;

  RCLCPP_INFO(rclcpp::get_logger("SparkCanHardware"),
    "Gear ratio: %.1f | rads_per_rpm: %.6f | rpm_per_rads: %.2f",
    gear_ratio_, rads_per_rpm_, rpm_per_rads_);

  // These are the Per-joint params (from <joint><param> tags)
  wheels_.resize(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    auto & joint = info_.joints[i];
    auto & wheel = wheels_[i];

    wheel.joint_name = joint.name;
    wheel.can_id     = static_cast<uint8_t>(std::stoi(joint.parameters.at("can_id")));
    wheel.inverted   = (joint.parameters.at("inverted") == "true");

    // Make sure each each joint has exactly 1 command interface (velocity) and 2 state interfaces (position, velocity)
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(rclcpp::get_logger("SparkCanHardware"),
        "Joint '%s' must have exactly one velocity command interface.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_ERROR(rclcpp::get_logger("SparkCanHardware"),
        "Joint '%s' must have exactly two state interfaces (position, velocity).",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("SparkCanHardware"),
      "  joint: %-30s  CAN ID: %d  inverted: %s",
      joint.name.c_str(), wheel.can_id, wheel.inverted ? "yes" : "no");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// on_configure
// This is called on transition to "configured" state. It creates SparkMax 
// objects and sends one-time configuration.

hardware_interface::CallbackReturn SparkCanHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SparkCanHardware"),
    "Configuring %zu SPARK MAXs on interface '%s'...",
    wheels_.size(), can_interface_.c_str());

  size_t online_count = 0;

  
  //Diagnostic mode: skip all setter calls. Construct SparkMax objects only,
  // so the background read thread starts and write() can call SetVelocity.
  // The SPARKs use whatever was burned to flash via REV Hardware Client
  // (motor type, brake mode, control type, inversion, PID gains).
  // Once we know SetVelocity alone works, we can re-enable setters one at a
  // time to find which one breaks PID output.
  for (auto & wheel : wheels_) {
    try {
      wheel.spark = std::make_unique<SparkMax>(can_interface_, wheel.can_id);

      //One-time motor configuration
      // NEO V1.1 is brushless
      wheel.spark->SetMotorType(MotorType::kBrushless);

      // Brake mode: when velocity command is zero, actively resist motion.
      // Critical for "stop immediately" safety requirement.
      wheel.spark->SetIdleMode(IdleMode::kBrake);

      // Velocity closed-loop control mode on the SPARK's onboard PID.
      wheel.spark->SetCtrlType(CtrlType::kVelocity);

      // Motor direction (left side inverted so positive = forward for both sides)
      wheel.spark->SetInverted(wheel.inverted);

      // We do conversion in this code rather than on the SPARK, so leave
      // the SPARK's native units (RPM / rotations). This makes raw CAN
      // debugging easier, you see real motor RPM on the wire.
      spark->SetVelocityConversionFactor(1.0);  // default
      spark->SetPositionConversionFactor(1.0);  // default

      //Onboard velocity PID
      // Slot 0 is the default control slot.
      // kF (feedforward) does most of the work: output ≈ kF × setpoint_rpm.
      // kP adds a small correction for steady-state error.
      // Tune these on the real robot!
      wheel.spark->SetP(0, pid_kp_);
      wheel.spark->SetI(0, pid_ki_);
      wheel.spark->SetD(0, pid_kd_);
      wheel.spark->SetF(0, pid_kf_);

      // Persist config to flash so it survives power cycles.
      // Comment this out during active PID tuning (flash has limited writes).
      wheel.spark->BurnFlash();

      // Clear any lingering faults from previous sessions
      wheel.spark->ClearStickyFaults();

    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("SparkCanHardware"),
        "  CAN %d (%s): FAILED — %s", wheel.can_id, wheel.joint_name.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  //Presence check
  // CAN ACKs are bus-wide, not per-recipient, so the setter calls above
  // succeed as long as ANY device is on the bus, they can't tell us which
  // specific SPARKs actually exist.
  //
  // Real check: sparkcan spawns a background thread per SPARK that parses
  // incoming periodic status frames. Period 1 carries bus voltage at ~50 Hz.
  // 500 ms is enough for all per-device threads to start, get scheduled, and
  // catch several frames even under bus contention.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  for (auto & wheel : wheels_) {
    const float voltage = wheel.spark->GetVoltage();
    if (voltage > 5.0f) {
      RCLCPP_INFO(rclcpp::get_logger("SparkCanHardware"),
        "  CAN %d (%s): ONLINE — bus %.2f V",
        wheel.can_id, wheel.joint_name.c_str(), voltage);
      ++online_count;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("SparkCanHardware"),
        "  CAN %d (%s): OFFLINE — no status frames received",
        wheel.can_id, wheel.joint_name.c_str());
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("SparkCanHardware"),
    "Configure complete: %zu/%zu SPARK MAXs responding.",
    online_count, wheels_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

//export interfaces
// These register the double* pointers that ros2_control reads/writes
// each cycle. diff_drive_controller writes to cmd_vel and reads
// state_pos / state_vel.

std::vector<hardware_interface::StateInterface>
SparkCanHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (auto & wheel : wheels_) {
    interfaces.emplace_back(
      wheel.joint_name, hardware_interface::HW_IF_POSITION, &wheel.state_pos);
    interfaces.emplace_back(
      wheel.joint_name, hardware_interface::HW_IF_VELOCITY, &wheel.state_vel);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
SparkCanHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (auto & wheel : wheels_) {
    interfaces.emplace_back(
      wheel.joint_name, hardware_interface::HW_IF_VELOCITY, &wheel.cmd_vel);
  }
  return interfaces;
}

//on_activate / on_deactivate

hardware_interface::CallbackReturn SparkCanHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SparkCanHardware"), "Activating — zeroing commands.");
  for (auto & wheel : wheels_) {
    wheel.cmd_vel   = 0.0;
    wheel.state_pos = 0.0;
    wheel.state_vel = 0.0;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SparkCanHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SparkCanHardware"), "Deactivating — stopping motors.");
  stop_all();
  return hardware_interface::CallbackReturn::SUCCESS;
}

//on_cleanup
hardware_interface::CallbackReturn SparkCanHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SparkCanHardware"), "Cleaning up — releasing SPARK handles.");
  stop_all();
  for (auto & wheel : wheels_) {
    wheel.spark.reset();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

//read()
// Called every cycle BEFORE the controllers run.
// Pulls encoder data from each SPARK and converts to wheel-frame SI units.

hardware_interface::return_type SparkCanHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (auto & wheel : wheels_) {
    if (!wheel.spark) continue;

    // GetVelocity() returns motor shaft RPM (native SPARK unit).
    // GetPosition() returns motor shaft rotations (native SPARK unit).
    float motor_rpm       = wheel.spark->GetVelocity();
    float motor_rotations = wheel.spark->GetPosition();

    // Convert to wheel-frame SI units for ros2_control:
    //   wheel rad/s = motor_rpm x (2pi/60) / gear_ratio
    //   wheel rad   = motor_rotations x 2pi / gear_ratio
    wheel.state_vel = static_cast<double>(motor_rpm) * rads_per_rpm_;
    wheel.state_pos = static_cast<double>(motor_rotations) * rad_per_rot_;
  }

  return hardware_interface::return_type::OK;
}

//write()
// Called every cycle AFTER the controllers run.
// Converts wheel rad/s commands to motor RPM and sends to each SPARK.

  hardware_interface::return_type SparkCanHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  //Send a single static heartbeat frame to keep all CAN motor watchdogs happy
  SparkBase::Heartbeat();

  //Update the control commands for each wheel
  for (auto & wheel : wheels_) {
    if (!wheel.spark) continue;

    // Convert wheel angular velocity (rad/s) to motor shaft RPM
    float motor_rpm = static_cast<float>(wheel.cmd_vel * rpm_per_rads_);

    // Send the updated target velocity
    wheel.spark->SetVelocity(motor_rpm);
  }

  return hardware_interface::return_type::OK;
}

//stop_all

void SparkCanHardware::stop_all()
{
  for (auto & wheel : wheels_) {
    if (wheel.spark) {
      // SetDutyCycle(0) is the most reliable way to command a hard stop,
      // bypassing the velocity PID entirely.
      wheel.spark->SetDutyCycle(0.0f);
    }
  }
}

}  // namespace storm_teleop

//Register with pluginlib so controller_manager can discover this
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(storm_teleop::SparkCanHardware, hardware_interface::SystemInterface)
