// =============================================================================
// spark_can_hardware.cpp
//
// ros2_control SystemInterface plugin that bridges diff_drive_controller
// to 6× REV SPARK MAX motor controllers via sparkcan (Linux SocketCAN).
//
// Data flow each cycle:
//   read():  latest status from SparkStatusReader, convert motor
//             RPM/rotations → wheel rad/s and rad, flag dropped SPARKs
//   write(): convert wheel rad/s → motor RPM, call SetVelocity on each SPARK
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

#include <yaml-cpp/yaml.h>

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

  // Required hardware-level params (from <hardware><param> tags).
  can_interface_ = info_.hardware_parameters.at("can_interface");
  gear_ratio_    = std::stod(info_.hardware_parameters.at("gear_ratio"));

  // PID gains live in a dedicated SPARK config file (config/sparks.yaml). The
  // launch file resolves its absolute path and passes it in as the optional
  // `spark_config` hardware param. Precedence per gain:
  //   sparks.yaml value -> header default (members already hold the default).
  // A missing param, missing file, or missing key is non-fatal; we warn and
  // keep the header default so the robot still comes up.
  auto cfg_it = info_.hardware_parameters.find("spark_config");
  if (cfg_it == info_.hardware_parameters.end() || cfg_it->second.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("SparkCanHardware"),
      "No 'spark_config' param set, using built-in default PID gains.");
  } else {
    load_pid_from_yaml(cfg_it->second);
  }

  // Optional: persist config to flash this launch (default false). See header.
  auto burn_it = info_.hardware_parameters.find("burn_flash");
  burn_flash_ = (burn_it != info_.hardware_parameters.end() &&
                 burn_it->second == "true");

  // Optional pre-flight gate (default true): when set, on_configure fails if
  // any SPARK is missing from the bus, so the rover won't activate with a dead
  // wheel. Set false to allow limping up with fewer than all six.
  auto gate_it = info_.hardware_parameters.find("require_all_sparks");
  require_all_sparks_ = (gate_it == info_.hardware_parameters.end() ||
                         gate_it->second == "true");

  // How long without a status frame before a SPARK counts as dropped
  auto timeout_it = info_.hardware_parameters.find("status_timeout_ms");
  if (timeout_it != info_.hardware_parameters.end()) {
    status_timeout_ms_ = std::stoi(timeout_it->second);
  }

  // Drivetrain conversion constants
  //   wheel_rad_s = motor_rpm * (2pi/60) / gear_ratio
  //   motor_rpm   = wheel_rad_s * (60/2pi) * gear_ratio
  //   wheel_rad   = motor_rotations * 2pi / gear_ratio
  rads_per_rpm_ = (kTwoPi / 60.0) / gear_ratio_;
  rpm_per_rads_ = (60.0 / kTwoPi) * gear_ratio_;
  rad_per_rot_  = kTwoPi / gear_ratio_;

  RCLCPP_INFO(rclcpp::get_logger("SparkCanHardware"),
    "Gear ratio: %.1f | rads_per_rpm: %.6f | rpm_per_rads: %.2f",
    gear_ratio_, rads_per_rpm_, rpm_per_rads_);

  RCLCPP_INFO(rclcpp::get_logger("SparkCanHardware"),
    "Effective PID gains: kP=%.6f kI=%.6f kD=%.6f kF=%.6f",
    pid_kp_, pid_ki_, pid_kd_, pid_kf_);

  // These are the Per-joint params (from <joint><param> tags)
  wheels_.resize(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    auto & joint = info_.joints[i];
    auto & wheel = wheels_[i];

    wheel.joint_name = joint.name;
    wheel.can_id     = static_cast<uint8_t>(std::stoi(joint.parameters.at("can_id")));
    wheel.inverted   = (joint.parameters.at("inverted") == "true");

    // Make sure each joint has exactly 1 command interface (velocity) and 4 state interfaces (position, velocity, voltage, online)
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(rclcpp::get_logger("SparkCanHardware"),
        "Joint '%s' must have exactly one velocity command interface.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 4) {
      RCLCPP_ERROR(rclcpp::get_logger("SparkCanHardware"),
        "Joint '%s' must have exactly four state interfaces (position, velocity, voltage, online).",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("SparkCanHardware"),
      "  joint: %-30s  CAN ID: %d  inverted: %s",
      joint.name.c_str(), wheel.can_id, wheel.inverted ? "yes" : "no");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// load_pid_from_yaml
// Reads gains from config/sparks.yaml (path passed in via the spark_config
// hardware param). Any missing key or read error leaves that gain at its
// header default; this must never abort startup.

void SparkCanHardware::load_pid_from_yaml(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    RCLCPP_WARN(rclcpp::get_logger("SparkCanHardware"),
      "Could not read spark_config '%s' (%s): using default PID gains.",
      path.c_str(), e.what());
    return;
  }

  const YAML::Node pid = root["spark_config"] ? root["spark_config"]["pid"]
                                              : YAML::Node();
  if (!pid) {
    RCLCPP_WARN(rclcpp::get_logger("SparkCanHardware"),
      "spark_config '%s' has no spark_config.pid section, using default gains.",
      path.c_str());
    return;
  }

  // Override only the keys that are present; leave the rest at header defaults.
  if (pid["kp"]) pid_kp_ = pid["kp"].as<float>();
  if (pid["ki"]) pid_ki_ = pid["ki"].as<float>();
  if (pid["kd"]) pid_kd_ = pid["kd"].as<float>();
  if (pid["kf"]) pid_kf_ = pid["kf"].as<float>();

  RCLCPP_INFO(rclcpp::get_logger("SparkCanHardware"),
    "Loaded PID gains from %s", path.c_str());
}

// apply_config
// Full one-time configuration for one SPARK. Also re-run when a SPARK comes
// back after a dropout, since a rebooted SPARK only has its flash config.

void SparkCanHardware::apply_config(WheelHandle & wheel, bool burn_flash)
{
  // NEO V1.1 is brushless
  wheel.spark->SetMotorType(MotorType::kBrushless);

  // Brake mode: when velocity command is zero, actively resist motion.
  wheel.spark->SetIdleMode(IdleMode::kBrake);

  // Velocity closed-loop control mode on the SPARK's onboard PID.
  wheel.spark->SetCtrlType(CtrlType::kVelocity);

  // Motor direction (left side inverted so positive = forward for both sides)
  wheel.spark->SetInverted(wheel.inverted);

  // Leave native units (RPM / rotations) so raw CAN shows real motor RPM
  wheel.spark->SetVelocityConversionFactor(1.0);
  wheel.spark->SetPositionConversionFactor(1.0);

  // Onboard velocity PID, slot 0. kF does most of the work.
  wheel.spark->SetP(0, pid_kp_);
  wheel.spark->SetI(0, pid_ki_);
  wheel.spark->SetD(0, pid_kd_);
  wheel.spark->SetF(0, pid_kf_);

  if (burn_flash) {
    wheel.spark->BurnFlash();
  }

  wheel.spark->ClearStickyFaults();
  wheel.configured_at = std::chrono::steady_clock::now();
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

  std::vector<uint8_t> ids;
  for (const auto & wheel : wheels_) ids.push_back(wheel.can_id);
  try {
    status_reader_.open(can_interface_, ids);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("SparkCanHardware"), "%s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto & wheel : wheels_) {
    try {
      wheel.spark = std::make_unique<SparkMax>(can_interface_, wheel.can_id);
      apply_config(wheel, burn_flash_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("SparkCanHardware"),
        "  CAN %d (%s): FAILED: %s", wheel.can_id, wheel.joint_name.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Presence check
  // CAN ACKs are bus-wide, so the setters above succeed as long as ANY device
  // is on the bus. Status 1 frames arrive every ~20 ms per SPARK, so 500 ms
  // gives plenty of chances to hear from each one.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  size_t online_count = 0;
  for (auto & wheel : wheels_) {
    const SparkStatus s = status_reader_.get(wheel.can_id);
    wheel.online = s.seen;
    if (wheel.online) {
      RCLCPP_INFO(rclcpp::get_logger("SparkCanHardware"),
        "  CAN %d (%s): ONLINE, bus %.2f V",
        wheel.can_id, wheel.joint_name.c_str(), s.voltage);
      ++online_count;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("SparkCanHardware"),
        "  CAN %d (%s): OFFLINE, no status frames received",
        wheel.can_id, wheel.joint_name.c_str());
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("SparkCanHardware"),
    "Configure complete: %zu/%zu SPARK MAXs responding.",
    online_count, wheels_.size());

  // Pre-flight gate: refuse to configure (so the controller never activates)
  // if any SPARK is missing and require_all_sparks is set.
  if (require_all_sparks_ && online_count < wheels_.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("SparkCanHardware"),
      "Only %zu/%zu SPARK MAXs online and require_all_sparks is true, "
      "refusing to configure. Check CAN wiring/power, or set the "
      "require_all_sparks URDF param to false to allow limping up.",
      online_count, wheels_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

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
    // Not used by diff_drive; read by storm_telemetry via /dynamic_joint_states
    interfaces.emplace_back(wheel.joint_name, "voltage", &wheel.state_voltage);
    interfaces.emplace_back(wheel.joint_name, "online", &wheel.state_online);
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
  RCLCPP_INFO(rclcpp::get_logger("SparkCanHardware"), "Activating, zeroing commands.");
  for (auto & wheel : wheels_) {
    wheel.cmd_vel   = 0.0;
    wheel.state_pos = 0.0;
    wheel.state_vel = 0.0;
    wheel.state_voltage = 0.0;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SparkCanHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SparkCanHardware"), "Deactivating, stopping motors.");
  stop_all();
  return hardware_interface::CallbackReturn::SUCCESS;
}

//on_cleanup
hardware_interface::CallbackReturn SparkCanHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SparkCanHardware"), "Cleaning up, releasing SPARK handles.");
  stop_all();
  for (auto & wheel : wheels_) {
    wheel.spark.reset();
  }
  status_reader_.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

//read()
// Called every cycle BEFORE the controllers run.
// Pulls status from our own CAN reader and converts to wheel-frame SI units.
// A SPARK with no status 1 frame for status_timeout_ms is offline: the rest
// keep driving (limp mode) and it's reported via the "online" interface.

hardware_interface::return_type SparkCanHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const auto now = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::milliseconds(status_timeout_ms_);

  for (auto & wheel : wheels_) {
    const SparkStatus s = status_reader_.get(wheel.can_id);
    const bool online = s.seen && (now - s.last_status1) < timeout;

    if (wheel.online && !online) {
      RCLCPP_WARN(rclcpp::get_logger("SparkCanHardware"),
        "CAN %d (%s) dropped off the bus, limping on the rest.",
        wheel.can_id, wheel.joint_name.c_str());
    } else if (!wheel.online && online) {
      RCLCPP_INFO(rclcpp::get_logger("SparkCanHardware"),
        "CAN %d (%s) is back, re-applying config.",
        wheel.can_id, wheel.joint_name.c_str());
      wheel.needs_config = true;
    }
    wheel.online = online;
    wheel.state_online = online ? 1.0 : 0.0;

    if (!online) {
      // Keep the last position; a frozen velocity would fake motion in odometry
      wheel.state_vel = 0.0;
      continue;
    }

    // Sticky "has reset" means it rebooted (e.g. brownout) faster than the
    // timeout. Grace period lets ClearStickyFaults from the last config land.
    if ((s.sticky_faults & kFaultHasReset) && !wheel.needs_config &&
        now - wheel.configured_at > std::chrono::milliseconds(500))
    {
      RCLCPP_WARN(rclcpp::get_logger("SparkCanHardware"),
        "CAN %d (%s) reset, re-applying config.",
        wheel.can_id, wheel.joint_name.c_str());
      wheel.needs_config = true;
    }

    // Convert to wheel-frame SI units for ros2_control:
    //   wheel rad/s = motor_rpm x (2pi/60) / gear_ratio
    //   wheel rad   = motor_rotations x 2pi / gear_ratio
    wheel.state_vel = static_cast<double>(s.velocity_rpm) * rads_per_rpm_;
    wheel.state_pos = static_cast<double>(s.position_rot) * rad_per_rot_;
    wheel.state_voltage = static_cast<double>(s.voltage);
  }

  return hardware_interface::return_type::OK;
}

//write()
// Called every cycle AFTER the controllers run.
// Converts wheel rad/s commands to motor RPM and sends to each SPARK.

hardware_interface::return_type SparkCanHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  static rclcpp::Clock clock(RCL_STEADY_TIME);

  try {
    //Send a single static heartbeat frame to keep all CAN motor watchdogs happy
    SparkBase::Heartbeat();

    //Update the control commands for each wheel
    for (auto & wheel : wheels_) {
      if (!wheel.spark || !wheel.online) continue;

      if (wheel.needs_config) {
        apply_config(wheel, false);
        wheel.needs_config = false;
      }

      // Guard against a NaN/inf command (e.g. a misbehaving controller):
      // never forward a non-finite setpoint to the motor; hard-stop that
      // wheel instead so bad math can't run a motor open-loop.
      if (!std::isfinite(wheel.cmd_vel)) {
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("SparkCanHardware"),
          clock, 1000, "Non-finite command for CAN %d (%s): stopping wheel.",
          wheel.can_id, wheel.joint_name.c_str());
        wheel.spark->SetDutyCycle(0.0f);
        continue;
      }

      // Convert wheel angular velocity (rad/s) to motor shaft RPM
      float motor_rpm = static_cast<float>(wheel.cmd_vel * rpm_per_rads_);

      // Send the updated target velocity
      wheel.spark->SetVelocity(motor_rpm);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("SparkCanHardware"),
      clock, 1000, "write() failed: %s", e.what());
    return hardware_interface::return_type::ERROR;
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
