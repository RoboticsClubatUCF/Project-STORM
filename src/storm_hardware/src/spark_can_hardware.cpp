// Derived from Project-STORM's original SparkCanHardware, MIT licensed.
#include "storm_hardware/spark_can_hardware.hpp"
#include <chrono>
#include <cmath>
#include <set>
#include <stdexcept>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"
namespace storm_hardware {
using Callback = hardware_interface::CallbackReturn;
using Result = hardware_interface::return_type;
constexpr double two_pi = 6.28318530717958647692;
Callback SparkCanHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams & params) {
  if (SystemInterface::on_init(params) != Callback::SUCCESS) return Callback::ERROR;
  try {
    const auto & p = info_.hardware_parameters;
    can_interface_ = p.at("can_interface");
    ratio_ = std::stod(p.at("gear_ratio"));
    kp_ = std::stof(p.at("pid_kp")); ki_ = std::stof(p.at("pid_ki"));
    kd_ = std::stof(p.at("pid_kd")); kf_ = std::stof(p.at("pid_kf"));
    if (can_interface_.empty() || !std::isfinite(ratio_) || ratio_ <= 0 ||
        !std::isfinite(kp_) || !std::isfinite(ki_) || !std::isfinite(kd_) || !std::isfinite(kf_) || info_.joints.size() != 6)
      throw std::invalid_argument("Expected valid CAN parameters and six wheels");
    std::set<int> ids;
    wheels_.clear();
    for (const auto & joint : info_.joints) {
      std::set<std::string> states;
      for (const auto & state : joint.state_interfaces) states.insert(state.name);
      if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != "velocity" ||
          joint.state_interfaces.size() != 2 || states != std::set<std::string>{"position", "velocity"})
        throw std::invalid_argument("Wheel interfaces must be velocity command and position/velocity state");
      int id = std::stoi(joint.parameters.at("can_id"));
      auto inverted = joint.parameters.at("inverted");
      if (id < 1 || id > 62 || !ids.insert(id).second || (inverted != "true" && inverted != "false"))
        throw std::invalid_argument("CAN IDs must be unique in 1..62; inverted must be boolean");
      wheels_.push_back(Wheel{joint.name, static_cast<uint8_t>(id), inverted == "true", nullptr});
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Invalid STORM hardware configuration: %s", e.what());
    return Callback::ERROR;
  }
  return Callback::SUCCESS;
}
Callback SparkCanHardware::on_configure(const rclcpp_lifecycle::State &) {
  try {
    for (auto & wheel : wheels_) {
      wheel.motor = std::make_unique<SparkMax>(can_interface_, wheel.id);
      wheel.motor->SetMotorType(MotorType::kBrushless);
      wheel.motor->SetIdleMode(IdleMode::kBrake);
      wheel.motor->SetCtrlType(CtrlType::kVelocity);
      wheel.motor->SetInverted(wheel.inverted);
      wheel.motor->SetVelocityConversionFactor(1.0);
      wheel.motor->SetPositionConversionFactor(1.0);
      wheel.motor->SetP(0, kp_); wheel.motor->SetI(0, ki_);
      wheel.motor->SetD(0, kd_); wheel.motor->SetF(0, kf_);
      wheel.motor->SetVelocity(0.0f);
      // No BurnFlash: routine ROS lifecycle changes must not rewrite motor flash.
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    for (auto & wheel : wheels_) {
      float voltage = wheel.motor->GetVoltage();
      if (!std::isfinite(voltage) || voltage <= 5.0f)
        throw std::runtime_error("Missing SPARK status on CAN ID " + std::to_string(wheel.id));
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "STORM CAN configuration failed: %s", e.what());
    stop();
    for (auto & wheel : wheels_) wheel.motor.reset();
    return Callback::ERROR;
  }
  return Callback::SUCCESS;
}
Callback SparkCanHardware::on_activate(const rclcpp_lifecycle::State &) {
  for (auto & wheel : wheels_) {
    if (!wheel.motor) return Callback::ERROR;
    set_command(wheel.name + "/velocity", 0.0);
    wheel.motor->SetVelocity(0.0f);
  }
  return Callback::SUCCESS;
}
void SparkCanHardware::stop() noexcept {
  for (auto & wheel : wheels_) {
    if (wheel.motor) {try {wheel.motor->SetDutyCycle(0.0f);} catch (...) {}}
  }
}
Callback SparkCanHardware::on_deactivate(const rclcpp_lifecycle::State &) {stop(); return Callback::SUCCESS;}
Callback SparkCanHardware::on_cleanup(const rclcpp_lifecycle::State &) {
  stop(); for (auto & wheel : wheels_) wheel.motor.reset(); return Callback::SUCCESS;
}
Callback SparkCanHardware::on_shutdown(const rclcpp_lifecycle::State & state) {return on_cleanup(state);}
Callback SparkCanHardware::on_error(const rclcpp_lifecycle::State & state) {return on_cleanup(state);}
Result SparkCanHardware::read(const rclcpp::Time &, const rclcpp::Duration &) {
  try {
    for (auto & wheel : wheels_) {
      if (!wheel.motor) return Result::ERROR;
      double rpm = wheel.motor->GetVelocity(), rotations = wheel.motor->GetPosition();
      if (!std::isfinite(rpm) || !std::isfinite(rotations)) {stop(); return Result::ERROR;}
      set_state(wheel.name + "/position", rotations * two_pi / ratio_);
      set_state(wheel.name + "/velocity", rpm * two_pi / (60.0 * ratio_));
    }
  } catch (const std::exception & e) {RCLCPP_ERROR(get_logger(), "%s", e.what()); stop(); return Result::ERROR;}
  return Result::OK;
}
Result SparkCanHardware::write(const rclcpp::Time &, const rclcpp::Duration &) {
  try {
    // Validate all commands before issuing any motor setpoints.
    for (auto & wheel : wheels_) {
      if (!wheel.motor || !std::isfinite(get_command(wheel.name + "/velocity"))) {stop(); return Result::ERROR;}
    }
    SparkBase::Heartbeat();
    for (auto & wheel : wheels_)
      wheel.motor->SetVelocity(static_cast<float>(get_command(wheel.name + "/velocity") * 60.0 * ratio_ / two_pi));
  } catch (const std::exception & e) {RCLCPP_ERROR(get_logger(), "%s", e.what()); stop(); return Result::ERROR;}
  return Result::OK;
}
}  // namespace storm_hardware
PLUGINLIB_EXPORT_CLASS(storm_hardware::SparkCanHardware, hardware_interface::SystemInterface)
