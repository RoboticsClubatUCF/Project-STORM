#ifndef STORM_HARDWARE__SPARK_CAN_HARDWARE_HPP_
#define STORM_HARDWARE__SPARK_CAN_HARDWARE_HPP_
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <SparkMax.hpp>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp_lifecycle/state.hpp"
namespace storm_hardware {
struct Wheel {
  std::string name;
  uint8_t id;
  bool inverted;
  std::unique_ptr<SparkMax> motor;
};
class SparkCanHardware : public hardware_interface::SystemInterface {
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
private:
  void stop() noexcept;
  std::vector<Wheel> wheels_;
  std::string can_interface_;
  double ratio_{81.0};
  float kp_{0.0001f}, ki_{0}, kd_{0}, kf_{0.000176f};
};
}  // namespace storm_hardware
#endif
