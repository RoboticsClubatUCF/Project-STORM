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
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &state) override;
  hardware_interface::return_type read(const rclcpp::Time &state,
                                       const rclcpp::Duration &duration) override;
  hardware_interface::return_type write(const rclcpp::Time &state,
                                        const rclcpp::Duration &duration) override;

private:
  void stop() noexcept;
  std::vector<Wheel> wheels_;
  std::string can_interface_;
  
  double ratio_{81.0};
  
  static constexpr float kDefaultKp{0.0001F};
  static constexpr float kDefaultKi{0.0F};
  static constexpr float kDefaultKd{0.0F};
  static constexpr float kDefaultKf{0.000176F};

  float kp_{kDefaultKp}, ki_{kDefaultKi}, kd_{kDefaultKd}, kf_{kDefaultKf};
};
}  // namespace storm_hardware
#endif
