#include "gtest/gtest.h"
#include "storm_hardware/spark_can_hardware.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/component_parser.hpp"
namespace {
hardware_interface::HardwareComponentInterfaceParams configuration() {
  std::string xml=R"(<robot name="storm"><link name="base_link"/><ros2_control name="StormSystem" type="system"><hardware><plugin>storm_hardware/SparkCanHardware</plugin>
    <param name="can_interface">can0</param><param name="gear_ratio">81</param>
    <param name="pid_kp">0.0001</param><param name="pid_ki">0</param><param name="pid_kd">0</param><param name="pid_kf">0.000176</param></hardware>)";
  for (int i=1;i<=6;++i) xml+="<joint name='wheel"+std::to_string(i)+"'><param name='can_id'>"+std::to_string(i)+"</param><param name='inverted'>false</param><command_interface name='velocity'/><state_interface name='position'/><state_interface name='velocity'/></joint>";
  xml+="</ros2_control></robot>";
  std::string geometry;
  for (int i=1;i<=6;++i) {
    const auto name = "wheel" + std::to_string(i);
    geometry += "<link name='" + name + "_link'/><joint name='" + name +
      "' type='continuous'><parent link='base_link'/><child link='" + name +
      "_link'/><axis xyz='0 1 0'/></joint>";
  }
  xml.insert(xml.find("<ros2_control"), geometry);
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info=hardware_interface::parse_control_resources_from_urdf(xml).at(0);
  return params;
}
class HardwareInit : public ::testing::Test {
  void SetUp() override {if(!rclcpp::ok()) rclcpp::init(0,nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};
TEST_F(HardwareInit, AcceptsSixWheelsWithoutOpeningCAN) {
  storm_hardware::SparkCanHardware driver;
  EXPECT_EQ(driver.on_init(configuration()),hardware_interface::CallbackReturn::SUCCESS);
}
TEST_F(HardwareInit, RejectsInvalidGearRatio) {
  auto p=configuration();p.hardware_info.hardware_parameters["gear_ratio"]="0";
  storm_hardware::SparkCanHardware driver;
  EXPECT_EQ(driver.on_init(p),hardware_interface::CallbackReturn::ERROR);
}
TEST_F(HardwareInit, RejectsDuplicateMotorIDs) {
  auto p=configuration();p.hardware_info.joints[1].parameters["can_id"]="1";
  storm_hardware::SparkCanHardware driver;
  EXPECT_EQ(driver.on_init(p),hardware_interface::CallbackReturn::ERROR);
}
TEST_F(HardwareInit, RejectsWrongStateInterfaces) {
  auto p=configuration();p.hardware_info.joints[0].state_interfaces[1].name="effort";
  storm_hardware::SparkCanHardware driver;
  EXPECT_EQ(driver.on_init(p),hardware_interface::CallbackReturn::ERROR);
}
}
