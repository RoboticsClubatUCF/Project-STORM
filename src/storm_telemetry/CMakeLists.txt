cmake_minimum_required(VERSION 3.8)
project(storm_telemetry)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(storm_interfaces REQUIRED)

add_executable(can_health_node src/can_health_node.cpp)
ament_target_dependencies(can_health_node rclcpp diagnostic_updater diagnostic_msgs)

add_executable(system_monitor_node src/system_monitor_node.cpp)
ament_target_dependencies(system_monitor_node rclcpp storm_interfaces)

add_executable(battery_monitor_node src/battery_monitor_node.cpp)
ament_target_dependencies(battery_monitor_node rclcpp control_msgs storm_interfaces)

add_executable(spark_monitor_node src/spark_monitor_node.cpp)
ament_target_dependencies(spark_monitor_node rclcpp control_msgs diagnostic_msgs)

install(TARGETS can_health_node system_monitor_node battery_monitor_node spark_monitor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
