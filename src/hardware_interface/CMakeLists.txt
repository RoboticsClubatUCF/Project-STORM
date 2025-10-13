cmake_minimum_required(VERSION 3.8)
project(drivetrain_hardware)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sparkcan REQUIRED) 

# Build shared library
add_library(${PROJECT_NAME} SHARED
        src/drivetrain_hardware.cpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE sparkcan)

target_include_directories(${PROJECT_NAME}
        PRIVATE
        include
        include/${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        sparkcan
)

# Plugin description
pluginlib_export_plugin_description_file(hardware_interface plugin_description.xml)

# Install targets
install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(
        DIRECTORY include
        DESTINATION include
)

install(
        FILES plugin_description.xml
        DESTINATION share/${PROJECT_NAME}
)

# Export
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        sparkcan
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Package
ament_package()
