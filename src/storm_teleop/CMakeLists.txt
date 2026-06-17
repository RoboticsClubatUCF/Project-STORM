cmake_minimum_required(VERSION 3.8)
project(storm_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------- dependencies ----------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# sparkcan: system library installed via PPA (sudo apt install sparkcan).
# It installs headers to /usr/local/include and libsparkcan.so to /usr/local/lib.
# Adjust these paths if your install location differs.
find_library(SPARKCAN_LIB sparkcan
  PATHS /usr/local/lib /usr/lib
  REQUIRED
)
find_path(SPARKCAN_INCLUDE_DIR SparkMax.hpp
  PATHS /usr/local/include /usr/include
  REQUIRED
)

# ---------- hardware interface shared library ----------
add_library(spark_can_hardware SHARED
  src/spark_can_hardware.cpp
)
target_include_directories(spark_can_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SPARKCAN_INCLUDE_DIR}
)
target_link_libraries(spark_can_hardware ${SPARKCAN_LIB})
ament_target_dependencies(spark_can_hardware
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# Pluginlib needs this to find the .xml at runtime
pluginlib_export_plugin_description_file(hardware_interface spark_can_hardware.xml)

# ---------- install ----------

# Plugin library
install(TARGETS spark_can_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Header files
install(DIRECTORY include/
  DESTINATION include
)

# Config, launch, description, plugin XML
install(DIRECTORY config description launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES spark_can_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(spark_can_hardware)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

ament_package()
