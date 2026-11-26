cmake_minimum_required(VERSION 3.16)
project(storm_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
