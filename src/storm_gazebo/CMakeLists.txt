cmake_minimum_required(VERSION 3.16)
project(storm_gazebo)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf worlds config terrain launch DESTINATION share/${PROJECT_NAME})

ament_package()
