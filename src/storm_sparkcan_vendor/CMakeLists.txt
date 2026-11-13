cmake_minimum_required(VERSION 3.16)
project(storm_sparkcan_vendor)
find_package(ament_cmake REQUIRED)
option(STORM_BUILD_CAN_DRIVER "Build the CAN library and driver" OFF)
if(STORM_BUILD_CAN_DRIVER)
  include(FetchContent)
  FetchContent_Declare(sparkcan
    GIT_REPOSITORY https://github.com/grayson-arendt/sparkcan.git
    GIT_TAG 11d2d13787fc32e508690926efc3c9749e58770e
  )
  FetchContent_MakeAvailable(sparkcan)
  install(FILES "${sparkcan_SOURCE_DIR}/LICENSE" DESTINATION share/${PROJECT_NAME})
endif()
ament_package()
