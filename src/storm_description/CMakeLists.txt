cmake_minimum_required(VERSION 3.16)
project(storm_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(description_contract test/test_description.py)
endif()

ament_package()
