cmake_minimum_required(VERSION 3.8)
project(storm_basestation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch lichtblick
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/hotspot_up.sh scripts/check_link.sh scripts/link_up.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
