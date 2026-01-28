cmake_minimum_required(VERSION 3.8)
project(farm_robot_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/navigate_row.py
  scripts/monitor_farm.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
