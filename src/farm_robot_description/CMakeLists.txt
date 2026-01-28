cmake_minimum_required(VERSION 3.8)
project(farm_robot_description)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/delete_entity.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY urdf worlds launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
