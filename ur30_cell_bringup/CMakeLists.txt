cmake_minimum_required(VERSION 3.5)
project(ur30_cell_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
