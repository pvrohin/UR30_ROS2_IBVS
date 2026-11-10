cmake_minimum_required(VERSION 3.5)
project(ur30_cell_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/ibvs_test_runner.py scripts/part_shifter.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ibvs_test_runner test/test_ibvs_test_runner.py)
  ament_add_pytest_test(test_part_shifter test/test_part_shifter.py)
endif()

ament_package()
