cmake_minimum_required(VERSION 3.8)
project(ur30_ibvs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
# ros-jazzy-visp (3.7) exports plain variables, not an imported target:
# use ${VISP_INCLUDE_DIRS} and ${VISP_LIBRARIES}.
find_package(VISP REQUIRED)

# Plain C++ helper classes with no ROS dependency, so they can be unit tested.
add_library(ur30_ibvs_core
  src/panel_detector.cpp
  src/edge_tracker.cpp
  src/ibvs_controller.cpp
  src/panel_geometry.cpp
  src/servo_supervisor.cpp)
target_include_directories(ur30_ibvs_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS})
target_link_libraries(ur30_ibvs_core ${OpenCV_LIBS} ${VISP_LIBRARIES})

install(TARGETS ur30_ibvs_core LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_panel_detector test/test_panel_detector.cpp)
  target_link_libraries(test_panel_detector ur30_ibvs_core)
  ament_add_gtest(test_edge_tracker test/test_edge_tracker.cpp)
  target_link_libraries(test_edge_tracker ur30_ibvs_core)
  ament_add_gtest(test_ibvs_controller test/test_ibvs_controller.cpp)
  target_link_libraries(test_ibvs_controller ur30_ibvs_core)
  ament_add_gtest(test_panel_geometry test/test_panel_geometry.cpp)
  target_link_libraries(test_panel_geometry ur30_ibvs_core)
  ament_add_gtest(test_servo_supervisor test/test_servo_supervisor.cpp)
  target_link_libraries(test_servo_supervisor ur30_ibvs_core)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ibvs_test_runner test/test_ibvs_test_runner.py)
endif()

add_executable(ibvs_node src/ibvs_node_main.cpp src/ibvs_state_machine.cpp)
target_link_libraries(ibvs_node ur30_ibvs_core)
ament_target_dependencies(ibvs_node
  rclcpp
  rclcpp_action
  cv_bridge
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  control_msgs
  controller_manager_msgs
  moveit_msgs
  tf2_ros)

install(TARGETS ibvs_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/ibvs_test_runner.py DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
