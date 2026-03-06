cmake_minimum_required(VERSION 3.8)
project(top_hfsm_executors_v2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MY_ROBOT_INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/../../robots/engineer/engineer_interface")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(engineer_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(error_code_utils REQUIRED)
find_package(top_hfsm_core_v2 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/executors/arm_solve_client.cpp
  src/executors/vision_detect_client.cpp
  src/executors/gripper_control_node.cpp
  src/executors/moveit_servo_client.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${MY_ROBOT_INTERFACE}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  engineer_interfaces
  rclcpp_action
  std_srvs
  control_msgs
  error_code_utils
  top_hfsm_core_v2
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  engineer_interfaces
  rclcpp_action
  std_srvs
  control_msgs
  error_code_utils
  top_hfsm_core_v2
)

ament_package()
