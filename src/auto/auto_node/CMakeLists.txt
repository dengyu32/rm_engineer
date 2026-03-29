cmake_minimum_required(VERSION 3.8)
project(auto_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(params_utils REQUIRED)
find_package(engineer_interfaces REQUIRED)
find_package(task_orchestrator REQUIRED)
find_package(step_executor REQUIRED)
find_package(vision_detect_client REQUIRED)
find_package(slot_select_node REQUIRED)
find_package(gripper_control_node REQUIRED)
find_package(arm_solve_client REQUIRED)

add_library(${PROJECT_NAME}_component SHARED
  src/auto_node.cpp
  ../capabilities/auto_bridge/src/capability_registry.cpp
  ../capabilities/auto_bridge/src/arm_capability_bridge.cpp
  ../capabilities/auto_bridge/src/gripper_capability_bridge.cpp
  ../capabilities/auto_bridge/src/slot_capability_bridge.cpp
  ../capabilities/auto_bridge/src/vision_capability_bridge.cpp
)

target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../capabilities/auto_bridge/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../step_executor/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  std_msgs
  params_utils
  engineer_interfaces
  task_orchestrator
  step_executor
  vision_detect_client
  slot_select_node
  gripper_control_node
  arm_solve_client
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN engineer_auto::AutoNode
  EXECUTABLE auto_node_main
)

install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
