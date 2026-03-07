cmake_minimum_required(VERSION 3.8)
project(top_hfsm_node_v2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(engineer_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(error_code_utils REQUIRED)
find_package(top_hfsm_core_v2 REQUIRED)
find_package(top_hfsm_executors_v2 REQUIRED)

add_library(${PROJECT_NAME}_component SHARED
  src/node/top_hfsm_node.cpp
)

target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  std_msgs
  tf2_geometry_msgs
  engineer_interfaces
  rclcpp_action
  error_code_utils
  top_hfsm_core_v2
  top_hfsm_executors_v2
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN top_hfsm_v2::TopHFSMNode
  EXECUTABLE top_hfsm_node_v2
)

install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
