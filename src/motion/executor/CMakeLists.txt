cmake_minimum_required(VERSION 3.8)
project(executor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(calculator REQUIRED)
find_package(collision REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(params_utils REQUIRED)
find_package(planner REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(types REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/executor.cpp
  src/detail.cpp
  src/test.cpp
  src/time_params.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  calculator
  collision
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  params_utils
  planner
  rclcpp
  tf2
  tf2_eigen
  tf2_geometry_msgs
  types
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  calculator
  collision
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  params_utils
  planner
  rclcpp
  tf2
  tf2_eigen
  tf2_geometry_msgs
  types
)

ament_package()
