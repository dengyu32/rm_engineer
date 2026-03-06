cmake_minimum_required(VERSION 3.8)
project(ee_path_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(ee_path_node src/ee_path_node.cpp)
ament_target_dependencies(ee_path_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS ee_path_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
